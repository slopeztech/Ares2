#!/usr/bin/env python3
"""ARES WiFi filesystem and status console.

Textual TUI tool that:
- Detects whether host is connected to ARES WiFi (SSID prefix check).
- Shows current board status from /api/status.
- Lists files from /api/logs and /api/missions.
- Shows remote file preview for selected item.
- Supports download/delete for logs and missions.
- Supports upload for mission files (.ams) via PUT /api/missions/<name>.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

try:
    from textual.app import App, ComposeResult
    from textual.containers import Container, Horizontal
    from textual.widgets import Button, DataTable, Footer, Header, Input, RichLog, Static
except ImportError as exc:  # pragma: no cover - import guard
    raise SystemExit("textual is not installed. Run: pip install textual") from exc


DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 80
DEFAULT_TIMEOUT_S = 2.0
DEFAULT_REFRESH_S = 2.0
DEFAULT_DOWNLOAD_DIR = "downloads"
DEFAULT_SSID_PREFIX = "ARES-"
DEFAULT_TOKEN = ""


@dataclass(frozen=True)
class AppConfig:
    host: str
    port: int
    timeout_s: float
    refresh_s: float
    download_dir: str
    ssid_prefix: str
    enforce_ssid_check: bool
    token: str = ""


@dataclass(frozen=True)
class FsEntry:
    source: str  # logs | missions
    name: str
    size: int


class ApiClient:
    def __init__(self, host: str, port: int, timeout_s: float, token: str = "") -> None:
        self.base = f"http://{host}:{port}"
        self.timeout_s = timeout_s
        self._token = token

    def _request(self, method: str, path: str, data: Optional[bytes] = None,
                 content_type: Optional[str] = None) -> tuple[int, bytes]:
        headers: dict[str, str] = {}
        if content_type is not None:
            headers["Content-Type"] = content_type
        if self._token:
            headers["X-ARES-Token"] = self._token

        req = Request(self.base + path, data=data, method=method, headers=headers)
        try:
            with urlopen(req, timeout=self.timeout_s) as resp:
                body = resp.read()
                return int(resp.status), body
        except HTTPError as exc:
            return int(exc.code), exc.read()
        except URLError as exc:
            raise ConnectionError(str(exc)) from exc

    def get_json(self, path: str) -> tuple[int, Any]:
        status, body = self._request("GET", path)
        if len(body) == 0:
            return status, None
        try:
            return status, json.loads(body.decode("utf-8", errors="replace"))
        except json.JSONDecodeError:
            return status, {"raw": body.decode("utf-8", errors="replace")}

    def get_bytes(self, path: str) -> tuple[int, bytes]:
        return self._request("GET", path)

    def put_bytes(self, path: str, data: bytes, content_type: str = "text/plain") -> tuple[int, bytes]:
        return self._request("PUT", path, data=data, content_type=content_type)

    def delete(self, path: str) -> tuple[int, bytes]:
        return self._request("DELETE", path)


def current_wifi_ssid() -> Optional[str]:
    """Return current WLAN SSID on Windows, else None."""
    try:
        result = subprocess.run(
            ["netsh", "wlan", "show", "interfaces"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
    except Exception:
        return None

    if result.returncode != 0:
        return None

    # Match line like: "    SSID                   : ARES-ABCD"
    for line in result.stdout.splitlines():
        match = re.match(r"^\s*SSID\s*:\s*(.+)$", line)
        if match and "BSSID" not in line:
            ssid = match.group(1).strip()
            if ssid:
                return ssid
    return None


class AresFsApp(App[None]):
    CSS = """
    Screen {
        layout: vertical;
    }

    #root {
        padding: 1;
        height: 1fr;
    }

    #top {
        layout: horizontal;
        height: 11;
        margin-bottom: 1;
    }

    #wifi {
        width: 1fr;
        border: round #3a6ea5;
        padding: 0 1;
        margin-right: 1;
    }

    #status {
        width: 2fr;
        border: round #2f855a;
        padding: 0 1;
    }

    #files {
        height: 14;
        border: round #805ad5;
        margin-bottom: 1;
    }

    #preview {
        height: 1fr;
        border: round #4c8c9f;
        padding: 0 1;
        margin-bottom: 1;
    }

    #controls {
        layout: horizontal;
        height: 3;
        margin-bottom: 1;
    }

    #local_path {
        width: 2fr;
        margin-right: 1;
    }

    #remote_name {
        width: 1fr;
        margin-right: 1;
    }

    #log {
        height: 12;
        border: round #3f8f5f;
    }

    Button {
        margin-right: 1;
    }
    """

    BINDINGS = [
        ("r", "refresh", "Refresh"),
        ("p", "preview", "Preview Selected"),
        ("d", "download", "Download Selected"),
        ("u", "upload", "Upload Mission"),
        ("x", "delete", "Delete Selected"),
        ("q", "quit", "Quit"),
    ]

    def __init__(self, config: AppConfig) -> None:
        super().__init__()
        self.config = config
        self.api = ApiClient(config.host, config.port, config.timeout_s, token=config.token)
        self.entries: list[FsEntry] = []
        self.status_payload: dict[str, Any] = {}
        self.storage_health_payload: dict[str, Any] = {}
        self.connected_ssid: Optional[str] = None
        self.last_refresh = 0.0
        self._last_disconnect_log_s = 0.0

    def is_on_expected_wifi(self) -> bool:
        if not self.config.enforce_ssid_check:
            return True
        return (
            self.connected_ssid is not None
            and self.connected_ssid.startswith(self.config.ssid_prefix)
        )

    def _update_disconnected_status(self) -> None:
        panel = self.query_one("#status", Static)
        panel.update(
            "\n".join(
                [
                    "[b]System Status[/b]",
                    "[bold red]API paused[/]: connect your PC to ARES WiFi first.",
                    f"Expected SSID prefix: [yellow]{self.config.ssid_prefix}[/]",
                    f"Target API: [cyan]{self.config.host}:{self.config.port}[/]",
                ]
            )
        )

    def _log_disconnected_once(self) -> None:
        now = time.time()
        if now - self._last_disconnect_log_s >= 8.0:
            self.write_log(
                "[yellow]WARN[/] WiFi is not on ARES; skipping API calls to avoid timeout spam"
            )
            self._last_disconnect_log_s = now

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Container(id="root"):
            with Horizontal(id="top"):
                yield Static(id="wifi")
                yield Static(id="status")

            yield DataTable(id="files", zebra_stripes=True)

            yield Static(id="preview")

            with Horizontal(id="controls"):
                yield Input(placeholder="Local file path (for upload)", id="local_path")
                yield Input(placeholder="Remote mission name (e.g. flight1.ams)", id="remote_name")
                yield Button("Refresh", id="btn_refresh", variant="primary")
                yield Button("Preview", id="btn_preview")
                yield Button("Download", id="btn_download")
                yield Button("Upload", id="btn_upload")
                yield Button("Delete", id="btn_delete", variant="error")

            yield RichLog(id="log", wrap=True, highlight=True, max_lines=800)

        yield Footer()

    def on_mount(self) -> None:
        self.title = "ARES WiFi FS Console"
        self.sub_title = f"{self.config.host}:{self.config.port}"

        table = self.query_one("#files", DataTable)
        table.add_columns("Source", "Name", "Size", "Actions")
        self.set_interval(self.config.refresh_s, self.refresh_all)
        self.query_one("#preview", Static).update(
            "\n".join(
                [
                    "[b]Remote Preview[/b]",
                    "Select a row and press [b]p[/b] or Preview button.",
                ]
            )
        )
        self.refresh_all()

    def write_log(self, text: str) -> None:
        self.query_one("#log", RichLog).write(text)

    def refresh_all(self) -> None:
        self.connected_ssid = current_wifi_ssid()
        self.update_wifi_panel()
        if not self.is_on_expected_wifi():
            self.entries = []
            self._update_disconnected_status()
            self._log_disconnected_once()
            self.rebuild_table()
            self.last_refresh = time.time()
            return
        self.fetch_status()
        self.fetch_files()
        self.rebuild_table()
        self.last_refresh = time.time()

    def update_wifi_panel(self) -> None:
        panel = self.query_one("#wifi", Static)
        ssid = self.connected_ssid or "<none>"
        on_ares = self.connected_ssid is not None and self.connected_ssid.startswith(self.config.ssid_prefix)

        wifi_state = "[green]CONNECTED[/]" if on_ares else "[bold red]NOT ARES[/]"
        auth_line = "[green]token set[/]" if self.config.token else "[yellow]open (no token)[/]"
        panel.update(
            "\n".join(
                [
                    "[b]WiFi Link[/b]",
                    f"SSID: [cyan]{ssid}[/]",
                    f"Expected prefix: [yellow]{self.config.ssid_prefix}[/]",
                    f"State: {wifi_state}",
                    f"API Host: [cyan]{self.config.host}:{self.config.port}[/]",
                    f"Auth: {auth_line}",
                ]
            )
        )

    def fetch_status(self) -> None:
        panel = self.query_one("#status", Static)
        try:
            code, payload = self.api.get_json("/api/status")
        except ConnectionError as exc:
            self.status_payload = {}
            self.storage_health_payload = {}
            panel.update("\n".join(["[b]System Status[/b]", f"[bold red]API unreachable[/]: {exc}"]))
            return

        if code != 200 or not isinstance(payload, dict):
            self.storage_health_payload = {}
            panel.update("\n".join(["[b]System Status[/b]", f"[bold red]HTTP {code}[/] while reading status"]))
            return

        self.status_payload = payload
        mode = payload.get("mode", "-")
        armed = payload.get("armed", "-")
        uptime = payload.get("uptimeMs", "-")
        free_heap = payload.get("freeHeap", "-")
        clients = payload.get("wifiClients", "-")
        health = payload.get("health", {}) if isinstance(payload.get("health"), dict) else {}

        storage_line = "Storage: [yellow]unavailable[/]"
        recovery_line = "Recovery: [yellow]unavailable[/]"

        try:
            health_code, health_payload = self.api.get_json("/api/storage/health")
            if health_code == 200 and isinstance(health_payload, dict):
                self.storage_health_payload = health_payload

                mounted = health_payload.get("mounted", "-")
                used = health_payload.get("used", "-")
                free = health_payload.get("free", "-")
                total = health_payload.get("total", "-")
                mounted_is_ok = mounted is True
                mounted_style = "green" if mounted_is_ok else "bold red"
                storage_line = (
                    "Storage: "
                    f"mounted=[{mounted_style}]{mounted}[/] "
                    f"used=[cyan]{used}[/] "
                    f"free=[cyan]{free}[/] "
                    f"total=[cyan]{total}[/]"
                )

                scanned = health_payload.get("recovery_scanned", "-")
                restored = health_payload.get("recovered_from_bak", "-")
                rm_tmp = health_payload.get("removed_tmp", "-")
                rm_bak = health_payload.get("removed_bak", "-")
                rec_err = health_payload.get("recovery_errors", "-")
                rec_err_int = 0
                try:
                    rec_err_int = int(rec_err)
                except (TypeError, ValueError):
                    rec_err_int = 0
                rec_err_style = "bold red" if rec_err_int > 0 else "green"
                recovery_line = (
                    "Recovery: "
                    f"scanned=[cyan]{scanned}[/] "
                    f"restored=[cyan]{restored}[/] "
                    f"rm_tmp=[cyan]{rm_tmp}[/] "
                    f"rm_bak=[cyan]{rm_bak}[/] "
                    f"errors=[{rec_err_style}]{rec_err}[/]"
                )
            else:
                self.storage_health_payload = {}
                storage_line = f"Storage: [yellow]HTTP {health_code}[/]"
                recovery_line = "Recovery: [yellow]not available[/]"
        except ConnectionError:
            self.storage_health_payload = {}
            storage_line = "Storage: [yellow]API unreachable[/]"
            recovery_line = "Recovery: [yellow]not available[/]"

        panel.update(
            "\n".join(
                [
                    "[b]System Status[/b]",
                    f"Mode: [cyan]{mode}[/]    Armed: [cyan]{armed}[/]    Uptime(ms): [cyan]{uptime}[/]",
                    f"Free Heap: [cyan]{free_heap}[/]    WiFi Clients: [cyan]{clients}[/]",
                    f"Health: wifi={health.get('wifi', '-')} gps={health.get('gps', '-')} baro={health.get('baro', '-')}",
                    storage_line,
                    recovery_line,
                    "Use [b]r[/b]=refresh  [b]p[/b]=preview  [b]d[/b]=download  [b]u[/b]=upload  [b]x[/b]=delete",
                ]
            )
        )

    def fetch_files(self) -> None:
        self.entries = []

        # Logs
        try:
            code_logs, logs_payload = self.api.get_json("/api/logs")
            if code_logs == 200 and isinstance(logs_payload, list):
                for item in logs_payload:
                    if isinstance(item, dict):
                        name = str(item.get("name", ""))
                        size = int(item.get("size", 0))
                        self.entries.append(FsEntry(source="logs", name=name.split("/")[-1], size=size))
        except Exception as exc:
            self.write_log(f"[yellow]WARN[/] log list failed: {exc}")

        # Missions
        try:
            code_msn, msn_payload = self.api.get_json("/api/missions")
            if code_msn == 200 and isinstance(msn_payload, list):
                for item in msn_payload:
                    if isinstance(item, dict):
                        name = str(item.get("name", ""))
                        size = int(item.get("size", 0))
                        self.entries.append(FsEntry(source="missions", name=name.split("/")[-1], size=size))
        except Exception as exc:
            self.write_log(f"[yellow]WARN[/] mission list failed: {exc}")

    def rebuild_table(self) -> None:
        table = self.query_one("#files", DataTable)
        table.clear(columns=False)

        # Newest-ish view: missions first, then logs, then by name.
        for entry in sorted(self.entries, key=lambda e: (e.source, e.name)):
            actions = "download/delete"
            if entry.source == "missions":
                actions = "download/delete/upload"
            table.add_row(entry.source, entry.name, str(entry.size), actions, key=f"{entry.source}:{entry.name}")

    def get_selected_entry(self) -> Optional[FsEntry]:
        table = self.query_one("#files", DataTable)
        if table.cursor_row < 0 or table.cursor_row >= len(table.rows):
            return None

        row_key = table.coordinate_to_cell_key((table.cursor_row, 0)).row_key
        if row_key is None:
            return None
        key = str(row_key.value)

        for entry in self.entries:
            if f"{entry.source}:{entry.name}" == key:
                return entry
        return None

    def download_selected(self) -> None:
        if not self.is_on_expected_wifi():
            self.write_log("[yellow]Connect to ARES WiFi before download[/]")
            return

        entry = self.get_selected_entry()
        if entry is None:
            self.write_log("[yellow]No file selected[/]")
            return

        download_dir = Path(self.config.download_dir)
        download_dir.mkdir(parents=True, exist_ok=True)
        out_path = download_dir / entry.name

        endpoint = f"/api/{entry.source}/{quote(entry.name)}"
        code, payload = self.api.get_bytes(endpoint)
        if code != 200:
            self.write_log(f"[bold red]Download failed[/] {entry.source}/{entry.name} (HTTP {code})")
            return

        out_path.write_bytes(payload)
        self.write_log(f"[green]Downloaded[/] {entry.source}/{entry.name} -> {out_path}")

    def preview_selected(self) -> None:
        if not self.is_on_expected_wifi():
            self.write_log("[yellow]Connect to ARES WiFi before preview[/]")
            return

        entry = self.get_selected_entry()
        if entry is None:
            self.write_log("[yellow]No file selected[/]")
            return

        endpoint = f"/api/{entry.source}/{quote(entry.name)}"
        code, payload = self.api.get_bytes(endpoint)
        if code != 200:
            self.write_log(f"[bold red]Preview failed[/] {entry.source}/{entry.name} (HTTP {code})")
            return

        preview_panel = self.query_one("#preview", Static)
        preview_text = payload.decode("utf-8", errors="replace")
        max_chars = 4000
        truncated = len(preview_text) > max_chars
        if truncated:
            preview_text = preview_text[:max_chars]

        header = f"[b]Remote Preview[/b] [cyan]{entry.source}/{entry.name}[/] ({entry.size} bytes)"
        if truncated:
            header += " [yellow](truncated)[/]"

        preview_panel.update(f"{header}\n\n{preview_text}")

    def upload_mission(self) -> None:
        if not self.is_on_expected_wifi():
            self.write_log("[yellow]Connect to ARES WiFi before upload[/]")
            return

        local_input = self.query_one("#local_path", Input).value.strip()
        remote_input = self.query_one("#remote_name", Input).value.strip()

        if not local_input:
            self.write_log("[yellow]Local path required for upload[/]")
            return

        local_path = Path(local_input)
        if not local_path.exists() or not local_path.is_file():
            self.write_log(f"[bold red]Local file not found[/]: {local_path}")
            return

        remote_name = remote_input if remote_input else local_path.name
        if not remote_name.endswith(".ams"):
            remote_name += ".ams"

        try:
            payload = local_path.read_bytes()
        except OSError as exc:
            self.write_log(f"[bold red]Read failed[/]: {exc}")
            return

        endpoint = f"/api/missions/{quote(remote_name)}"
        code, _ = self.api.put_bytes(endpoint, payload, content_type="text/plain")
        if code not in (200, 201, 204):
            self.write_log(f"[bold red]Upload failed[/] {remote_name} (HTTP {code})")
            return

        self.write_log(f"[green]Uploaded[/] {local_path} -> missions/{remote_name}")
        self.fetch_files()
        self.rebuild_table()

    def delete_selected(self) -> None:
        if not self.is_on_expected_wifi():
            self.write_log("[yellow]Connect to ARES WiFi before delete[/]")
            return

        entry = self.get_selected_entry()
        if entry is None:
            self.write_log("[yellow]No file selected[/]")
            return

        endpoint = f"/api/{entry.source}/{quote(entry.name)}"
        code, _ = self.api.delete(endpoint)
        if code not in (200, 204):
            self.write_log(f"[bold red]Delete failed[/] {entry.source}/{entry.name} (HTTP {code})")
            return

        self.write_log(f"[green]Deleted[/] {entry.source}/{entry.name}")
        self.fetch_files()
        self.rebuild_table()

    def action_refresh(self) -> None:
        self.refresh_all()

    def action_preview(self) -> None:
        self.preview_selected()

    def action_download(self) -> None:
        self.download_selected()

    def action_upload(self) -> None:
        self.upload_mission()

    def action_delete(self) -> None:
        self.delete_selected()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        btn_id = event.button.id
        if btn_id == "btn_refresh":
            self.action_refresh()
        elif btn_id == "btn_preview":
            self.action_preview()
        elif btn_id == "btn_download":
            self.action_download()
        elif btn_id == "btn_upload":
            self.action_upload()
        elif btn_id == "btn_delete":
            self.action_delete()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ARES WiFi filesystem/status Textual console")
    parser.add_argument("--host", default=DEFAULT_HOST, help="ARES API host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="ARES API port")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="HTTP timeout seconds")
    parser.add_argument("--refresh", type=float, default=DEFAULT_REFRESH_S, help="Refresh interval seconds")
    parser.add_argument("--download-dir", default=DEFAULT_DOWNLOAD_DIR, help="Local download directory")
    parser.add_argument("--ssid-prefix", default=DEFAULT_SSID_PREFIX, help="Expected WiFi SSID prefix")
    parser.add_argument(
        "--ignore-ssid-check",
        action="store_true",
        help="Do not require SSID prefix match before API calls",
    )
    parser.add_argument(
        "--token",
        default=DEFAULT_TOKEN,
        metavar="TOKEN",
        help="API bearer token (X-ARES-Token header). Leave empty for open mode.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    cfg = AppConfig(
        host=args.host,
        port=args.port,
        timeout_s=args.timeout,
        refresh_s=args.refresh,
        download_dir=args.download_dir,
        ssid_prefix=args.ssid_prefix,
        enforce_ssid_check=not args.ignore_ssid_check,
        token=args.token,
    )
    AresFsApp(cfg).run()


if __name__ == "__main__":
    main()
