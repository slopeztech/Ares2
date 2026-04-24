#!/usr/bin/env python3
"""ARES API operations console (Textual).

Features:
- Verifies host WiFi SSID prefix (ARES- by default).
- Live status panel from /api/status and /api/storage/health.
- Executes all firmware API endpoints from a selectable catalog.
- Supports dynamic endpoints with {file} placeholders.
- Also allows custom method/path/body requests.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from dataclasses import dataclass
from typing import Any, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

try:
    from textual.app import App, ComposeResult
    from textual.containers import Container, Horizontal
    from textual.widgets import Button, DataTable, Footer, Header, Input, RichLog, Static
except ImportError as exc:  # pragma: no cover
    raise SystemExit("textual is not installed. Run: pip install textual") from exc


DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 80
DEFAULT_TIMEOUT_S = 2.0
DEFAULT_REFRESH_S = 2.0
DEFAULT_SSID_PREFIX = "ARES-"


@dataclass(frozen=True)
class AppConfig:
    host: str
    port: int
    timeout_s: float
    refresh_s: float
    ssid_prefix: str
    enforce_ssid_check: bool


@dataclass(frozen=True)
class EndpointDef:
    name: str
    method: str
    path_template: str
    body_template: str
    content_type: str
    note: str


ENDPOINTS: list[EndpointDef] = [
    EndpointDef("Status", "GET", "/api/status", "", "application/json", "System snapshot"),
    EndpointDef("Storage health", "GET", "/api/storage/health", "", "application/json", "FS health counters"),
    EndpointDef("Get config", "GET", "/api/config", "", "application/json", "Current runtime config"),
    EndpointDef(
        "Put config",
        "PUT",
        "/api/config",
        '{"telemetryIntervalMs":2000,"nodeId":1,"ledBrightness":32}',
        "application/json",
        "Partial/full config update",
    ),
    EndpointDef("Mode idle", "POST", "/api/mode", '{"mode":"idle"}', "application/json", "Mode transition"),
    EndpointDef("Mode test", "POST", "/api/mode", '{"mode":"test"}', "application/json", "Mode transition"),
    EndpointDef("Mode flight", "POST", "/api/mode", '{"mode":"flight"}', "application/json", "Mode transition"),
    EndpointDef("Arm", "POST", "/api/arm", "", "application/json", "Requires FLIGHT mode"),
    EndpointDef("Abort", "POST", "/api/abort", "", "application/json", "Requires FLIGHT mode"),
    EndpointDef("List logs", "GET", "/api/logs", "", "application/json", "List log files"),
    EndpointDef("Delete all logs", "DELETE", "/api/logs", "", "application/json", "Idle mode only"),
    EndpointDef("Get log file", "GET", "/api/logs/{file}", "", "application/json", "Raw file bytes"),
    EndpointDef("Delete log file", "DELETE", "/api/logs/{file}", "", "application/json", "Idle mode only"),
    EndpointDef("Mission status", "GET", "/api/mission", "", "application/json", "Active mission state"),
    EndpointDef("Mission active alias", "GET", "/api/missions/active", "", "application/json", "Alias to mission status"),
    EndpointDef("List missions", "GET", "/api/missions", "", "application/json", "List .ams files"),
    EndpointDef("Get mission file", "GET", "/api/missions/{file}", "", "application/json", "Download mission text"),
    EndpointDef("Put mission file", "PUT", "/api/missions/{file}", "STATE IDLE:\n  EVERY 2000 -> REPORT\n", "text/plain", "Upload .ams content"),
    EndpointDef("Delete mission file", "DELETE", "/api/missions/{file}", "", "application/json", "Delete mission file"),
    EndpointDef(
        "Mission activate",
        "POST",
        "/api/mission/activate",
        '{"file":"lectura_baro.ams"}',
        "application/json",
        "Activate script",
    ),
    EndpointDef("Mission deactivate", "POST", "/api/mission/deactivate", "{}", "application/json", "Deactivate script"),
    EndpointDef(
        "Mission command",
        "POST",
        "/api/mission/command",
        '{"command":"LAUNCH"}',
        "application/json",
        "Inject TC command",
    ),
]


class ApiClient:
    def __init__(self, host: str, port: int, timeout_s: float) -> None:
        self.base = f"http://{host}:{port}"
        self.timeout_s = timeout_s

    def request(self, method: str, path: str, body: bytes = b"", content_type: str = "application/json") -> tuple[int, bytes]:
        headers: dict[str, str] = {}
        if len(body) > 0:
            headers["Content-Type"] = content_type

        req = Request(self.base + path, data=(body if len(body) > 0 else None), method=method, headers=headers)
        try:
            with urlopen(req, timeout=self.timeout_s) as resp:
                return int(resp.status), resp.read()
        except HTTPError as exc:
            return int(exc.code), exc.read()
        except URLError as exc:
            raise ConnectionError(str(exc)) from exc

    def get_json(self, path: str) -> tuple[int, Any]:
        code, payload = self.request("GET", path)
        if len(payload) == 0:
            return code, None
        try:
            return code, json.loads(payload.decode("utf-8", errors="replace"))
        except json.JSONDecodeError:
            return code, {"raw": payload.decode("utf-8", errors="replace")}


def current_wifi_ssid() -> Optional[str]:
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

    for line in result.stdout.splitlines():
        match = re.match(r"^\s*SSID\s*:\s*(.+)$", line)
        if match and "BSSID" not in line:
            ssid = match.group(1).strip()
            if ssid:
                return ssid
    return None


class AresApiApp(App[None]):
    CSS = """
    Screen {
        layout: vertical;
    }

    #root {
        padding: 1;
        height: 1fr;
        layout: horizontal;
    }

    /* ── Left column ─────────────────────────────────── */
    #left {
        width: 3fr;
        layout: vertical;
        margin-right: 1;
    }

    #top {
        layout: horizontal;
        height: 9;
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

    #endpoints {
        height: 1fr;
        border: round #805ad5;
        margin-bottom: 1;
    }

    #controls {
        layout: vertical;
        height: 6;
    }

    #toolbar {
        layout: horizontal;
        height: 3;
    }

    #bodyrow {
        layout: horizontal;
        height: 3;
    }

    #body_label {
        width: 8;
        height: 3;
        content-align: left middle;
        padding: 0 1;
        color: $text-muted;
    }

    #method_input {
        width: 10;
        margin-right: 1;
    }

    #path_input {
        width: 2fr;
        margin-right: 1;
    }

    #content_type_input {
        width: 14;
        margin-right: 1;
    }

    #file_input {
        width: 14;
        margin-right: 1;
    }

    #body_input {
        width: 1fr;
    }

    Button {
        margin-right: 1;
    }

    /* ── Right column: response pane ─────────────────── */
    #right {
        width: 2fr;
        layout: vertical;
    }

    #response_header {
        height: 1;
        color: $text-muted;
        padding: 0 1;
    }

    #log {
        height: 1fr;
        border: round #3f8f5f;
    }
    """

    BINDINGS = [
        ("r", "refresh", "Refresh"),
        ("e", "execute_selected", "Run Selected"),
        ("c", "execute_custom", "Run Custom"),
        ("q", "quit", "Quit"),
    ]

    def __init__(self, config: AppConfig) -> None:
        super().__init__()
        self.config = config
        self.api = ApiClient(config.host, config.port, config.timeout_s)
        self.connected_ssid: Optional[str] = None
        self._last_disconnect_log_s = 0.0

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="root"):
            with Container(id="left"):
                with Horizontal(id="top"):
                    yield Static(id="wifi")
                    yield Static(id="status")
                yield DataTable(id="endpoints", zebra_stripes=True)
                with Container(id="controls"):
                    with Horizontal(id="toolbar"):
                        yield Input(value="GET", id="method_input", placeholder="Method")
                        yield Input(value="/api/status", id="path_input", placeholder="Path")
                        yield Input(value="application/json", id="content_type_input", placeholder="Content-Type")
                        yield Input(value="", id="file_input", placeholder="{file} value")
                        yield Button("Run [e]", id="btn_run_selected", variant="primary")
                        yield Button("Custom [c]", id="btn_run_custom")
                        yield Button("Refresh [r]", id="btn_refresh")
                    with Horizontal(id="bodyrow"):
                        yield Static("Body:", id="body_label")
                        yield Input(value="", id="body_input", placeholder="JSON body or text payload")
            with Container(id="right"):
                yield Static("[b]API Response[/b]", id="response_header")
                yield RichLog(id="log", wrap=True, highlight=True, max_lines=2000)
        yield Footer()

    def on_mount(self) -> None:
        self.title = "ARES API Console"
        self.sub_title = f"{self.config.host}:{self.config.port}"

        table = self.query_one("#endpoints", DataTable)
        table.add_columns("Name", "Method", "Path", "Content-Type", "Notes")
        for item in ENDPOINTS:
            table.add_row(item.name, item.method, item.path_template, item.content_type, item.note, key=item.name)

        self.set_interval(self.config.refresh_s, self.refresh_live_status)
        self.refresh_live_status()

    def write_log(self, text: str) -> None:
        self.query_one("#log", RichLog).write(text)

    def clear_log(self) -> None:
        self.query_one("#log", RichLog).clear()

    def is_on_expected_wifi(self) -> bool:
        if not self.config.enforce_ssid_check:
            return True
        return self.connected_ssid is not None and self.connected_ssid.startswith(self.config.ssid_prefix)

    def _log_disconnected_once(self) -> None:
        now = time.time()
        if now - self._last_disconnect_log_s >= 8.0:
            self.write_log("[yellow]WARN[/] Not connected to ARES WiFi; API calls paused")
            self._last_disconnect_log_s = now

    def update_wifi_panel(self) -> None:
        panel = self.query_one("#wifi", Static)
        ssid = self.connected_ssid or "<none>"
        on_ares = self.is_on_expected_wifi()
        state = "[green]CONNECTED[/]" if on_ares else "[bold red]NOT ARES[/]"

        panel.update(
            "\n".join(
                [
                    "[b]WiFi Link[/b]",
                    f"SSID: [cyan]{ssid}[/]",
                    f"Expected prefix: [yellow]{self.config.ssid_prefix}[/]",
                    f"State: {state}",
                    f"API Host: [cyan]{self.config.host}:{self.config.port}[/]",
                ]
            )
        )

    def refresh_live_status(self) -> None:
        self.connected_ssid = current_wifi_ssid()
        self.update_wifi_panel()

        panel = self.query_one("#status", Static)
        if not self.is_on_expected_wifi():
            panel.update(
                "\n".join(
                    [
                        "[b]System Status[/b]",
                        "[bold red]API paused[/]: connect to ARES WiFi first.",
                        f"Expected SSID prefix: [yellow]{self.config.ssid_prefix}[/]",
                        f"Target API: [cyan]{self.config.host}:{self.config.port}[/]",
                    ]
                )
            )
            self._log_disconnected_once()
            return

        try:
            code_status, status_payload = self.api.get_json("/api/status")
            code_health, health_payload = self.api.get_json("/api/storage/health")
        except ConnectionError as exc:
            panel.update("\n".join(["[b]System Status[/b]", f"[bold red]API unreachable[/]: {exc}"]))
            return

        if code_status != 200 or not isinstance(status_payload, dict):
            panel.update("\n".join(["[b]System Status[/b]", f"[bold red]HTTP {code_status}[/] for /api/status"]))
            return

        mode = status_payload.get("mode", "-")
        armed = status_payload.get("armed", "-")
        uptime = status_payload.get("uptimeMs", "-")
        free_heap = status_payload.get("freeHeap", "-")
        clients = status_payload.get("wifiClients", "-")

        storage_line = "Storage: [yellow]unavailable[/]"
        recovery_line = "Recovery: [yellow]unavailable[/]"
        if code_health == 200 and isinstance(health_payload, dict):
            mounted = health_payload.get("mounted", "-")
            used = health_payload.get("used", "-")
            free = health_payload.get("free", "-")
            total = health_payload.get("total", "-")
            rec_err = health_payload.get("recovery_errors", "-")
            rec_style = "green"
            try:
                if int(rec_err) > 0:
                    rec_style = "bold red"
            except Exception:
                pass

            mount_style = "green" if mounted is True else "bold red"
            storage_line = (
                "Storage: "
                f"mounted=[{mount_style}]{mounted}[/] "
                f"used=[cyan]{used}[/] "
                f"free=[cyan]{free}[/] "
                f"total=[cyan]{total}[/]"
            )
            recovery_line = (
                "Recovery: "
                f"scanned=[cyan]{health_payload.get('recovery_scanned', '-')}[/] "
                f"restored=[cyan]{health_payload.get('recovered_from_bak', '-')}[/] "
                f"rm_tmp=[cyan]{health_payload.get('removed_tmp', '-')}[/] "
                f"rm_bak=[cyan]{health_payload.get('removed_bak', '-')}[/] "
                f"errors=[{rec_style}]{rec_err}[/]"
            )

        panel.update(
            "\n".join(
                [
                    "[b]System Status[/b]",
                    f"Mode: [cyan]{mode}[/]    Armed: [cyan]{armed}[/]    Uptime(ms): [cyan]{uptime}[/]",
                    f"Free Heap: [cyan]{free_heap}[/]    WiFi Clients: [cyan]{clients}[/]",
                    storage_line,
                    recovery_line,
                ]
            )
        )

    def _get_selected_endpoint(self) -> Optional[EndpointDef]:
        table = self.query_one("#endpoints", DataTable)
        if table.cursor_row < 0 or table.cursor_row >= len(table.rows):
            return None

        row_key = table.coordinate_to_cell_key((table.cursor_row, 0)).row_key
        if row_key is None:
            return None
        key = str(row_key.value)

        for item in ENDPOINTS:
            if item.name == key:
                return item
        return None

    def _expand_path(self, path_template: str) -> str:
        file_value = self.query_one("#file_input", Input).value.strip()
        if "{file}" not in path_template:
            return path_template
        if not file_value:
            raise ValueError("{file} placeholder requires a file value")
        return path_template.replace("{file}", quote(file_value))

    def _execute(self, method: str, path: str, body_text: str, content_type: str) -> None:
        if not self.is_on_expected_wifi():
            self.write_log("[yellow]Connect to ARES WiFi before API calls[/]")
            return

        body_bytes = body_text.encode("utf-8") if len(body_text) > 0 else b""

        try:
            code, payload = self.api.request(method, path, body=body_bytes, content_type=content_type)
        except ConnectionError as exc:
            self.write_log(f"[bold red]Connection error[/]: {exc}")
            return

        self.clear_log()
        self.write_log(f"[cyan]{method}[/] {path} -> [bold]{code}[/]")

        if len(payload) == 0:
            self.write_log("[green]No content[/]")
            self.refresh_live_status()
            return

        text = payload.decode("utf-8", errors="replace")
        pretty = text
        try:
            pretty = json.dumps(json.loads(text), indent=2, ensure_ascii=False)
        except Exception:
            if len(text) > 1200:
                pretty = text[:1200] + "\n...[truncated]"

        self.write_log(pretty)
        self.refresh_live_status()

    def action_execute_selected(self) -> None:
        endpoint = self._get_selected_endpoint()
        if endpoint is None:
            self.write_log("[yellow]No endpoint selected[/]")
            return

        try:
            path = self._expand_path(endpoint.path_template)
        except ValueError as exc:
            self.write_log(f"[yellow]{exc}[/]")
            return

        self.query_one("#method_input", Input).value = endpoint.method
        self.query_one("#path_input", Input).value = path
        self.query_one("#content_type_input", Input).value = endpoint.content_type
        self.query_one("#body_input", Input).value = endpoint.body_template

        body_text = endpoint.body_template
        body_override = self.query_one("#body_input", Input).value
        if body_override != "":
            body_text = body_override

        self._execute(endpoint.method, path, body_text, endpoint.content_type)

    def action_execute_custom(self) -> None:
        method = self.query_one("#method_input", Input).value.strip().upper()
        path = self.query_one("#path_input", Input).value.strip()
        content_type = self.query_one("#content_type_input", Input).value.strip() or "application/json"

        if not method or not path:
            self.write_log("[yellow]Method and path are required[/]")
            return

        body_text = self.query_one("#body_input", Input).value

        self._execute(method, path, body_text, content_type)

    def action_refresh(self) -> None:
        self.refresh_live_status()

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        endpoint = self._get_selected_endpoint()
        if endpoint is None:
            return

        try:
            path = self._expand_path(endpoint.path_template)
        except ValueError:
            path = endpoint.path_template

        self.query_one("#method_input", Input).value = endpoint.method
        self.query_one("#path_input", Input).value = path
        self.query_one("#content_type_input", Input).value = endpoint.content_type
        self.query_one("#body_input", Input).value = endpoint.body_template

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id == "file_input":
            endpoint = self._get_selected_endpoint()
            if endpoint is None:
                return
            try:
                path = self._expand_path(endpoint.path_template)
                self.query_one("#path_input", Input).value = path
            except ValueError:
                pass

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "btn_run_selected":
            self.action_execute_selected()
        elif event.button.id == "btn_run_custom":
            self.action_execute_custom()
        elif event.button.id == "btn_refresh":
            self.action_refresh()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ARES full API Textual console")
    parser.add_argument("--host", default=DEFAULT_HOST, help="ARES API host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="ARES API port")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="HTTP timeout seconds")
    parser.add_argument("--refresh", type=float, default=DEFAULT_REFRESH_S, help="Live status refresh seconds")
    parser.add_argument("--ssid-prefix", default=DEFAULT_SSID_PREFIX, help="Expected WiFi SSID prefix")
    parser.add_argument(
        "--ignore-ssid-check",
        action="store_true",
        help="Do not require SSID prefix match before API calls",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    cfg = AppConfig(
        host=args.host,
        port=args.port,
        timeout_s=args.timeout,
        refresh_s=args.refresh,
        ssid_prefix=args.ssid_prefix,
        enforce_ssid_check=not args.ignore_ssid_check,
    )
    AresApiApp(cfg).run()


if __name__ == "__main__":
    main()
