#!/usr/bin/env python3
"""Textual receiver for ARES LoRa UART downlink.

Reads raw bytes from a serial port, resynchronises on the ARES wire
protocol sync marker, validates CRC-32, and renders decoded frames in
an interactive Textual TUI.
"""

from __future__ import annotations

import argparse
import queue
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from statistics import pstdev
from typing import Optional

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - import guard
    raise SystemExit(
        "pyserial is not installed. Run: pip install pyserial"
    ) from exc

try:
    from textual.app import App, ComposeResult
    from textual.containers import Container, Horizontal
    from textual.widgets import DataTable, Footer, Header, RichLog, Static
except ImportError as exc:  # pragma: no cover - import guard
    raise SystemExit(
        "textual is not installed. Run: pip install textual"
    ) from exc


# Configurable defaults
DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 9600
DEFAULT_TIMEOUT_S = 0.2
DEFAULT_READ_SIZE = 256
DEFAULT_UI_REFRESH_S = 0.2
DEFAULT_SHOW_RAW = False
DEFAULT_LOG_LINES = 1000
DEFAULT_LINK_TIMEOUT_S = 8.0
DEFAULT_HISTORY_ROWS = 120
DEFAULT_QUALITY_WINDOW = 30
DEFAULT_FPS_WARN_MIN = 0.12
DEFAULT_FPS_CRIT_MIN = 0.08
DEFAULT_PERIOD_WARN_MS = 7000.0
DEFAULT_PERIOD_CRIT_MS = 10000.0
DEFAULT_JITTER_WARN_MS = 1200.0
DEFAULT_JITTER_CRIT_MS = 2500.0
DEFAULT_DISCARD_WARN_PCT = 2.0
DEFAULT_DISCARD_CRIT_PCT = 5.0

# ARES protocol
SYNC = b"\xAE\x55\xC3\x1A"
PROTOCOL_VERSION = 0x02
HEADER_LEN = 10
CRC_LEN = 4
MIN_FRAME_LEN = HEADER_LEN + CRC_LEN
MAX_PAYLOAD_LEN = 200
FLAGS_RESERVED = 0xF0
FLAG_ACK_REQ = 0x01
FLAG_RETRANSMIT = 0x02
FLAG_PRIORITY = 0x04
FLAG_FRAGMENT = 0x08

MSG_TYPE_NAMES = {
    0x01: "TELEMETRY",
    0x02: "EVENT",
    0x03: "COMMAND",
    0x04: "ACK",
    0x05: "HEARTBEAT",
}

NODE_NAMES = {
    0x00: "BROADCAST",
    0x01: "ROCKET",
    0x02: "GROUND",
    0x03: "PAYLOAD",
    0xFF: "UNASSIGNED",
}

PRIORITY_NAMES = {
    0: "CRITICAL",
    1: "HIGH",
    2: "NORMAL",
    3: "LOW",
}

COMMAND_NAMES = {
    0x01: "ARM_FLIGHT",
    0x02: "ABORT",
    0x03: "FIRE_PYRO_A",
    0x04: "FIRE_PYRO_B",
    0x05: "SET_MODE",
    0x06: "SET_FCS_ACTIVE",
    0x10: "REQUEST_TELEMETRY",
    0x11: "SET_TELEM_INTERVAL",
    0x20: "REQUEST_STATUS",
    0x21: "REQUEST_CONFIG",
    0x22: "SET_CONFIG_PARAM",
    0x23: "VERIFY_CONFIG",
    0x24: "FACTORY_RESET",
}

EVENT_SEVERITY_NAMES = {
    0: "INFO",
    1: "WARN",
    2: "ERR",
}

EVENT_NAMES = {
    0x01: "MODE_CHANGE",
    0x02: "PHASE_CHANGE",
    0x03: "FCS_RULE_FIRED",
    0x04: "PYRO_FIRED",
    0x05: "ABORT_TRIGGERED",
    0x06: "SENSOR_FAILURE",
    0x07: "CRC_FAILURE",
    0x08: "FPL_VIOLATION",
    0x09: "LINK_LOST",
    0x0A: "LINK_RESTORED",
}

FAILURE_NAMES = {
    0x00: "NONE",
    0x01: "CRC_INVALID",
    0x02: "UNKNOWN_TYPE",
    0x03: "UNKNOWN_COMMAND",
    0x04: "PRECONDITION_FAIL",
    0x05: "EXECUTION_ERROR",
    0x06: "QUEUE_FULL",
    0x07: "INVALID_PARAM",
}

FLIGHT_PHASE_NAMES = {
    0: "PAD_IDLE",
    1: "ARMED",
    2: "POWERED_ASCENT",
    3: "COAST",
    4: "APOGEE",
    5: "DROGUE_DESCENT",
    6: "MAIN_DESCENT",
    7: "LANDED",
    8: "ERROR",
}


@dataclass(frozen=True)
class AppConfig:
    port: str
    baud: int
    timeout_s: float
    read_size: int
    show_raw: bool
    link_timeout_s: float
    history_rows: int
    quality_window: int
    fps_warn_min: float
    fps_crit_min: float
    period_warn_ms: float
    period_crit_ms: float
    jitter_warn_ms: float
    jitter_crit_ms: float
    discard_warn_pct: float
    discard_crit_pct: float


@dataclass(frozen=True)
class Frame:
    ver: int
    flags: int
    node: int
    msg_type: int
    seq: int
    length: int
    payload: bytes
    crc: int
    raw: bytes
    received_at: str


@dataclass(frozen=True)
class TelemetryView:
    timestamp_ms: int
    phase_name: str
    status_text: str
    altitude_agl_m: float
    vertical_vel_ms: float
    accel_mag: float
    pressure_pa: float
    temperature_c: float
    latitude: float
    longitude: float
    gps_alt_m: float
    gps_sats: int
    battery_pct: int


@dataclass(frozen=True)
class ReceiverStats:
    frames: int = 0
    discarded_bytes: int = 0
    buffer_len: int = 0
    bytes_read: int = 0
    last_seq: str = "-"
    last_type: str = "-"
    last_crc: str = "-"


@dataclass(frozen=True)
class QueueMessage:
    kind: str
    payload: object


def crc32_ares(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if (crc & 1) != 0:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def format_flags(flags: int) -> str:
    names = []
    if (flags & FLAG_ACK_REQ) != 0:
        names.append("ACK_REQ")
    if (flags & FLAG_RETRANSMIT) != 0:
        names.append("RETRANSMIT")
    if (flags & FLAG_PRIORITY) != 0:
        names.append("PRIORITY")
    if (flags & FLAG_FRAGMENT) != 0:
        names.append("FRAGMENT")
    if not names:
        names.append("NONE")
    return "|".join(names)


def color_metric_high_good(value: float, warn: float, crit: float, fmt: str) -> str:
    if value < crit:
        return f"[bold red]{fmt.format(value)}[/]"
    if value < warn:
        return f"[yellow]{fmt.format(value)}[/]"
    return f"[green]{fmt.format(value)}[/]"


def color_metric_low_good(value: float, warn: float, crit: float, fmt: str) -> str:
    if value > crit:
        return f"[bold red]{fmt.format(value)}[/]"
    if value > warn:
        return f"[yellow]{fmt.format(value)}[/]"
    return f"[green]{fmt.format(value)}[/]"


def parse_status_bits(bits: int) -> str:
    parts = [
        f"armed={(bits >> 0) & 0x01}",
        f"fcsActive={(bits >> 1) & 0x01}",
        f"gpsValid={(bits >> 2) & 0x01}",
        f"pyroAFired={(bits >> 3) & 0x01}",
        f"pyroBFired={(bits >> 4) & 0x01}",
        f"reserved={(bits >> 5) & 0x07}",
    ]
    return ", ".join(parts)


def parse_frame(frame_bytes: bytes) -> Optional[Frame]:
    if len(frame_bytes) < MIN_FRAME_LEN:
        return None
    if frame_bytes[:4] != SYNC:
        return None

    ver = frame_bytes[4]
    flags = frame_bytes[5]
    node = frame_bytes[6]
    msg_type = frame_bytes[7]
    seq = frame_bytes[8]
    payload_len = frame_bytes[9]

    if ver != PROTOCOL_VERSION:
        return None
    if (flags & FLAGS_RESERVED) != 0:
        return None
    if payload_len > MAX_PAYLOAD_LEN:
        return None
    if msg_type not in MSG_TYPE_NAMES:
        return None

    total_len = HEADER_LEN + payload_len + CRC_LEN
    if len(frame_bytes) != total_len:
        return None

    payload = frame_bytes[HEADER_LEN:HEADER_LEN + payload_len]
    crc_offset = HEADER_LEN + payload_len
    crc_recv = int.from_bytes(frame_bytes[crc_offset:crc_offset + CRC_LEN], "little")
    crc_calc = crc32_ares(frame_bytes[4:HEADER_LEN + payload_len])
    if crc_calc != crc_recv:
        return None

    return Frame(
        ver=ver,
        flags=flags,
        node=node,
        msg_type=msg_type,
        seq=seq,
        length=payload_len,
        payload=payload,
        crc=crc_recv,
        raw=frame_bytes,
        received_at=time.strftime("%Y-%m-%d %H:%M:%S"),
    )


def decode_telemetry(payload: bytes) -> Optional[TelemetryView]:
    if len(payload) != 38:
        return None

    unpacked = struct.unpack("<IBBfffffiiHBB", payload)
    return TelemetryView(
        timestamp_ms=unpacked[0],
        phase_name=FLIGHT_PHASE_NAMES.get(unpacked[1], f"UNKNOWN({unpacked[1]})"),
        status_text=parse_status_bits(unpacked[2]),
        altitude_agl_m=unpacked[3],
        vertical_vel_ms=unpacked[4],
        accel_mag=unpacked[5],
        pressure_pa=unpacked[6],
        temperature_c=unpacked[7],
        latitude=unpacked[8] / 1e7,
        longitude=unpacked[9] / 1e7,
        gps_alt_m=unpacked[10] / 10.0,
        gps_sats=unpacked[11],
        battery_pct=unpacked[12],
    )


def describe_payload(frame: Frame) -> str:
    if (frame.flags & FLAG_FRAGMENT) != 0 and frame.length >= 6:
        transfer_id, segment_num, total_segments = struct.unpack("<HHH", frame.payload[:6])
        frag_prefix = (
            f"fragment transferId={transfer_id} segment={segment_num}/{total_segments - 1} "
            f"fragPayload={hex_bytes(frame.payload[6:])}"
        )
    else:
        frag_prefix = ""

    if frame.msg_type == 0x01:
        telemetry = decode_telemetry(frame.payload)
        if telemetry is None:
            detail = f"invalid_payload_len={len(frame.payload)} raw={hex_bytes(frame.payload)}"
        else:
            detail = (
                f"timestampMs={telemetry.timestamp_ms} phase={telemetry.phase_name} "
                f"status=[{telemetry.status_text}] altitudeAglM={telemetry.altitude_agl_m:.2f} "
                f"verticalVelMs={telemetry.vertical_vel_ms:.2f} accelMag={telemetry.accel_mag:.2f} "
                f"pressurePa={telemetry.pressure_pa:.2f} temperatureC={telemetry.temperature_c:.2f} "
                f"lat={telemetry.latitude:.7f} lon={telemetry.longitude:.7f} "
                f"gpsAltM={telemetry.gps_alt_m:.1f} gpsSats={telemetry.gps_sats} "
                f"batteryPct={telemetry.battery_pct}"
            )
    elif frame.msg_type == 0x02:
        if len(frame.payload) < 6:
            detail = f"payload_too_short={len(frame.payload)} raw={hex_bytes(frame.payload)}"
        else:
            timestamp_ms, severity, event_id = struct.unpack("<IBB", frame.payload[:6])
            # AMS engine appends the raw UTF-8 event text after the 6-byte EventHeader.
            text_str = frame.payload[6:].decode("utf-8", errors="replace").rstrip("\x00")
            detail = (
                f"timestampMs={timestamp_ms} severity={EVENT_SEVERITY_NAMES.get(severity, severity)} "
                f"event={EVENT_NAMES.get(event_id, hex(event_id))} text=\"{text_str}\""
            )
    elif frame.msg_type == 0x03:
        if len(frame.payload) < 2:
            detail = f"payload_too_short={len(frame.payload)} raw={hex_bytes(frame.payload)}"
        else:
            priority, command_id = struct.unpack("<BB", frame.payload[:2])
            detail = (
                f"priority={PRIORITY_NAMES.get(priority, priority)} "
                f"command={COMMAND_NAMES.get(command_id, hex(command_id))} "
                f"params={hex_bytes(frame.payload[2:])}"
            )
    elif frame.msg_type == 0x04:
        if len(frame.payload) != 4:
            detail = f"invalid_payload_len={len(frame.payload)} raw={hex_bytes(frame.payload)}"
        else:
            original_seq, original_node, failure_code, failure_data = struct.unpack(
                "<BBBB", frame.payload
            )
            detail = (
                f"originalSeq={original_seq} originalNode={NODE_NAMES.get(original_node, hex(original_node))} "
                f"failure={FAILURE_NAMES.get(failure_code, hex(failure_code))} failureData=0x{failure_data:02X}"
            )
    elif frame.msg_type == 0x05:
        detail = "empty_payload" if len(frame.payload) == 0 else f"payload={hex_bytes(frame.payload)}"
    else:
        detail = f"payload={hex_bytes(frame.payload)}"

    if frag_prefix:
        return f"{frag_prefix} {detail}".strip()
    return detail


class StreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.discarded_bytes = 0

    def feed(self, chunk: bytes) -> list[Frame]:
        frames: list[Frame] = []
        self.buffer.extend(chunk)

        while True:
            sync_index = self.buffer.find(SYNC)
            if sync_index < 0:
                if len(self.buffer) > len(SYNC) - 1:
                    self.discarded_bytes += len(self.buffer) - (len(SYNC) - 1)
                    del self.buffer[: len(self.buffer) - (len(SYNC) - 1)]
                break

            if sync_index > 0:
                self.discarded_bytes += sync_index
                del self.buffer[:sync_index]

            if len(self.buffer) < MIN_FRAME_LEN:
                break

            payload_len = self.buffer[9]
            if payload_len > MAX_PAYLOAD_LEN:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue

            total_len = HEADER_LEN + payload_len + CRC_LEN
            if len(self.buffer) < total_len:
                break

            candidate = bytes(self.buffer[:total_len])
            frame = parse_frame(candidate)
            if frame is None:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue

            frames.append(frame)
            del self.buffer[:total_len]

        return frames


class SerialReceiver(threading.Thread):
    def __init__(self, config: AppConfig, out_queue: queue.Queue[QueueMessage]) -> None:
        super().__init__(daemon=True)
        self.config = config
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.parser = StreamParser()
        self.frame_count = 0
        self.bytes_read = 0

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        try:
            with serial.Serial(
                port=self.config.port,
                baudrate=self.config.baud,
                timeout=self.config.timeout_s,
            ) as ser:
                self.out_queue.put(
                    QueueMessage(
                        kind="status",
                        payload=f"Connected to {self.config.port} @ {self.config.baud}",
                    )
                )

                while not self.stop_event.is_set():
                    chunk = ser.read(self.config.read_size)
                    if not chunk:
                        self._push_stats()
                        continue

                    self.bytes_read += len(chunk)
                    frames = self.parser.feed(chunk)
                    for frame in frames:
                        self.frame_count += 1
                        self.out_queue.put(QueueMessage(kind="frame", payload=frame))
                    self._push_stats()
        except SerialException as exc:
            self.out_queue.put(QueueMessage(kind="error", payload=str(exc)))

    def _push_stats(self) -> None:
        stats = ReceiverStats(
            frames=self.frame_count,
            discarded_bytes=self.parser.discarded_bytes,
            buffer_len=len(self.parser.buffer),
            bytes_read=self.bytes_read,
        )
        self.out_queue.put(QueueMessage(kind="stats", payload=stats))


class RxTestApp(App[None]):
    CSS = """
    Screen {
        layout: vertical;
    }

    #root {
        height: 1fr;
        padding: 1;
    }

    #top {
        layout: horizontal;
        height: 14;
        margin-bottom: 1;
    }

    #summary {
        width: 1fr;
        border: round #3a6ea5;
        padding: 0 1;
        margin-right: 1;
    }

    #telemetry {
        width: 1fr;
        border: round #2f855a;
        padding: 0 1;
    }

    #bottom {
        height: 1fr;
    }

    #history {
        height: 12;
        border: round #3f8f5f;
        margin-bottom: 1;
    }

    #log {
        height: 1fr;
        border: round #805ad5;
    }
    """

    BINDINGS = [("q", "quit", "Quit")]

    def __init__(self, config: AppConfig) -> None:
        super().__init__()
        self.config = config
        self.message_queue: queue.Queue[QueueMessage] = queue.Queue()
        self.receiver = SerialReceiver(config, self.message_queue)
        self.status_text = "Starting..."
        self.stats = ReceiverStats()
        self.last_frame: Optional[Frame] = None
        self.last_telemetry: Optional[TelemetryView] = None
        self.last_frame_monotonic: Optional[float] = None
        self.history_entries: deque[tuple[str, str, str, str, str, str, str, str]] = deque()
        self.history_row_id = 0
        self.frame_times: deque[float] = deque()
        self.frame_intervals_ms: deque[float] = deque()

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Container(id="root"):
            with Horizontal(id="top"):
                yield Static(id="summary")
                yield Static(id="telemetry")
            with Container(id="bottom"):
                yield DataTable(id="history", zebra_stripes=True)
                yield RichLog(id="log", max_lines=DEFAULT_LOG_LINES, wrap=True, highlight=True)
        yield Footer()

    def on_mount(self) -> None:
        self.title = "ARES RX Test"
        self.sub_title = f"{self.config.port} @ {self.config.baud} baud"
        self.receiver.start()
        self.set_interval(DEFAULT_UI_REFRESH_S, self.process_queue)
        self.setup_history_table()
        self.refresh_summary()
        self.refresh_telemetry()
        self.write_log("Listening on serial port...")
        self.update_layout_for_size()

    def on_resize(self) -> None:
        """Handle terminal resize and adjust layout responsively."""
        self.update_layout_for_size()

    def update_layout_for_size(self) -> None:
        """Update layout based on available terminal width."""
        try:
            top = self.query_one("#top", Horizontal)
            summary = self.query_one("#summary", Static)
            telemetry = self.query_one("#telemetry", Static)
            
            # Get available width (approximate terminal width)
            available_width = self.size.width if self.size else 100
            
            # Threshold: if less than 140 chars, stack vertically
            if available_width < 140:
                # Stack vertically
                top.styles.layout = "vertical"
                summary.styles.margin_right = (0, 0, 0, 0)
                summary.styles.margin_bottom = (1, 0, 0, 0)
                telemetry.styles.width = "100%"
            else:
                # Side by side
                top.styles.layout = "horizontal"
                summary.styles.margin_right = (0, 1, 0, 0)
                summary.styles.margin_bottom = (0, 0, 0, 0)
                telemetry.styles.width = "1fr"
        except Exception:
            pass  # Gracefully ignore if widgets not yet ready

    def setup_history_table(self) -> None:
        table = self.query_one("#history", DataTable)
        table.add_columns("Time", "Seq", "Type", "Alt/Event", "Press(Pa)", "Temp(C)", "GPS", "CRC")

    def on_unmount(self) -> None:
        self.receiver.stop()

    def process_queue(self) -> None:
        changed = False
        while True:
            try:
                message = self.message_queue.get_nowait()
            except queue.Empty:
                break

            if message.kind == "status":
                self.status_text = str(message.payload)
                changed = True
            elif message.kind == "error":
                self.status_text = f"ERROR: {message.payload}"
                self.write_log(f"[bold red]ERROR[/]: {message.payload}")
                changed = True
            elif message.kind == "stats":
                stats = message.payload
                if isinstance(stats, ReceiverStats):
                    self.stats = ReceiverStats(
                        frames=stats.frames,
                        discarded_bytes=stats.discarded_bytes,
                        buffer_len=stats.buffer_len,
                        bytes_read=stats.bytes_read,
                        last_seq=self.stats.last_seq,
                        last_type=self.stats.last_type,
                        last_crc=self.stats.last_crc,
                    )
                    changed = True
            elif message.kind == "frame":
                frame = message.payload
                if isinstance(frame, Frame):
                    now_mono = time.monotonic()
                    if self.last_frame_monotonic is not None:
                        interval_ms = (now_mono - self.last_frame_monotonic) * 1000.0
                        self.frame_intervals_ms.append(interval_ms)
                        while len(self.frame_intervals_ms) > self.config.quality_window:
                            self.frame_intervals_ms.popleft()

                    self.frame_times.append(now_mono)
                    while len(self.frame_times) > self.config.quality_window:
                        self.frame_times.popleft()

                    self.last_frame = frame
                    self.last_frame_monotonic = now_mono
                    self.stats = ReceiverStats(
                        frames=self.stats.frames + 1,
                        discarded_bytes=self.stats.discarded_bytes,
                        buffer_len=self.stats.buffer_len,
                        bytes_read=self.stats.bytes_read,
                        last_seq=str(frame.seq),
                        last_type=MSG_TYPE_NAMES.get(frame.msg_type, str(frame.msg_type)),
                        last_crc=f"0x{frame.crc:08X}",
                    )
                    telemetry = decode_telemetry(frame.payload) if frame.msg_type == 0x01 else None
                    if telemetry is not None:
                        self.last_telemetry = telemetry
                    self.add_history_row(frame, telemetry)
                    self.write_log(self.format_frame_log(frame))
                    changed = True

        if changed:
            self.refresh_summary()
            self.refresh_telemetry()

    def refresh_summary(self) -> None:
        link_status = "NO DATA"
        if self.last_frame_monotonic is not None:
            elapsed = time.monotonic() - self.last_frame_monotonic
            if elapsed <= self.config.link_timeout_s:
                link_status = f"OK ({elapsed:.1f}s)"
            else:
                link_status = f"LOST ({elapsed:.1f}s)"

        if link_status.startswith("OK"):
            link_markup = f"[green]{link_status}[/]"
        elif link_status.startswith("LOST"):
            link_markup = f"[bold red]{link_status}[/]"
        else:
            link_markup = f"[yellow]{link_status}[/]"

        fps_value: Optional[float] = None
        period_value: Optional[float] = None
        jitter_value: Optional[float] = None

        if len(self.frame_times) >= 2:
            span_s = self.frame_times[-1] - self.frame_times[0]
            if span_s > 0.0:
                fps = (len(self.frame_times) - 1) / span_s
                fps_value = fps

        if len(self.frame_intervals_ms) >= 1:
            mean_ms = sum(self.frame_intervals_ms) / len(self.frame_intervals_ms)
            period_value = mean_ms

        if len(self.frame_intervals_ms) >= 2:
            jitter_ms = pstdev(self.frame_intervals_ms)
            jitter_value = jitter_ms

        discard_pct = 0.0
        if self.stats.bytes_read > 0:
            discard_pct = (self.stats.discarded_bytes * 100.0) / self.stats.bytes_read

        fps_markup = "-"
        if fps_value is not None:
            fps_markup = color_metric_high_good(
                fps_value,
                self.config.fps_warn_min,
                self.config.fps_crit_min,
                "{:.2f}",
            )

        period_markup = "-"
        if period_value is not None:
            period_markup = color_metric_low_good(
                period_value,
                self.config.period_warn_ms,
                self.config.period_crit_ms,
                "{:.1f}",
            )

        jitter_markup = "-"
        if jitter_value is not None:
            jitter_markup = color_metric_low_good(
                jitter_value,
                self.config.jitter_warn_ms,
                self.config.jitter_crit_ms,
                "{:.1f}",
            )

        discard_markup = color_metric_low_good(
            discard_pct,
            self.config.discard_warn_pct,
            self.config.discard_crit_pct,
            "{:.2f}%",
        )

        summary = self.query_one("#summary", Static)
        summary.update(
            "\n".join(
                [
                    "[b]Status[/b]",
                    f"Port: [cyan]{self.config.port}[/]    Baud: [cyan]{self.config.baud}[/]    Timeout: [cyan]{self.config.timeout_s:.2f}s[/]",
                    f"Connection: [green]{self.status_text}[/]    Link: {link_markup}",
                    f"Frames: [bold]{self.stats.frames}[/]    Bytes read: [bold]{self.stats.bytes_read}[/]    Buffer: [bold]{self.stats.buffer_len}[/]    Dropped: [bold]{self.stats.discarded_bytes}[/]",
                    f"Quality: fps={fps_markup}    period_ms={period_markup}    jitter_ms={jitter_markup}    drop={discard_markup}",
                    f"Last type: [yellow]{self.stats.last_type}[/]    Last seq: [yellow]{self.stats.last_seq}[/]    Last CRC: [yellow]{self.stats.last_crc}[/]",
                ]
            )
        )

    def add_history_row(self, frame: Frame, telemetry: Optional[TelemetryView]) -> None:
        table = self.query_one("#history", DataTable)

        if telemetry is not None:
            alt_text = f"{telemetry.altitude_agl_m:.2f}"
            pres_text = f"{telemetry.pressure_pa:.1f}"
            temp_text = f"{telemetry.temperature_c:.2f}"
            gps_text = f"{telemetry.latitude:.5f},{telemetry.longitude:.5f}"
        elif frame.msg_type == 0x02 and len(frame.payload) >= 6:
            # EVENT frame: show severity + AMS text string in the Alt/Event column.
            _, severity, _ = struct.unpack("<IBB", frame.payload[:6])
            sev_name = EVENT_SEVERITY_NAMES.get(severity, str(severity))
            text_str = frame.payload[6:].decode("utf-8", errors="replace").rstrip("\x00")
            alt_text = f"{sev_name}: {text_str[:28]}"
            pres_text = "-"
            temp_text = "-"
            gps_text = "-"
        else:
            alt_text = "-"
            pres_text = "-"
            temp_text = "-"
            gps_text = "-"

        row = (
            frame.received_at.split(" ")[1],
            str(frame.seq),
            MSG_TYPE_NAMES.get(frame.msg_type, str(frame.msg_type)),
            alt_text,
            pres_text,
            temp_text,
            gps_text,
            f"{frame.crc:08X}",
        )
        self.history_entries.appendleft(row)

        while len(self.history_entries) > self.config.history_rows:
            self.history_entries.pop()

        # Rebuild table so newest frame is always shown first.
        table.clear(columns=False)
        for entry in self.history_entries:
            self.history_row_id += 1
            table.add_row(*entry, key=f"row-{self.history_row_id}")

    def refresh_telemetry(self) -> None:
        telemetry_panel = self.query_one("#telemetry", Static)
        if self.last_telemetry is None or self.last_frame is None:
            telemetry_panel.update(
                "\n".join(
                    [
                        "[b]Latest Telemetry[/b]",
                        "Waiting for a valid TELEMETRY frame...",
                    ]
                )
            )
            return

        telemetry = self.last_telemetry
        telemetry_panel.update(
            "\n".join(
                [
                    "[b]Latest Telemetry[/b]",
                    f"Received: [cyan]{self.last_frame.received_at}[/]    Seq: [cyan]{self.last_frame.seq}[/]",
                    # AMS-generated TELEMETRY frames never set flightPhase (always 0 = PAD_IDLE).
                    f"Phase: [green]{telemetry.phase_name}[/] [dim](AMS frames: always PAD_IDLE)[/]    Flags: [green]{format_flags(self.last_frame.flags)}[/]",
                    f"Status: {telemetry.status_text}",
                    f"Alt AGL: [bold]{telemetry.altitude_agl_m:.2f} m[/]    VVel: [bold]{telemetry.vertical_vel_ms:.2f} m/s[/]    Accel: [bold]{telemetry.accel_mag:.2f} m/s2[/]",
                    f"Pressure: [bold]{telemetry.pressure_pa:.2f} Pa[/]    Temp: [bold]{telemetry.temperature_c:.2f} C[/]",
                    f"Lat: [bold]{telemetry.latitude:.7f}[/]    Lon: [bold]{telemetry.longitude:.7f}[/]",
                    f"GPS Alt: [bold]{telemetry.gps_alt_m:.1f} m[/]    Sats: [bold]{telemetry.gps_sats}[/]    Battery: [bold]{telemetry.battery_pct}%[/]",
                ]
            )
        )

    def write_log(self, text: str) -> None:
        log = self.query_one("#log", RichLog)
        log.write(text)

    def format_frame_log(self, frame: Frame) -> str:
        header = (
            f"[{frame.received_at}] [b]{MSG_TYPE_NAMES[frame.msg_type]}[/] seq={frame.seq} "
            f"len={frame.length} node={NODE_NAMES.get(frame.node, hex(frame.node))} "
            f"flags={format_flags(frame.flags)} crc=0x{frame.crc:08X}"
        )
        detail = describe_payload(frame)
        raw = f" raw={hex_bytes(frame.raw)}" if self.config.show_raw else ""
        return f"{header}\n  {detail}{raw}"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ARES LoRa receiver TUI")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port to open")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="Serial read timeout in seconds",
    )
    parser.add_argument(
        "--read-size",
        type=int,
        default=DEFAULT_READ_SIZE,
        help="Maximum bytes per serial read",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        default=DEFAULT_SHOW_RAW,
        help="Also show raw frame bytes in hexadecimal",
    )
    parser.add_argument(
        "--link-timeout",
        type=float,
        default=DEFAULT_LINK_TIMEOUT_S,
        help="Seconds without frames before link is marked as lost",
    )
    parser.add_argument(
        "--history-rows",
        type=int,
        default=DEFAULT_HISTORY_ROWS,
        help="Maximum number of rows in history table",
    )
    parser.add_argument(
        "--quality-window",
        type=int,
        default=DEFAULT_QUALITY_WINDOW,
        help="Sample window size for quality metrics",
    )
    parser.add_argument("--fps-warn-min", type=float, default=DEFAULT_FPS_WARN_MIN)
    parser.add_argument("--fps-crit-min", type=float, default=DEFAULT_FPS_CRIT_MIN)
    parser.add_argument("--period-warn-ms", type=float, default=DEFAULT_PERIOD_WARN_MS)
    parser.add_argument("--period-crit-ms", type=float, default=DEFAULT_PERIOD_CRIT_MS)
    parser.add_argument("--jitter-warn-ms", type=float, default=DEFAULT_JITTER_WARN_MS)
    parser.add_argument("--jitter-crit-ms", type=float, default=DEFAULT_JITTER_CRIT_MS)
    parser.add_argument("--discard-warn-pct", type=float, default=DEFAULT_DISCARD_WARN_PCT)
    parser.add_argument("--discard-crit-pct", type=float, default=DEFAULT_DISCARD_CRIT_PCT)
    return parser


def build_config(args: argparse.Namespace) -> AppConfig:
    return AppConfig(
        port=args.port,
        baud=args.baud,
        timeout_s=args.timeout,
        read_size=args.read_size,
        show_raw=args.raw,
        link_timeout_s=args.link_timeout,
        history_rows=args.history_rows,
        quality_window=max(2, args.quality_window),
        fps_warn_min=args.fps_warn_min,
        fps_crit_min=args.fps_crit_min,
        period_warn_ms=args.period_warn_ms,
        period_crit_ms=args.period_crit_ms,
        jitter_warn_ms=args.jitter_warn_ms,
        jitter_crit_ms=args.jitter_crit_ms,
        discard_warn_pct=args.discard_warn_pct,
        discard_crit_pct=args.discard_crit_pct,
    )


def main() -> int:
    args = build_arg_parser().parse_args()
    config = build_config(args)
    app = RxTestApp(config)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
