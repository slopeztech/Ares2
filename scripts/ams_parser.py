#!/usr/bin/env python3
"""ams_parser.py — ARES Mission Script offline validator.

Replicates the exact parse rules of MissionScriptEngine (mission_script_engine.cpp)
so that syntax errors are caught before uploading to the device.

Usage:
    python scripts/ams_parser.py <file.ams> [<file2.ams> ...]
    python scripts/ams_parser.py ams_examples/radiotest.ams

Exit code:
    0  all files valid
    1  one or more files have errors
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ── Limits (mirror config.h) ────────────────────────────────────────────────
AMS_MAX_SCRIPT_BYTES    = 4096
AMS_MAX_STATES          = 10
AMS_MAX_STATE_NAME      = 15   # sscanf("%15[^:]")
AMS_MAX_EVENT_TEXT      = 63   # sscanf("%63[^\"]")
AMS_MAX_HK_FIELDS       = 16
AMS_MAX_LINE_LEN        = 128
MISSION_FILENAME_MAX    = 32
TELEMETRY_INTERVAL_MIN  = 100  # ms

TC_COMMANDS = {"LAUNCH", "ABORT", "RESET"}

# ── Driver registry: MODEL -> peripheral kind ────────────────────────────────
# Maps each known model name to its peripheral kind string.
# Unknown models produce a warning (custom drivers may be valid).
DRIVER_REGISTRY: dict[str, str] = {
    "BN220":    "GPS",
    "BN880":    "GPS",
    "BMP280":   "BARO",
    "BMP390":   "BARO",
    "LORA":     "COM",
    "DXLR03":   "COM",
    "MPU6050":  "IMU",
    "ICM42688": "IMU",
}

# ── Valid sensor fields per peripheral kind ──────────────────────────────────
SENSOR_FIELDS: dict[str, set[str]] = {
    "GPS":  {"lat", "lon", "alt", "speed", "sats", "hdop"},
    "BARO": {"alt", "temp", "pressure"},
    "COM":  set(),
    "IMU":  {"accel_x", "accel_y", "accel_z", "accel_mag",
             "gyro_x", "gyro_y", "gyro_z", "gyro_mag", "temp"},
}

# ── APID → node mapping (mapApidToNode) ──────────────────────────────────────
VALID_APIDS = {0, 1, 2, 3}

# ── Diagnostic dataclasses ───────────────────────────────────────────────────

@dataclass
class Diagnostic:
    line_no: int
    message: str
    is_error: bool = True

    def __str__(self) -> str:
        tag = "ERROR" if self.is_error else "WARN "
        return f"  line {self.line_no:3d}  [{tag}]  {self.message}"


@dataclass
class StateSummary:
    name: str
    has_hk_every: bool = False
    has_log_every: bool = False
    hk_field_count: int = 0
    log_field_count: int = 0
    has_transition: bool = False
    transition_target: Optional[str] = None
    has_on_enter_event: bool = False


@dataclass
class ParseResult:
    file: Path
    diagnostics: list[Diagnostic] = field(default_factory=list)
    states: list[StateSummary] = field(default_factory=list)
    apid: Optional[int] = None

    @property
    def errors(self) -> list[Diagnostic]:
        return [d for d in self.diagnostics if d.is_error]

    @property
    def warnings(self) -> list[Diagnostic]:
        return [d for d in self.diagnostics if not d.is_error]

    @property
    def ok(self) -> bool:
        return len(self.errors) == 0


# ── Parser ────────────────────────────────────────────────────────────────────

class AmsParser:
    """Stateful parser — one instance per file."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self._result = ParseResult(file=path)
        self._states: list[StateSummary] = []
        self._current: Optional[StateSummary] = None
        self._block: Optional[str] = None   # "HK" | "LOG" | None
        self._apid: Optional[int] = None
        self._aliases: dict[str, str] = {}  # alias -> peripheral kind
        self._declared_vars: set[str] = {}  # AMS variable names declared with 'var'
        self._hk_alias: str = "HK"
        self._event_alias: str = "EVENT"
        self._tc_alias: str = "TC"

    # ── Public entry point ───────────────────────────────────────────────────

    def parse(self) -> ParseResult:
        try:
            raw = self._path.read_bytes()
        except OSError as exc:
            self._result.diagnostics.append(
                Diagnostic(0, f"cannot read file: {exc}")
            )
            return self._result

        if len(raw) > AMS_MAX_SCRIPT_BYTES:
            self._warn(
                0,
                f"file size {len(raw)} B exceeds firmware limit "
                f"{AMS_MAX_SCRIPT_BYTES} B — upload will be rejected",
            )

        try:
            # Accept both UTF-8 and UTF-8 with BOM so line-1 directives
            # like 'include ...' are parsed consistently.
            text = raw.decode("utf-8-sig")
        except UnicodeDecodeError as exc:
            self._result.diagnostics.append(
                Diagnostic(0, f"UTF-8 decode error: {exc}")
            )
            return self._result

        for lineno, raw_line in enumerate(text.splitlines(), start=1):
            if len(raw_line) > AMS_MAX_LINE_LEN:
                self._error(
                    lineno,
                    f"line exceeds {AMS_MAX_LINE_LEN} chars (firmware truncates it)",
                )
            line = raw_line.strip()
            self._parse_line(line, lineno)
            if not self._result.ok and len(self._result.errors) >= 20:
                self._error(lineno, "too many errors — stopping")
                break

        self._post_parse()

        self._result.states = self._states
        self._result.apid = self._apid
        return self._result

    # ── Line dispatcher (mirrors parseLineLocked) ─────────────────────────────

    def _parse_line(self, line: str, ln: int) -> None:
        # Empty / comment
        if not line or line.startswith("//"):
            return

        # Block close
        if line == "}":
            self._block = None
            return

        # include <MODEL> as <KIND>
        if line.startswith("include "):
            self._parse_include(line, ln)
            return

        # Ignored directives
        if line.startswith("pus.service "):
            self._parse_service(line, ln)
            return

        # pus.apid
        if line.startswith("pus.apid"):
            self._parse_apid(line, ln)
            return

        # Top-level var/const declarations
        if line.startswith("var ") or line.startswith("const "):
            if self._current is not None:
                kw = "var" if line.startswith("var ") else "const"
                self._error(ln, f"{kw} declaration must appear before any state block")
            elif line.startswith("var "):
                m = re.match(r"var\s+(\w{1,15})\s*=", line)
                if m:
                    self._declared_vars.add(m.group(1))
            return

        # State header
        if line.startswith("state "):
            self._block = None
            self._parse_state(line, ln)
            return

        # Must be inside a state from here on
        if self._current is None:
            self._error(ln, "statement outside state block")
            return

        self._parse_state_scoped(line, ln)

    # ── Top-level parsers ─────────────────────────────────────────────────────

    def _parse_include(self, line: str, ln: int) -> None:
        if self._current is not None:
            self._error(ln, "include directive must appear before any state block")
            return
        m = re.match(r"include\s+(\S+)\s+as\s+(\S+)", line)
        if not m:
            self._error(ln, "invalid include syntax (expected: include MODEL as ALIAS)")
            return
        model = m.group(1)
        alias = m.group(2)

        if alias in self._aliases:
            self._error(ln, f"duplicate alias '{alias}' — each alias must be unique")
            return

        if len(self._aliases) >= 8:  # AMS_MAX_INCLUDES = 8
            self._error(ln, "too many includes (max 8 per script)")
            return

        kind = DRIVER_REGISTRY.get(model)
        if kind is None:
            known = sorted(DRIVER_REGISTRY.keys())
            self._warn(
                ln,
                f"unrecognised model '{model}' — make sure the driver is compiled "
                f"into the firmware (known: {known})",
            )
            # Still register with a placeholder so downstream validation
            # doesn't cascade errors on every field/transition referencing it.
            kind = "UNKNOWN"

        self._aliases[alias] = kind

    def _parse_service(self, line: str, ln: int) -> None:
        m = re.match(r"pus\.service\s+(\d+)\s+as\s+(\S+)", line)
        if not m:
            self._error(ln, "invalid pus.service syntax (expected: pus.service N as ALIAS)")
            return
        svc = int(m.group(1))
        alias = m.group(2)
        if svc not in (1, 3, 5):
            self._error(ln, f"pus.service: unsupported service number {svc} (valid: 1, 3, 5)")
            return
        if alias in ("TIME", "LOG"):
            self._error(ln, f"pus.service: alias '{alias}' conflicts with reserved keyword")
            return
        if svc == 1:
            self._tc_alias = alias
        elif svc == 3:
            self._hk_alias = alias
        elif svc == 5:
            self._event_alias = alias

    def _parse_apid(self, line: str, ln: int) -> None:
        m = re.search(r"=\s*(\S+)", line)
        if not m:
            self._error(ln, "invalid pus.apid syntax: missing '='")
            return
        try:
            apid = int(m.group(1))
        except ValueError:
            self._error(ln, f"invalid pus.apid value: '{m.group(1)}' is not an integer")
            return
        if apid < 0 or apid > 2047:
            self._error(ln, f"invalid pus.apid value: {apid} out of range [0, 2047]")
            return
        if apid not in VALID_APIDS:
            self._error(
                ln,
                f"unsupported APID {apid} for ARES node mapping "
                f"(valid: {sorted(VALID_APIDS)})",
            )
            return
        if self._apid is not None:
            self._warn(ln, "pus.apid redefined")
        self._apid = apid

    def _parse_state(self, line: str, ln: int) -> None:
        if len(self._states) >= AMS_MAX_STATES:
            self._error(ln, f"too many states (max {AMS_MAX_STATES})")
            return
        m = re.match(r"state\s+([^:]{1,15}):", line)
        if not m:
            # Check for name too long
            m2 = re.match(r"state\s+(\S+)\s*:", line)
            if m2 and len(m2.group(1)) > AMS_MAX_STATE_NAME:
                self._error(
                    ln,
                    f"state name '{m2.group(1)}' too long "
                    f"(max {AMS_MAX_STATE_NAME} chars)",
                )
            else:
                self._error(ln, "invalid state syntax (expected 'state NAME:')")
            return
        name = m.group(1).strip()
        if any(s.name == name for s in self._states):
            self._warn(ln, f"duplicate state name '{name}'")
        st = StateSummary(name=name)
        self._states.append(st)
        self._current = st

    # ── State-scoped parsers (mirrors parseStateScopedLineLocked) ─────────────

    def _parse_state_scoped(self, line: str, ln: int) -> None:
        assert self._current is not None

        # Block headers that reset block context
        if line in ("on_enter:", "on_exit:"):
            self._block = None
            return

        if line == "on_error:":
            self._block = "ON_ERROR"
            return

        if line == "conditions:":
            self._block = "CONDITIONS"
            return

        if re.match(r"on_timeout\s+\d+\s*ms:", line):
            self._block = "ON_TIMEOUT"
            return

        # on_timeout / on_error / on_exit content
        if self._block in ("ON_TIMEOUT", "ON_ERROR", "ON_EXIT"):
            if line.startswith(self._event_alias + "."):
                self._parse_event(line, ln)
            elif line.startswith("set "):
                pass  # validated by firmware
            elif line.startswith("transition to "):
                # Inside on_timeout/on_error, transition has no 'when' clause
                m = re.match(r"transition\s+to\s+(\S{1,15})\s*$", line)
                if not m:
                    self._error(ln, "invalid forced-transition syntax (expected: 'transition to STATE')")
                    return
                assert self._current is not None
                self._current.has_transition = True
                self._current.transition_target = m.group(1)
            # ignore other lines inside these blocks
            return

        if line.startswith(self._event_alias + "."):
            self._block = None
            self._parse_event(line, ln)
            return

        if line.startswith("set "):
            self._block = None
            return

        if line.startswith("every "):
            self._block = None
            self._parse_every(line, ln)
            return

        if line.startswith("log_every "):
            self._block = None
            self._parse_log_every(line, ln)
            return

        if line.startswith("priorities "):
            self._block = None
            self._parse_priorities(line, ln)
            return

        if line.startswith(self._hk_alias + ".report"):
            self._block = None
            if not self._current.has_hk_every:
                self._error(ln, "HK.report requires an 'every NNNms:' block above it")
                return
            self._block = "HK"
            return

        if line.startswith("LOG.report"):
            self._block = None
            if not self._current.has_log_every:
                self._error(ln, "LOG.report requires a 'log_every NNNms:' block above it")
                return
            self._block = "LOG"
            return

        if self._block == "HK":
            self._parse_field(line, ln, "HK")
            return

        if self._block == "LOG":
            self._parse_field(line, ln, "LOG")
            return

        if self._block == "CONDITIONS":
            self._parse_condition(line, ln)
            return

        if line.startswith("transition to "):
            self._parse_transition(line, ln)
            return

        if line.startswith("fallback transition to "):
            self._current.has_transition = True
            return

        self._error(ln, f"unsupported statement: '{line}'")

    def _parse_condition(self, line: str, ln: int) -> None:
        """Parse a single conditions: block line (LHS OP RHS, no 'when' keyword)."""
        if line in ("{", "}"):
            return
        parts = line.split(None, 2)
        if len(parts) < 2:
            self._error(ln, "invalid condition syntax (expected: ALIAS.field OP VALUE)")
            return
        lhs = parts[0]
        if lhs.startswith("TIME."):
            return  # TIME.elapsed — always valid, no alias needed
        dot = lhs.find(".")
        if dot <= 0:
            self._error(ln, f"invalid condition LHS '{lhs}' (expected ALIAS.field or TIME.elapsed)")
            return
        alias = lhs[:dot]
        field = lhs[dot + 1:]
        if alias not in self._aliases:
            self._error(ln, f"unknown alias '{alias}' in condition — add 'include MODEL as {alias}'")
            return
        kind = self._aliases[alias]
        if kind == "UNKNOWN":
            return
        valid_fields = SENSOR_FIELDS.get(kind, set())
        if field not in valid_fields:
            self._error(
                ln,
                f"field '{field}' not valid for alias '{alias}' (kind={kind}, "
                f"valid: {sorted(valid_fields)})",
            )

    def _parse_event(self, line: str, ln: int) -> None:
        # <ALIAS>.<verb> "<text>"
        pfx = re.escape(self._event_alias)
        m = re.match(rf'{pfx}\.(\w+)\s+"([^"]*)"', line)
        if not m:
            # Check common mistakes
            if '"' not in line:
                self._error(ln, "invalid EVENT syntax: text must be in double quotes")
            elif len(re.findall(r'"', line)) < 2:
                self._error(ln, "invalid EVENT syntax: unclosed string literal")
            else:
                self._error(ln, "invalid EVENT syntax (expected: EVENT.verb \"text\")")
            return
        verb = m.group(1)
        text = m.group(2)
        if verb not in ("info", "warning", "error"):
            self._error(ln, f"unknown EVENT verb '{verb}' (valid: info, warning, error)")
            return
        if len(text) > AMS_MAX_EVENT_TEXT:
            self._warn(
                ln,
                f"EVENT text length {len(text)} exceeds firmware limit "
                f"{AMS_MAX_EVENT_TEXT} chars (will be truncated)",
            )
        assert self._current is not None
        self._current.has_on_enter_event = True

    def _parse_every(self, line: str, ln: int) -> None:
        # every NNNms:
        m = re.match(r"every\s+(\d+)\s*ms:\s*$", line)
        if not m:
            self._error(
                ln,
                "invalid 'every' syntax (expected: 'every NNNms:')",
            )
            return
        ms = int(m.group(1))
        if ms < TELEMETRY_INTERVAL_MIN:
            self._error(
                ln,
                f"HK interval {ms} ms is below minimum {TELEMETRY_INTERVAL_MIN} ms (APUS-19.3)",
            )
            return
        assert self._current is not None
        self._current.has_hk_every = True
        self._current.hk_field_count = 0

    def _parse_log_every(self, line: str, ln: int) -> None:
        # log_every NNNms:
        m = re.match(r"log_every\s+(\d+)\s*ms:\s*$", line)
        if not m:
            self._error(
                ln,
                "invalid 'log_every' syntax (expected: 'log_every NNNms:')",
            )
            return
        ms = int(m.group(1))
        if ms == 0:
            self._error(ln, "log_every period must be > 0 ms")
            return
        assert self._current is not None
        self._current.has_log_every = True
        self._current.log_field_count = 0

    def _parse_priorities(self, line: str, ln: int) -> None:
        # priorities [event=N] [hk=N] [log=N] [budget=N]
        # Any subset is valid; values 0-9 for event/hk/log, 1-3 for budget.
        keys_found: dict[str, int] = {}
        for kv in re.findall(r"(\w+)=(\d+)", line):
            keys_found[kv[0]] = int(kv[1])

        if not keys_found:
            self._error(ln, "invalid priorities syntax: no key=value pairs found")
            return

        unknown = set(keys_found) - {"event", "hk", "log", "budget"}
        if unknown:
            self._warn(ln, f"unknown priority keys: {sorted(unknown)}")

        for k in ("event", "hk", "log"):
            if k in keys_found and not (0 <= keys_found[k] <= 9):
                self._error(ln, f"priorities: {k}={keys_found[k]} out of range [0, 9]")

        if "budget" in keys_found and not (1 <= keys_found["budget"] <= 3):
            self._error(
                ln,
                f"priorities: budget={keys_found['budget']} out of range [1, 3]",
            )

    def _parse_field(self, line: str, ln: int, ctx: str) -> None:
        # { is legal (opening brace)
        if line == "{":
            return

        # key: ALIAS.field
        m = re.match(r"^([^:]{1,19}):\s+(\S+)$", line)
        if not m:
            self._error(ln, f"invalid {ctx} field syntax (expected 'label: ALIAS.field' or 'label: varname')")
            return

        expr = m.group(2)
        self._validate_sensor_expr(expr, ln, ctx)

        assert self._current is not None
        if ctx == "HK":
            self._current.hk_field_count += 1
            if self._current.hk_field_count > AMS_MAX_HK_FIELDS:
                self._error(ln, f"too many HK fields (max {AMS_MAX_HK_FIELDS})")
        else:
            self._current.log_field_count += 1
            if self._current.log_field_count > AMS_MAX_HK_FIELDS:
                self._error(ln, f"too many LOG fields (max {AMS_MAX_HK_FIELDS})")

    def _parse_transition(self, line: str, ln: int) -> None:
        # transition to TARGET when LHS OP RHS
        m = re.match(
            r"transition\s+to\s+(\S{1,15})\s+when\s+(\S+)\s+(\S+)\s+(\S+)",
            line,
        )
        if not m:
            self._error(
                ln,
                "invalid transition syntax "
                "(expected: 'transition to STATE when LHS OP RHS')",
            )
            return

        target, lhs, op, rhs = m.group(1), m.group(2), m.group(3), m.group(4)

        assert self._current is not None
        if self._current.has_transition:
            self._warn(ln, f"state '{self._current.name}' already has a transition — only the last one is used")

        self._current.has_transition = True
        self._current.transition_target = target

        # TC.command == LAUNCH|ABORT|RESET
        tc_cmd = f"{self._tc_alias}.command"
        if lhs == tc_cmd:
            if op != "==":
                self._error(ln, f"TC.command transition requires '==' operator, got '{op}'")
                return
            if rhs not in TC_COMMANDS:
                self._error(
                    ln,
                    f"invalid TC.command value '{rhs}' (valid: {sorted(TC_COMMANDS)})",
                )
            return

        # TIME.elapsed > VALUE  (no include needed)
        if lhs == "TIME.elapsed":
            if op != ">":
                self._error(ln, f"TIME.elapsed only supports '>' operator, got '{op}'")
                return
            try:
                val = float(rhs)
            except ValueError:
                self._error(ln, f"invalid TIME.elapsed threshold '{rhs}': not a number")
                return
            if not math.isfinite(val):
                self._error(ln, f"invalid TIME.elapsed threshold '{rhs}': must be finite")
            return

        # ALIAS.field < VALUE  or  ALIAS.field > VALUE
        dot = lhs.find(".")
        if dot <= 0:
            self._error(
                ln,
                f"invalid condition LHS '{lhs}' (expected ALIAS.field, "
                "TC.command, or TIME.elapsed)",
            )
            return

        alias = lhs[:dot]
        field = lhs[dot + 1:]

        if alias not in self._aliases:
            self._error(ln, f"unknown alias '{alias}' in transition condition — add 'include MODEL as {alias}'")
            return

        kind = self._aliases[alias]
        valid_fields = SENSOR_FIELDS.get(kind, set())
        if field not in valid_fields:
            self._error(
                ln,
                f"field '{field}' not valid for alias '{alias}' (kind={kind}, "
                f"valid: {sorted(valid_fields)})",
            )
            return

        if op not in ("<", ">"):
            self._error(ln, f"sensor condition only supports '<' or '>' operators, got '{op}'")
            return

        # Numeric threshold
        try:
            val = float(rhs)
        except ValueError:
            self._error(ln, f"invalid sensor threshold '{rhs}': not a number")
            return
        if not math.isfinite(val):
            self._error(ln, f"invalid sensor threshold '{rhs}': must be finite")

    # ── Post-parse cross-checks ───────────────────────────────────────────────

    def _post_parse(self) -> None:
        if not self._states:
            self._error(0, "no states defined")
            return

        state_names = {s.name for s in self._states}

        for st in self._states:
            # Transition target resolution
            if st.has_transition and st.transition_target is not None:
                if st.transition_target not in state_names:
                    ln = 0  # we don't track original line here
                    self._error(
                        ln,
                        f"state '{st.name}': transition target '{st.transition_target}' "
                        "does not exist",
                    )

            # HK.report without any fields
            if st.has_hk_every and st.hk_field_count == 0:
                self._warn(
                    0,
                    f"state '{st.name}': 'every' block defined but HK.report has no fields",
                )

        # pus.apid not set
        if self._apid is None:
            self._warn(0, "pus.apid not set — firmware will use default APID 1 (NODE_ROCKET)")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _validate_sensor_expr(self, expr: str, ln: int, ctx: str) -> None:
        """Validate an ALIAS.field expression or AMS variable name against declared identifiers."""
        dot = expr.find(".")
        if dot <= 0:
            # No dot — try as a variable name (AMS-4.8).
            if expr in self._declared_vars:
                return
            self._error(
                ln,
                f"invalid {ctx} expression '{expr}' "
                f"(expected ALIAS.field or declared variable name)",
            )
            return

        alias = expr[:dot]
        field = expr[dot + 1:]

        if alias not in self._aliases:
            self._error(
                ln,
                f"unknown alias '{alias}' in {ctx} field \u2014 add 'include MODEL as {alias}'",
            )
            return

        kind = self._aliases[alias]
        if kind == "UNKNOWN":
            return  # warning already issued at include time

        valid_fields = SENSOR_FIELDS.get(kind, set())
        if field not in valid_fields:
            self._error(
                ln,
                f"field '{field}' not valid for alias '{alias}' (kind={kind}, "
                f"valid: {sorted(valid_fields)})",
            )

    def _error(self, ln: int, msg: str) -> None:
        self._result.diagnostics.append(Diagnostic(ln, msg, is_error=True))

    def _warn(self, ln: int, msg: str) -> None:
        self._result.diagnostics.append(Diagnostic(ln, msg, is_error=False))


# ── Report ────────────────────────────────────────────────────────────────────

def _colour(text: str, code: str) -> str:
    """Wrap text in ANSI colour if stdout is a tty."""
    if not sys.stdout.isatty():
        return text
    return f"\033[{code}m{text}\033[0m"


def print_result(result: ParseResult) -> None:
    path_str = str(result.file)

    if result.ok:
        tag = _colour("OK", "32;1")
    else:
        tag = _colour("FAIL", "31;1")

    print(f"{tag}  {path_str}")

    if result.states:
        names = ", ".join(s.name for s in result.states)
        print(f"       states ({len(result.states)}): {names}")

    if result.apid is not None:
        node_map = {0: "BROADCAST", 1: "ROCKET", 2: "GROUND", 3: "PAYLOAD"}
        print(f"       apid: {result.apid} ({node_map.get(result.apid, '?')})")

    for d in sorted(result.diagnostics, key=lambda x: x.line_no):
        print(str(d))

    if result.ok and not result.diagnostics:
        print("       no issues found")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Validate ARES Mission Script (.ams) files offline.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("files", nargs="+", metavar="FILE", help=".ams file(s) to validate")
    ap.add_argument(
        "--strict",
        action="store_true",
        help="Treat warnings as errors (exit 1 if any warning)",
    )
    args = ap.parse_args()

    any_fail = False
    for f in args.files:
        path = Path(f)
        result = AmsParser(path).parse()
        print_result(result)
        print()
        if not result.ok:
            any_fail = True
        elif args.strict and result.warnings:
            any_fail = True

    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main())
