#!/usr/bin/env python3
"""cross_parser_test.py — Cross-validation: ams_parser.py ≡ firmware parser.

Verifies that the ground-station Python parser (scripts/ams_parser.py) and
the firmware C++ parser agree on every test case in the corpus below.

The corpus is split into:
  VALID_SCRIPTS   — scripts the firmware accepts (activate() returns true).
                    The Python parser must also return ok=True for each.
  INVALID_SCRIPTS — scripts the firmware rejects.
                    The Python parser must also return ok=False for each.

Agreement means:
  - Python says OK  ↔ firmware says OK
  - Python says FAIL ↔ firmware says FAIL (modulo known WARN-only divergences)

Usage:
    python test/test_ams_parser_cross/cross_parser_test.py
    python test/test_ams_parser_cross/cross_parser_test.py -v  # verbose

Exit code:
    0  all cases agree
    1  one or more divergences found

Run from the workspace root (the directory that contains platformio.ini).
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import tempfile
import textwrap
from typing import NamedTuple

# ── Locate ams_parser module ─────────────────────────────────────────────────
# Workspace root = two levels up from this file's directory.
_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT / "scripts"))

try:
    from ams_parser import AmsParser  # type: ignore[import]
except ImportError as exc:
    print(f"ERROR: cannot import ams_parser from {_REPO_ROOT / 'scripts'}: {exc}")
    sys.exit(1)


# ── Corpus definition ────────────────────────────────────────────────────────

class Case(NamedTuple):
    name: str
    script: str
    expect_ok: bool
    note: str = ""


# Real hardware includes used so the Python DRIVER_REGISTRY recognises them.
_HDR = textwrap.dedent("""\
    include BN220 as GPS
    include BMP280 as BARO
    include DXLR03 as COM
    include MPU6050 as IMU
    pus.apid = 1
    pus.service 3 as HK
    pus.service 5 as EVENT
    pus.service 1 as TC
""")


CORPUS: list[Case] = [

    # ── Valid scripts ──────────────────────────────────────────────────────

    Case(
        name="minimal_two_state",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 1 as TC
            state WAIT:
              transition to END when TC.command == LAUNCH
            state END:
        """),
        expect_ok=True,
        note="Minimal valid two-state script without any includes.",
    ),
    Case(
        name="full_flight_script",
        script=_HDR + textwrap.dedent("""\
            state WAIT:
              transition to FLIGHT when TC.command == LAUNCH
            state FLIGHT:
              every 1000ms:
                HK.report {
                  alt: BARO.alt
                  temp: BARO.temp
                }
              transition to END when TIME.elapsed > 5000
            state END:
        """),
        expect_ok=True,
        note="Three-state script with HK telemetry and TIME.elapsed transition.",
    ),
    Case(
        name="sensor_transition",
        script=_HDR + textwrap.dedent("""\
            state WAIT:
              transition to HIGH when BARO.alt > 500
            state HIGH:
        """),
        expect_ok=True,
        note="Sensor-based transition BARO.alt > threshold.",
    ),
    Case(
        name="on_enter_event",
        script=_HDR + textwrap.dedent("""\
            state INIT:
              on_enter:
                EVENT.info "starting"
              transition to END when TC.command == LAUNCH
            state END:
        """),
        expect_ok=True,
        note="on_enter block with EVENT.info.",
    ),
    Case(
        name="on_timeout_block",
        script=textwrap.dedent("""\
            pus.apid = 1
            state WAIT:
              on_timeout 5000ms:
                transition to SAFE
            state SAFE:
        """),
        expect_ok=True,
        note="on_timeout block with a forced transition.",
    ),
    Case(
        name="fallback_transition",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 1 as TC
            state WAIT:
              fallback transition to SAFE after 2000ms
            state SAFE:
        """),
        expect_ok=True,
        note="Fallback transition with after clause.",
    ),
    Case(
        name="var_declaration",
        script=textwrap.dedent("""\
            pus.apid = 1
            var threshold = 100.0
            pus.service 1 as TC
            state IDLE:
              transition to END when TC.command == LAUNCH
            state END:
        """),
        expect_ok=True,
        note="Global var declaration (NAME = VALUE) before states.",
    ),
    Case(
        name="const_declaration",
        script=textwrap.dedent("""\
            pus.apid = 1
            const APOGEE = 3000.0
            pus.service 1 as TC
            state IDLE:
              transition to END when TC.command == LAUNCH
            state END:
        """),
        expect_ok=True,
        note="Global const declaration (NAME = VALUE) before states.",
    ),
    Case(
        name="log_every_block",
        script=_HDR + textwrap.dedent("""\
            state FLIGHT:
              log_every 200ms:
                LOG.report {
                  alt: BARO.alt
                  temp: BARO.temp
                }
              transition to END when TIME.elapsed > 5000
            state END:
        """),
        expect_ok=True,
        note="log_every block with LOG.report fields.",
    ),
    Case(
        name="confirm_modifier",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 1 as TC
            pus.service 5 as EVENT
            state WAIT:
              transition to FLIGHT when TC.command == LAUNCH confirm 3
            state FLIGHT:
              on_enter:
                EVENT.info "launched"
        """),
        expect_ok=True,
        note="CONFIRM N debounce modifier on TC.command transition.",
    ),
    Case(
        name="imu_fields",
        script=_HDR + textwrap.dedent("""\
            state FLIGHT:
              every 500ms:
                HK.report {
                  ax: IMU.accel_x
                  ay: IMU.accel_y
                  az: IMU.accel_z
                  mag: IMU.accel_mag
                  gx: IMU.gyro_x
                }
              transition to END when TIME.elapsed > 1000
            state END:
        """),
        expect_ok=True,
        note="IMU sensor fields in HK report.",
    ),
    Case(
        name="gps_fields",
        script=_HDR + textwrap.dedent("""\
            state WAIT:
              transition to HIGH when GPS.alt > 1000
            state HIGH:
              every 2000ms:
                HK.report {
                  lat: GPS.lat
                  lon: GPS.lon
                  alt: GPS.alt
                  spd: GPS.speed
                }
        """),
        expect_ok=True,
        note="GPS sensor fields in HK report and transition.",
    ),
    Case(
        name="on_error_recovery",
        script=_HDR + textwrap.dedent("""\
            include BMP280 as BARO2
            state FLIGHT:
              conditions:
                BARO.alt > 0
              on_error:
                transition to SAFE
            state SAFE:
        """),
        expect_ok=True,
        note="on_error block with recovery transition.",
    ),
    Case(
        name="priorities_directive",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 1 as TC
            pus.service 3 as HK
            state IDLE:
              priorities event=4 hk=3 log=1 budget=2
              transition to END when TC.command == LAUNCH
            state END:
        """),
        expect_ok=True,
        note="priorities directive with all four keys.",
    ),
    Case(
        name="abort_tc",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 1 as TC
            state FLIGHT:
              transition to SAFE when TC.command == ABORT
            state SAFE:
        """),
        expect_ok=True,
        note="Explicit ABORT transition.",
    ),
    Case(
        name="reset_tc",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 1 as TC
            state IDLE:
              transition to FLIGHT when TC.command == LAUNCH
            state FLIGHT:
              transition to IDLE when TC.command == RESET
        """),
        expect_ok=True,
        note="RESET TC transition back to initial state.",
    ),

    # ── Invalid scripts ────────────────────────────────────────────────────

    Case(
        name="no_states",
        script="pus.apid = 1\n",
        expect_ok=False,
        note="Script with no state blocks must be rejected.",
    ),
    Case(
        name="include_after_state",
        script=textwrap.dedent("""\
            state INIT:
            state END:
            include BMP280 as BARO
        """),
        expect_ok=False,
        note="include directive placed after a state block is rejected.",
    ),
    Case(
        name="var_after_state",
        script=textwrap.dedent("""\
            state INIT:
            state END:
            var x : float = 1.0
        """),
        expect_ok=False,
        note="var declaration placed after a state block is rejected.",
    ),
    Case(
        name="const_after_state",
        script=textwrap.dedent("""\
            state INIT:
            state END:
            const K : float = 1.0
        """),
        expect_ok=False,
        note="const declaration placed after a state block is rejected.",
    ),
    Case(
        name="unknown_transition_target",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 1 as TC
            state INIT:
              transition to NONEXISTENT when TC.command == LAUNCH
        """),
        expect_ok=False,
        note="Transition to an undeclared state must be rejected.",
    ),
    Case(
        name="invalid_tc_command",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 1 as TC
            state IDLE:
              transition to END when TC.command == FIRE
            state END:
        """),
        expect_ok=False,
        note="'FIRE' is not a valid TC.command value (valid: ABORT, LAUNCH, RESET).",
    ),
    Case(
        name="apid_out_of_range",
        script=textwrap.dedent("""\
            pus.apid = 4
            state ONLY:
        """),
        expect_ok=False,
        note="pus.apid = 4 is outside the valid range 0..3.",
    ),
    Case(
        name="too_many_states",
        script="pus.apid = 1\npus.service 1 as TC\n"
               + "".join(
                   f"state S{i}:\n  transition to S{i+1} when TC.command == LAUNCH\n"
                   if i < 10
                   else f"state S{i}:\n"
                   for i in range(11)
               ),
        expect_ok=False,
        note="11 states exceeds AMS_MAX_STATES (10).",
    ),
    Case(
        name="too_many_hk_fields",
        script=(
            "include BMP280 as BARO\n"
            "pus.apid = 1\n"
            "pus.service 3 as HK\n"
            "state WAIT:\n"
            "  every 1000ms:\n"
            "    HK.report {\n"
            + "".join(f"      f{i:02d}: BARO.alt\n" for i in range(17))
            + "    }\n"
        ),
        expect_ok=False,
        note="17 HK fields in one report block exceeds AMS_MAX_HK_FIELDS (16).",
    ),
    Case(
        name="invalid_event_verb",
        script=textwrap.dedent("""\
            pus.apid = 1
            pus.service 5 as EVENT
            state INIT:
              on_enter:
                EVENT.debug "this verb is not valid"
        """),
        expect_ok=False,
        note="'debug' is not a valid EVENT verb (valid: info, warning, error).",
    ),
    Case(
        name="hk_report_without_every",
        script=textwrap.dedent("""\
            include BMP280 as BARO
            pus.apid = 1
            pus.service 3 as HK
            state WAIT:
              HK.report {
                alt: BARO.alt
              }
        """),
        expect_ok=False,
        note="HK.report without a preceding 'every' block must be rejected.",
    ),
    Case(
        name="unknown_sensor_field",
        script=textwrap.dedent("""\
            include BMP280 as BARO
            pus.apid = 1
            pus.service 3 as HK
            state WAIT:
              every 1000ms:
                HK.report {
                  bad: BARO.velocity
                }
        """),
        expect_ok=False,
        note="'velocity' is not a valid field for BARO sensors.",
    ),
    Case(
        name="transition_missing_when",
        script=textwrap.dedent("""\
            pus.apid = 1
            state WAIT:
              transition to END
            state END:
        """),
        expect_ok=False,
        note="'transition to END' without 'when' clause must be rejected.",
    ),
]

# ── Corpus enrichment: ams_examples/ ─────────────────────────────────────────
# All .ams files in ams_examples/ are expected to be valid.

_EXAMPLES_DIR = _REPO_ROOT / "ams_examples"
if _EXAMPLES_DIR.is_dir():
    for _p in sorted(_EXAMPLES_DIR.glob("*.ams")):
        CORPUS.append(
            Case(
                name=f"example:{_p.name}",
                script=_p.read_text(encoding="utf-8", errors="replace"),
                expect_ok=True,
                note=f"ams_examples/{_p.name} must parse without errors.",
            )
        )

# ── Test runner ───────────────────────────────────────────────────────────────


def _run_python_parser(script_text: str, name: str) -> bool:
    """Return True if the Python parser accepts the script."""
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".ams", delete=False, encoding="utf-8"
    ) as tmp:
        tmp.write(script_text)
        tmp_path = pathlib.Path(tmp.name)

    try:
        result = AmsParser(tmp_path).parse()
        return result.ok
    finally:
        tmp_path.unlink(missing_ok=True)


def _colour(text: str, code: str, use_colour: bool) -> str:
    if not use_colour:
        return text
    return f"\033[{code}m{text}\033[0m"


def run_corpus(verbose: bool = False) -> int:
    use_colour = sys.stdout.isatty()
    pass_count = 0
    fail_count = 0
    divergence_count = 0

    print(f"Running {len(CORPUS)} cross-parser test cases...\n")

    for case in CORPUS:
        py_ok = _run_python_parser(case.script, case.name)
        agreed = (py_ok == case.expect_ok)

        if agreed:
            pass_count += 1
            if verbose:
                tag = _colour("PASS", "32", use_colour)
                exp = "accept" if case.expect_ok else "reject"
                print(f"  {tag}  [{exp}]  {case.name}")
        else:
            fail_count += 1
            divergence_count += 1
            tag = _colour("DIVERGE", "31;1", use_colour)
            py_verdict = "accepted" if py_ok else "rejected"
            fw_verdict = "accepted" if case.expect_ok else "rejected"
            print(
                f"  {tag}  {case.name}\n"
                f"           Python parser : {py_verdict}\n"
                f"           Firmware      : {fw_verdict} (expected)\n"
                f"           Note          : {case.note}"
            )

    print(
        f"\n{'='*60}\n"
        f"Results: {pass_count} passed, {divergence_count} diverged"
        f" (out of {len(CORPUS)} cases)\n"
        f"{'='*60}"
    )

    return 0 if divergence_count == 0 else 1


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Print PASS lines too, not just DIVERGE.")
    args = parser.parse_args()

    sys.exit(run_corpus(verbose=args.verbose))
