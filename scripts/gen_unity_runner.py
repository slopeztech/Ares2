#!/usr/bin/env python3
"""
gen_unity_runner.py — Unity test runner generator for PlatformIO suites.

Scans a test-suite directory for ``void test_*()`` function definitions in
all ``.cpp`` files (excluding ``main.cpp`` itself), then writes a fresh
``main.cpp`` runner with:

  - One ``extern void test_*();`` forward-declaration block per source file.
  - A single ``main()`` that calls ``UNITY_BEGIN()``, one ``RUN_TEST()`` per
    test in declaration order, and ``UNITY_END()``.

Usage::

    python scripts/gen_unity_runner.py test/test_ams_integration
    python scripts/gen_unity_runner.py test/test_ams_parser
    python scripts/gen_unity_runner.py test/test_radio_protocol

The generated file carries a ``GENERATED FILE`` banner so that maintainers
know to re-run the script instead of editing it by hand.
"""

import re
import sys
from pathlib import Path

# ── Constants ────────────────────────────────────────────────────────────────

LINE_WIDTH = 80

# Regex: top-level (non-indented) ``void test_*()`` definition.
# Matches both ``void test_foo()`` and ``void test_foo(void)``.
_TEST_DEF_RE = re.compile(r'^void\s+(test_\w+)\s*\(\s*(?:void)?\s*\)', re.MULTILINE)


# ── Helpers ──────────────────────────────────────────────────────────────────

def _section(label: str) -> str:
    """Return a ``// ── label ──...`` separator line of LINE_WIDTH chars."""
    prefix = f"// \u2500\u2500 {label} "
    fill = max(1, LINE_WIDTH - len(prefix))
    return prefix + "\u2500" * fill


def _extract_tests(cpp_file: Path) -> list[str]:
    """Return function names for all ``void test_*()`` definitions in *cpp_file*."""
    text = cpp_file.read_text(encoding="utf-8", errors="replace")
    return _TEST_DEF_RE.findall(text)


# ── Generator ────────────────────────────────────────────────────────────────

def generate(suite_dir: Path) -> str:
    """Generate and return the full ``main.cpp`` content for *suite_dir*."""

    suite_name = suite_dir.name

    # Collect source files (alphabetical, deterministic).
    cpp_files = sorted(
        f for f in suite_dir.glob("*.cpp") if f.name != "main.cpp"
    )

    # Map filename → list of test function names (preserving definition order).
    file_tests: list[tuple[str, list[str]]] = []
    for f in cpp_files:
        names = _extract_tests(f)
        if names:
            file_tests.append((f.name, names))

    total = sum(len(t) for _, t in file_tests)

    out: list[str] = []

    # File header ─────────────────────────────────────────────────────────────
    out += [
        "/**",
        " * @file  main.cpp",
        f" * @brief Unity runner for {suite_name} tests.",
        " *",
        " * GENERATED FILE — do not edit manually.",
        f" * Run: python scripts/gen_unity_runner.py test/{suite_name}",
        " *",
        f" * Test count: {total}",
        " */",
        "#include <unity.h>",
        "",
        "void setUp()    {}",
        "void tearDown() {}",
        "",
    ]

    # Forward declarations ────────────────────────────────────────────────────
    for filename, names in file_tests:
        out.append(_section(filename))
        out.append("")
        for name in names:
            out.append(f"extern void {name}();")
        out.append("")

    # Runner ──────────────────────────────────────────────────────────────────
    out.append(_section("Runner"))
    out.append("")
    out.append("int main()")
    out.append("{")
    out.append("    UNITY_BEGIN();")

    for filename, names in file_tests:
        out.append("")
        # Section comment: bare stem of filename (no "test_" prefix, no ".cpp").
        stem = filename.removesuffix(".cpp").removeprefix("test_")
        out.append(f"    // {stem}")
        for name in names:
            out.append(f"    RUN_TEST({name});")

    out.append("")
    out.append("    return UNITY_END();")
    out.append("}")
    out.append("")

    return "\n".join(out)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: gen_unity_runner.py <test-suite-dir>", file=sys.stderr)
        print("  e.g. gen_unity_runner.py test/test_ams_integration", file=sys.stderr)
        return 1

    suite_dir = Path(sys.argv[1])
    if not suite_dir.is_dir():
        print(f"error: '{suite_dir}' is not a directory", file=sys.stderr)
        return 1

    out_path = suite_dir / "main.cpp"
    content  = generate(suite_dir)
    out_path.write_text(content, encoding="utf-8")

    # Count and report.
    n = content.count("RUN_TEST(")
    print(f"wrote {out_path}  ({n} tests)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
