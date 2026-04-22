# ARES MISRA C++ Review Profile

Operational profile for MISRA C++ audits with traceability and use of certifiable tooling.
This profile complements the local standard [misra_standard.md](../standards/misra_standard.md).

## 1. Objective

Define a repeatable process to:
- Review MISRA C++ compliance in project-owned code (`src/`).
- Separate real findings from external framework noise.
- Produce auditable evidence for each release.

## 2. Scope

Includes:
- Application code in `src/`.
- Project-owned headers in `src/` and `include/`.

Excludes:
- PlatformIO/Arduino/ESP-IDF dependencies under `.pio/`.
- Third-party code not maintained by this project.

## 3. Tooling and Minimum Evidence

For formal compliance, use a tool with certifiable MISRA C++ support
(for example, PC-lint Plus or an equivalent toolset with a current MISRA C++ catalog).

Minimum evidence per run:
- Tool version and active rule set.
- Analysis configuration and flags.
- Raw report output (XML/HTML/TXT).
- Approved deviation list.
- Associated build result (`platformio run`).

## 4. Severity Profile

Recommended classification:
- Blocking: Required violations without approved deviation.
- Major: Advisory violations that are recurrent or functionally risky.
- Minor: Style/readability issues without immediate functional impact.

Recommended CI gate:
- Fail pipeline if at least 1 new Blocking finding exists.
- Allow Major findings only with ticket and remediation due date.

## 5. Priority Review Rules (ARES)

Apply this sequence in each cycle:
1. Fixed-width types and explicit conversions.
2. Deterministic flow and no undefined behavior.
3. Memory policy: no runtime heap use.
4. Pointers: validation and restricted casts.
5. Error handling and return-value checks.
6. `switch` completeness and no fallthrough.
7. Function size, complexity, and nesting limits.
8. Magic literals and constant discipline.

## 6. Deviation Template

Record each approved deviation with:
- ID: `DEV-MISRA-XXXX`.
- Impacted MISRA rule.
- File and symbol.
- Technical justification and risk analysis.
- Applied mitigation.
- Date, owner, and closure criteria.

## 7. Release Workflow

1. Run a clean build.
2. Run practical static analysis (Cppcheck without MISRA addon).
3. Run certifiable MISRA C++ tooling.
4. Consolidate and classify findings (Blocking/Major/Minor).
5. Fix findings or create approved deviations.
6. Re-run and attach final evidence to the release.

## 8. Suggested Artifacts

Store under `evidence/latest/` during development, or under `evidence/<release>/` when tagging a release:
- `build.log`
- `cppcheck-practical.xml`
- `misra-certified-report.xml` (or equivalent)
- `deviations.md`
- `compliance-summary.md`

## 9. Notes for This Repository

- The Cppcheck `misra.py` addon may produce false positives on modern C++
  (for example `nullptr`, `static_cast`, templates), so it must not be used
  as the only criterion for formal compliance.
- Keep practical Cppcheck as a daily safety net and use certifiable
  MISRA C++ tooling for compliance closure.
