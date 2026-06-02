# Versioning Policy

ARES follows a three-part version number: **MAJOR.MINOR.PATCH**

```
  2   .   1   .   0
  │       │       └─ PATCH  — bug fixes and minor adjustments
  │       └───────── MINOR  — new features or controlled breaking changes
  └───────────────── MAJOR  — full redesign or platform-level breaking change
```

---

## When to increment each field

| Field   | Increment when…                                                                                              | Examples                                                  |
|---------|--------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------|
| `MAJOR` | Full redesign, platform migration, or total incompatibility with previous hardware/API versions              | Migrate from ESP32 to another SoC; rewrite the AMS engine |
| `MINOR` | A new feature is added or a controlled breaking change is introduced                                         | New AMS command, new hardware subsystem, API breakage     |
| `PATCH` | Bug fix, comment/documentation adjustment, or cosmetic change with no functional impact                      | Parse error fix, stale comment, doc update                |

When `MAJOR` is incremented, both `MINOR` and `PATCH` are reset to **0**.  
When `MINOR` is incremented, `PATCH` is reset to **0**.

---

## Source of truth

The canonical version lives in `src/config.h`:

```cpp
#define ARES_VERSION_STRING "2.4.0"
```

---

## Changelog

Each version has its own file under `docs/changelog/`:

```
docs/changelog/
    v2.0.0.md   ← first structured release
    v2.1.0.md   ← pulse.channel / 4 channels / pin rename
    v2.1.1.md   ← AMS-4.18.6 strict token check (bug fix)
    v2.1.2.md   ← AMS_MAX_STATES 10→16; BFS bitmask uint32_t
    v2.2.0.md   ← AMS-4.8.8 arithmetic expressions in set
    v2.2.1.md   ← CI: branch coverage, uncovered lines, trend
    v2.2.2.md   ← Safety: FIRE_PULSE_C/D dispatcher + tests
    v2.2.3.md   ← Coverage: ares_log.cpp null guards + truncation tests (H2)
    v2.2.4.md   ← Test: test_api_routing — 10 HTTP routing + auth integration tests (H3)
    v2.2.5.md   ← Safety: formatHkFieldValueLocked returns false for sensor failures; "nan" sentinel in CSV (H4)
    v2.2.6.md   ← Fix: resolve all ares_code_lint findings (1 ERROR + 39 WARNINGs); CERT/DOX/MISRA/RTOS compliance
    v2.2.7.md   ← Refactor: split mission_script_engine.h into _types.h and _internal.h ([M1])
    v2.2.8.md   ← Fix: replace hardcoded TELEM_INTERVAL limits with TELEMETRY_INTERVAL_MIN/MAX ([M2])
    v2.2.9.md   ← Fix: sliding-window anti-replay bitmap for COMMAND SEQ ([H5])
    v2.3.0.md   ← Minor: radio_retry_drops health counter + TWDT sleep cap
    v2.3.1.md   ← Fix: RFC 7230 OWS compliance in parseHeaders() [M3, M14]
    v2.3.2.md   ← Security: first-boot credentials, constant-time token, WiFi-in-flight, HTTP fixes
    v2.3.3.md   ← Security: radio link HMAC-SHA256 authentication ([C1])
    v2.3.4.md   ← Security: rolling timestamp anti-replay window for COMMAND frames ([C1])
    v2.3.5.md   ← Fix: documentation clarity (BUG-13), CSV robustness (BUG-15), compile-time sentinel guard (BUG-16)
    v2.3.6.md   ← Robustness: LED BOOT state guaranteed to exit on every setup() path (BUG-18)
    v2.3.7.md   ← Security: global mission-upload buffer ownership invariant enforced (BUG-3)
    v2.3.8.md   ← Hardening: ARES_REQUIRE macro; release-invisible safety checks replaced (P0-1, P0-5, P1-1); CI fix
    v2.3.9.md   ← Fix: P0-4 pulse timer rollback on begin() failure; applyRadioMacKey helper; clang-tidy clean
    v2.3.10.md  ← Hardening: P0-2/P0-3/P1-5/P1-6 API server fixes; P3-4 HTTP parser fuzz tests; clang-tidy clean
    v2.3.11.md  ← Patch: P1-2 radio dispatcher single-task enforcement; P1-4 encode() unambiguous bool return; P3-1 dxlr03 magic-number cleanup
    v2.3.12.md  ← Patch: P1-3 SIOF main.cpp (placement new + startApiServer); P2-1/P2-4 truncation guards; P2-5 fill_n; P3-2 kCrcSuffix; P2-2 single-task contract
    v2.3.13.md  ← Patch: tick() flush — single-exit restructure guarantees flushPendingIoUnlocked() on every exit path; 4 new SITL tests
    v2.3.14.md  ← Patch: checkpoint timing markers committed only after confirmed writeFile(); 1 new SITL retry test
    v2.3.15.md  ← Patch: hardening pass — checkpoint atomicity, parser trailing-garbage, radio.config refactor, sensor NaN guard, BMP280 suite, clang-tidy PASS
    v2.4.0.md   ← Minor: A2-3 `every Nms via ALIAS:` per-slot COM routing; `HkSlot.comAlias`; runtime fallback LOG_W; 10 new tests
    v2.4.1.md   ← Patch: A2-4 null-iface driver detection at activation time; `validateAliasIfacesLocked()`; 1 new lifecycle test
```

File names follow the pattern `vMAJOR.MINOR.PATCH.md`.

---

## Version history

| Version | Date       | Type  | Summary                                                       |
|---------|------------|-------|---------------------------------------------------------------|
| [2.0.0](../changelog/v2.0.0.md) | 2026-05-11 | Release | First structured release; modular AMS; REST auth             |
| [2.1.0](../changelog/v2.1.0.md) | 2026-05-17 | Minor   | AMS-4.18 `pulse.channel`; 4 pulse channels; pin rename       |
| [2.1.1](../changelog/v2.1.1.md) | 2026-05-17 | Patch   | AMS-4.18.6 strict token check; reject `pulse.channel A FOO`  |
| [2.1.2](../changelog/v2.1.2.md) | 2026-05-17 | Patch   | `AMS_MAX_STATES` 10→16; BFS bitmask `uint32_t`; 2 new tests  |
| [2.2.0](../changelog/v2.2.0.md) | 2026-05-17 | Minor   | AMS-4.8.8 arithmetic expressions in `set` statements         |
| [2.2.1](../changelog/v2.2.1.md) | 2026-05-18 | Patch   | CI: branch coverage, uncovered lines, trend (`run_coverage.ps1`) |
| [2.2.2](../changelog/v2.2.2.md) | 2026-05-18 | Patch   | Safety: `FIRE_PULSE_C`/`FIRE_PULSE_D` dispatcher + tests |
| [2.2.3](../changelog/v2.2.3.md) | 2026-05-18 | Patch   | Coverage: `ares_log.cpp` null guards + truncation tests (H2) |
| [2.2.4](../changelog/v2.2.4.md) | 2026-05-18 | Patch   | Test: `test_api_routing` — 10 HTTP routing + auth integration tests (H3) |
| [2.2.5](../changelog/v2.2.5.md) | 2026-05-18 | Patch   | Safety: `formatHkFieldValueLocked` returns false for sensor failures; `"nan"` sentinel in CSV (H4) |
| [2.2.6](../changelog/v2.2.6.md) | 2026-05-19 | Patch   | Fix: resolve all `ares_code_lint` findings (1 ERROR + 39 WARNINGs); CERT/DOX/MISRA/RTOS compliance |
| [2.2.7](../changelog/v2.2.7.md) | 2026-05-19 | Patch   | Refactor: split `mission_script_engine.h` into `_types.h` + `_internal.h` ([M1]); −39% header size |
| [2.2.8](../changelog/v2.2.8.md) | 2026-05-19 | Patch   | Fix: replace hardcoded `[100, 60000]` limits with `TELEMETRY_INTERVAL_MIN/MAX` constants ([M2]); 3 new boundary tests |
| [2.2.9](../changelog/v2.2.9.md) | 2026-05-19 | Patch   | Fix: replace equality-only SEQ duplicate check with 64-slot sliding-window `SeqBitmap` ([H5]); 10 new unit tests |
| [2.3.0](../changelog/v2.3.0.md) | 2026-05-20 | Minor   | New: `radio_retry_drops` health counter in `GET /api/status` (APUS-4.5); Fix: TWDT sleep cap `LOOP_SLEEP_MAX_MS = 2000U`; 4 new unit tests |
| [2.3.1](../changelog/v2.3.1.md) | 2026-05-20 | Patch   | Fix: RFC 7230 §3.2.3 OWS compliance in `parseHeaders()` — unbounded leading OWS + trailing OWS on token ([M3], [M14]); 13 new unit tests |
| [2.3.2](../changelog/v2.3.2.md) | 2026-05-22 | Patch   | Security: first-boot TRNG credentials ([C2], [C3]); WiFi off in flight ([C5]); correct HTTP reason phrases ([H6]); 202 on wifi_password change ([H7]); scan endpoints locked in flight ([H8]); abort allows RUNNING engine ([H10]); defensive double-init guards ([H14]); constant-time token compare ([M4]); `toPublicJson` bufSize guard ([M7]); 414 on oversized URI ([M8]); empty password preserves stored ([M10]); `sendError` message truncation ([M11]); 17 new unit tests |
| [2.3.3](../changelog/v2.3.3.md) | 2026-05-22 | Patch   | Security: radio link HMAC-SHA256 authentication for COMMAND frames ([C1]) |
| [2.3.4](../changelog/v2.3.4.md) | 2026-05-26 | Patch   | Security: rolling timestamp anti-replay window for COMMAND frames ([C1]) |
| [2.3.5](../changelog/v2.3.5.md) | 2026-05-27 | Patch   | Fix: documentation clarity (BUG-13); CSV robustness (BUG-15); compile-time sentinel guard (BUG-16) |
| [2.3.6](../changelog/v2.3.6.md) | 2026-05-27 | Patch   | Robustness: LED BOOT state guaranteed to exit on every `setup()` path (BUG-18) |
| [2.3.7](../changelog/v2.3.7.md) | 2026-05-28 | Patch   | Security: global mission-upload buffer ownership invariant enforced at access point (BUG-3) |
| [2.3.8](../changelog/v2.3.8.md) | 2026-05-29 | Patch   | Hardening: `ARES_REQUIRE` macro; release-invisible safety checks replaced; engine destructor lifetime fix; CI parser fix |
| [2.3.9](../changelog/v2.3.9.md)   | 2026-05-30 | Patch   | Fix: P0-4 pulse timer rollback on `begin()` failure; `applyRadioMacKey` helper; clang-tidy clean |
| [2.3.10](../changelog/v2.3.10.md) | 2026-05-31 | Patch   | Hardening: P0-2/P0-3/P1-5/P1-6 API server fixes; P3-4 HTTP parser fuzz tests (10 new); clang-tidy clean |
| [2.3.11](../changelog/v2.3.11.md) | 2026-05-31 | Patch   | Patch: P1-2 radio dispatcher single-task enforcement (`ARES_REQUIRE`); P1-4 `encode()` bool + out-param; P3-1 `dxlr03` magic-number cleanup |
| [2.3.12](../changelog/v2.3.12.md) | 2026-05-31 | Patch   | Patch: P1-3 SIOF `main.cpp` (placement new, `startApiServer`); P2-1/P2-4 truncation guards; P2-5 `std::fill_n`; P3-2 `kCrcSuffix`; P2-2 single-task contract |
| [2.3.13](../changelog/v2.3.13.md) | 2026-05-31 | Patch   | Patch:  `tick()` flush — single-exit restructure; P3-1 `uint64_t` elapsed; 4 new SITL flush tests |
| [2.3.14](../changelog/v2.3.14.md) | 2026-05-31 | Patch   | Patch: checkpoint timing markers deferred to confirmed `writeFile()`; re-arm dirty on failure; 1 new SITL retry test |
| [2.3.15](../changelog/v2.3.15.md) | 2026-06-01 | Patch   | Patch: checkpoint atomicity; parser trailing-garbage rejection; `radio.config` `TokenCursor` refactor; sensor NaN guard; `std::fill_n`; API/storage defensive returns; BMP280 suite; clang-tidy PASS |
| [2.4.0](../changelog/v2.4.0.md)   | 2026-06-02 | Minor   | Minor: A2-3 `every Nms via ALIAS:` per-slot COM routing; `HkSlot.comAlias[16]`; runtime unresolved-alias `LOG_W`; 7 parser tests + 3 integration tests |
| [2.4.1](../changelog/v2.4.1.md)   | 2026-06-02 | Patch   | Patch: A2-4 null-iface driver detection at activation — `validateAliasIfacesLocked()`; 1 new lifecycle test |
