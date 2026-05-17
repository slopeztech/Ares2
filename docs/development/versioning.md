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
#define ARES_VERSION_STRING "2.1.0"
```

---

## Changelog

Each version has its own file under `docs/changelog/`:

```
docs/changelog/
    v2.0.0.md   ← first structured release
    v2.1.0.md   ← pulse.channel / 4 channels / pin rename
```

File names follow the pattern `vMAJOR.MINOR.PATCH.md`.

---

## Version history

| Version | Date       | Type  | Summary                                                       |
|---------|------------|-------|---------------------------------------------------------------|
| [2.0.0](../changelog/v2.0.0.md) | 2026-05-11 | Release | First structured release; modular AMS; REST auth             |
| [2.1.0](../changelog/v2.1.0.md) | 2026-05-17 | Minor | AMS-4.18 `pulse.channel`; 4 pulse channels; pin rename       |
