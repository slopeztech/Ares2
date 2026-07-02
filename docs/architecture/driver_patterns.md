# Driver Architecture & Hardware Abstraction

**Version:** 2.6.5+  
**Scope:** Platform-agnostic hardware interface pattern; static driver registry; extensibility framework

---

## 1. Overview

ARES2 uses a **static registry pattern** for hardware drivers, eliminating dynamic allocation
while maintaining type safety and future extensibility. All platform-specific implementations
(sensors, radio, serial I/O, etc.) are registered in a central, bounded registry at bootstrap time
and can be queried by subsystems without direct coupling to concrete driver classes.

**Core principle:**
> "One module per hardware interface; one registry to bind them all."

---

## 2. Driver Architecture: 5-Layer Stack

```
┌──────────────────────────────────────────────────┐
│ Application Layer (e.g., AMS Engine)             │ (consumes drivers from registry)
├──────────────────────────────────────────────────┤
│ Selector Layer (e.g., GpsSelector, BaroSelector) │ (queries registry for runtime driver choice)
├──────────────────────────────────────────────────┤
│ Hardware Registry (installed_driver_registry)    │ (single source of truth)
├──────────────────────────────────────────────────┤
│ Driver Implementation Layer                      │ (concrete drivers: BN220, BMP280, etc.)
├──────────────────────────────────────────────────┤
│ HAL Interface Layer (pure virtual interfaces)    │ (GpsInterface, BarometerInterface, etc.)
└──────────────────────────────────────────────────┘
```

---

## 3. HAL Interface Layer

Each hardware capability is defined by a **pure virtual interface** (contract):

```cpp
// src/hal/gps/gps_interface.h
class GpsInterface {
public:
    virtual ~GpsInterface() = default;
    virtual GpsReadingStatus read(GpsReading* out) = 0;
    virtual void begin() = 0;
    virtual void end() = 0;
};

// src/hal/serial/serial_interface.h
class SerialInterface {
public:
    virtual ~SerialInterface() = default;
    virtual uint32_t availableForWrite() const = 0;
    virtual uint32_t write(const uint8_t* data, uint32_t len) = 0;
};
```

**Design principles:**
- **Pure virtual** (no implementations, no instance variables).
- **Minimal surface** (only methods essential to the consumer).
- **Nullable-friendly** (no constructor side effects).
- **No allocations** (pointers are by reference, lifetimes managed by bootstrapper).

---

## 4. Driver Implementation Layer

A **concrete driver** implements the interface and encapsulates platform-specific details:

```cpp
// src/drivers/gps/bn220_driver.h
class Bn220Driver : public GpsInterface {
public:
    explicit Bn220Driver(HardwareSerial& uart, int rx, int tx, uint32_t baud);
    GpsReadingStatus read(GpsReading* out) override;
    void begin() override;
    void end() override;
private:
    HardwareSerial& uart_;
    // ... UART state, parser, etc.
};

// src/drivers/serial/arduino_serial_interface.h
class ArduinoSerialInterface : public SerialInterface {
public:
    explicit ArduinoSerialInterface(Print& serial);
    uint32_t availableForWrite() const override;
    uint32_t write(const uint8_t* data, uint32_t len) override;
private:
    Print& serial_;
};
```

**Characteristics:**
- Static allocation (no `new`).
- Receives dependencies by reference in constructor (no ownership transfer).
- Implements all interface methods (linker will catch any missing `override`).
- Private state managed safely (no leaks, no dangling pointers).

---

## 5. Hardware Registry

The **central registry** holds a bounded list of all installed drivers and their interface
pointers. It serves as the **single source of truth** for hardware availability and naming.

### 5.1 Registry Structure

**File:** `src/sys/hardware/installed_driver_registry.h`

```cpp
namespace ares::hardware {

// DriverKind enumeration — names the subsystems ARES2 supports
enum class DriverKind {
    GPS,       // Global Positioning System
    BARO,      // Barometer (altitude)
    COM,       // Communication (radio)
    IMU,       // Inertial Measurement Unit
    SERIAL_IO, // Serial output
};

// Entry template: (model name, interface pointer)
template <typename InterfaceType>
struct DriverEntry {
    const char* modelName;
    InterfaceType* iface;
};

// Type aliases for each driver category
using GpsDriverEntry = DriverEntry<GpsInterface>;
using BaroDriverEntry = DriverEntry<BarometerInterface>;
using ComDriverEntry = DriverEntry<RadioInterface>;
using ImuDriverEntry = DriverEntry<ImuInterface>;
using SerialDriverEntry = DriverEntry<SerialInterface>;

// List descriptor: array metadata
template <typename EntryType>
struct DriverList {
    EntryType* entries;
    uint32_t count;
};

// References to all installed drivers — passed to bindInstalledHardware()
struct InstalledHardwareRefs {
    BarometerInterface* barometer;
    GpsInterface* gps;
    RadioInterface* radio;
    ImuInterface* imu1;
    ImuInterface* imu2;
    SerialInterface* serialOut;
};

} // namespace ares::hardware
```

### 5.2 Registry Binding

**File:** `src/sys/hardware/installed_driver_registry.cpp`

At bootstrap time (typically in `main.cpp`), all concrete driver instances are
bound into the registry:

```cpp
static Bn220Driver gps(...);
static Bmp280Driver baro(...);
static DxLr03Driver radio(...);
static Mpu6050Driver imu1(...);
static Mpu6050Driver imu2(...);
static ArduinoSerialInterface serialOut(Serial);

void bindInstalledHardware(const InstalledHardwareRefs& refs) {
    // Bind each driver category's interface pointer(s)
    g_baroEntries[0].iface = refs.barometer;
    g_gpsEntries[0].iface = refs.gps;
    g_comEntries[0].iface = refs.radio;
    g_imuEntries[0].iface = refs.imu1;
    g_imuEntries[1].iface = refs.imu2;
    g_serialEntries[0].iface = refs.serialOut;
}
```

### 5.3 Registry Accessors

Public query functions allow subsystems to discover available drivers:

```cpp
// Query by driver category
const DriverList<GpsDriverEntry>& gpsDrivers();
const DriverList<BaroDriverEntry>& baroDrivers();
const DriverList<ComDriverEntry>& comDrivers();
const DriverList<ImuDriverEntry>& imuDrivers();
const DriverList<SerialDriverEntry>& serialDrivers();

// Query default / recommended driver for a category
const char* defaultGpsDriverModel();
const char* defaultBaroDriverModel();
const char* defaultComDriverModel();
const char* defaultSerialDriverModel();
```

---

## 6. Selector Layer

**Selectors** are optional components that allow runtime driver switching without
recompilation. They query the registry for available drivers and maintain a user-selected
choice (often persisted in device configuration).

```cpp
// src/sys/gps/gps_selector.h
class GpsSelector {
public:
    explicit GpsSelector(GpsDriverEntry* entries, uint32_t count, uint32_t defaultIdx);
    
    // Query available drivers
    const GpsDriverEntry& driver(uint32_t idx) const;
    uint32_t driverCount() const;
    
    // Get currently selected driver's interface
    GpsInterface* selectedDriver() const;
    
    // Change selection at runtime
    void selectDriver(uint32_t idx);
};
```

Selectors are used by:
- **REST API:** `GET /api/device/config/gps-driver` — list and select drivers
- **Platform bootstrap:** `main.cpp` creates selectors and initializes from persistent config

---

## 7. Application Layer Integration

Subsystems consume drivers from the registry or through selectors, avoiding hard dependencies
on concrete classes.

### 7.1 Direct Registry Query

When a subsystem needs the serial output interface (v2.6.4+):

```cpp
// src/ams/mission_script_engine.cpp
void MissionScriptEngine::setSerialInterface(SerialInterface* serialIface) {
    serialIface_ = serialIface;  // Nullable — engine handles nil gracefully
}

// In MissionScriptEngine telemetry handler:
if (serialIface_ != nullptr && serialIface_->availableForWrite() >= kMinBytesNeeded) {
    serialIface_->write(data, len);
}
```

### 7.2 Selector-Based Selection

When the API or configuration manager needs to select a GPS driver:

```cpp
// src/sys/gps/gps_selector.h (consumed by API)
GpsInterface* gps = gpsSelector_.selectedDriver();
if (gps != nullptr) {
    GpsReading reading;
    gps->read(&reading);
}
```

---

## 8. Adding a New Driver Category: The Serial Example

Here's how v2.6.4 extended ARES2 to support pluggable serial transport:

### 8.1 Define the Interface

```cpp
// src/hal/serial/serial_interface.h
class SerialInterface {
public:
    virtual ~SerialInterface() = default;
    virtual uint32_t availableForWrite() const = 0;
    virtual uint32_t write(const uint8_t* data, uint32_t len) = 0;
};
```

### 8.2 Implement Concrete Drivers

```cpp
// src/drivers/serial/arduino_serial_interface.h
class ArduinoSerialInterface : public SerialInterface {
    // ... implementation
};

// Future: add CDC serial driver, ESP-IDF UART driver, etc.
// class CdcSerialDriver : public SerialInterface { ... };
// class EspUartDriver : public SerialInterface { ... };
```

### 8.3 Extend the Registry

```cpp
// src/sys/hardware/installed_driver_registry.h
enum class DriverKind {
    GPS, BARO, COM, IMU,
    SERIAL_IO,  // ← NEW
};

using SerialDriverEntry = DriverEntry<SerialInterface>;

struct InstalledHardwareRefs {
    // ... existing fields ...
    SerialInterface* serialOut;  // ← NEW
};
```

### 8.4 Bind at Bootstrap

```cpp
// src/main.cpp
static ArduinoSerialInterface serialOut(Serial);

bindInstalledHardware({
    pBaro, &gps, &radio, &imu, &imu2,
    &serialOut  // ← NEW
});

missionEngine.setSerialInterface(
    ares::hardware::serialDrivers().entries[0].iface
);
```

### 8.5 Register in AMS

```cpp
// src/ams/ams_driver_registry.h
using SerialEntry = ares::hardware::SerialDriverEntry;
```

Now AMS scripts can reference drivers:
```ams
include SERIAL0 as DBG  // Future: slot-selectable serial output
```

---

## 9. Design Constraints & Trade-offs

| Constraint | Reason | Trade-off |
|---|---|---|
| **Static allocation only** | Real-time + PO10-3 (no heap) | Fixed driver count at compile time |
| **Single registry** | Deterministic lookup, no search overhead | All drivers must be known at bootstrap |
| **Bounded lists** | Cache-friendly, no iteration surprises | Max ~10 drivers per category sufficient for ARES2 |
| **Nullable interfaces** | Graceful degradation (e.g., SERIAL.report without serial) | Every consumer must handle `nullptr` |
| **Pure virtual only** | No default implementations, linker safety | Must implement every method in concrete class |

---

## 10. Testing & Validation

### 10.1 Unit Tests

Each driver's test suite validates:
- Interface contract compliance (`override` keyword, return types).
- MISRA/CERT adherence (no casts, no dangling pointers).
- Resource lifecycle (begin/end symmetry, no leaks).

```cpp
// test/test_gps_drivers/test_bn220_driver.cpp
TEST(Bn220DriverTest, ImplementsGpsInterface) {
    Bn220Driver driver(mockSerial, ...);
    // Verify it can be used as GpsInterface*
    GpsInterface* iface = &driver;
    ASSERT_NE(nullptr, iface);
}
```

### 10.2 Registry Tests

Registry binding is tested in `test_ams_integration/test_ams_lifecycle.cpp`:

```cpp
TEST(AmsIntegrationTest, RegistryBindingWithSerialDriver) {
    // Verify serialDrivers() returns bound interface
    auto serials = ares::hardware::serialDrivers();
    EXPECT_GT(serials.count, 0U);
    EXPECT_NE(nullptr, serials.entries[0].iface);
}
```

### 10.3 Integration Tests

AMS telemetry generation tests verify:
- Graceful behavior when serial interface is `nullptr`.
- Backpressure drops under congestion (silent, no blocking).
- Output correctness (frame format, field values).

---

## 11. Future Extensibility

This pattern supports adding new drivers with **zero changes to existing code**:

1. **New sensor category (e.g., Magnetometer)?**
   - Add `MagnetometerInterface` in `src/hal/mag/`.
   - Implement concrete driver(s) in `src/drivers/mag/`.
   - Extend registry enum, struct, and accessor functions.
   - Bind in `main.cpp` and wire through AMS.

2. **Alternative serial transport (e.g., CDC)?**
   - Implement `class CdcSerialDriver : public SerialInterface`.
   - Add entry to `g_serialEntries[]` in registry.
   - Expose selector in API for runtime switching.

3. **Stubbed drivers for testing?**
   - Implement `class StubGpsDriver : public GpsInterface` in test fixtures.
   - Swap in via registry at test bootstrap.
   - No production code changes needed.

---

## 12. References

- **MISRA C++:2023** — Dynamic memory allocation, rule A1-4-1 (avoided via static registry).
- **CERT C++** — Resource Management, recommendation MEM50-CPP (static lifetime, registry-mediated access).
- **ARES2 Code:** [`src/sys/hardware/installed_driver_registry.h`](../../src/sys/hardware/installed_driver_registry.h)
- **Examples:**
  - GPS driver integration: [`src/drivers/gps/bn220_driver.h`](../../src/drivers/gps/bn220_driver.h)
  - Serial driver integration: [`src/drivers/serial/arduino_serial_interface.h`](../../src/drivers/serial/arduino_serial_interface.h)
  - AMS usage: [`src/ams/mission_script_engine.cpp`](../../src/ams/mission_script_engine.cpp)
