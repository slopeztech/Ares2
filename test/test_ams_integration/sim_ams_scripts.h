/**
 * @file  sim_ams_scripts.h
 * @brief Inline AMS mission scripts for SITL integration tests.
 *
 * Scripts use SIM_GPS / SIM_BARO / SIM_COM / SIM_IMU as driver model names,
 * which match the identifiers returned by the corresponding SimXxxDriver
 * instances in test/stubs/hal_sim/.
 *
 * All scripts are stored as static const char arrays (no heap, PO10-3).
 */
#pragma once

namespace ares
{
namespace sim
{

/**
 * Minimal two-state script used by lifecycle tests.
 *
 * Flow: WAIT --(TC LAUNCH)--> END
 *
 * The END state is terminal (no transitions, no periodic actions) so the
 * engine self-transitions to COMPLETE after one tick there.
 */
static const char kScriptLifecycle[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  priorities event=4 hk=3 log=1 budget=2\n"
    "  transition to END when TC.command == LAUNCH\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"MISSION COMPLETE\"\n"
    "  priorities event=4 hk=1 log=1 budget=1\n";

/**
 * Time-driven three-state script used by scenario tests.
 *
 * Flow:  WAIT --(TC LAUNCH)--> FLIGHT --(5 s elapsed)--> END
 *
 * The FLIGHT state emits HK telemetry every 1000 ms (via COM/radio).
 * The TIME.elapsed transition fires after 5000 ms of sim-clock time.
 */
static const char kScriptFlight[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING FOR LAUNCH\"\n"
    "  priorities event=4 hk=3 log=1 budget=2\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  on_enter:\n"
    "    EVENT.info \"LIFTOFF\"\n"
    "  priorities event=4 hk=3 log=1 budget=2\n"
    "  every 1000ms:\n"
    "    HK.report {\n"
    "      gps_alt: GPS.alt\n"
    "      baro_alt: BARO.alt\n"
    "      baro_temp: BARO.temp\n"
    "    }\n"
    "  transition to END when TIME.elapsed > 5000\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"MISSION COMPLETE\"\n"
    "  priorities event=4 hk=1 log=1 budget=1\n";

/**
 * Sensor-driven script used by altitude transition tests.
 *
 * Flow:  WAIT --(TC LAUNCH)--> ASCENT --(BARO.alt > 200)--> RECOVERY --> END
 *
 * Tests that sensor-based transition conditions are correctly evaluated
 * against the sim flight profile values.
 */
static const char kScriptSensorTransition[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  priorities event=4 hk=1 log=1 budget=1\n"
    "  transition to ASCENT when TC.command == LAUNCH\n"
    "\n"
    "state ASCENT:\n"
    "  on_enter:\n"
    "    EVENT.info \"ASCENDING\"\n"
    "  priorities event=4 hk=2 log=1 budget=2\n"
    "  every 500ms:\n"
    "    HK.report {\n"
    "      baro_alt: BARO.alt\n"
    "    }\n"
    "  transition to RECOVERY when BARO.alt > 200\n"
    "\n"
    "state RECOVERY:\n"
    "  on_enter:\n"
    "    EVENT.info \"RECOVERY\"\n"
    "  priorities event=4 hk=2 log=1 budget=2\n"
    "  every 2000ms:\n"
    "    HK.report {\n"
    "      gps_lat: GPS.lat\n"
    "      gps_lon: GPS.lon\n"
    "      gps_alt: GPS.alt\n"
    "    }\n"
    "  transition to END when TIME.elapsed > 3000\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"LANDED\"\n"
    "  priorities event=4 hk=1 log=1 budget=1\n";

/**
 * Script with an explicit ABORT transition in the FLIGHT state.
 *
 * Flow:  WAIT --(LAUNCH)--> FLIGHT --(ABORT)--> SAFE
 *                                   --(30 s)-->  END
 *
 * Used to verify that an ABORT TC is *consumed* by a matching transition
 * instead of triggering engine force-deactivation.
 */
static const char kScriptAbortTransition[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  on_enter:\n"
    "    EVENT.info \"FLIGHT\"\n"
    "  every 2000ms:\n"
    "    HK.report {\n"
    "      baro_alt: BARO.alt\n"
    "    }\n"
    "  transition to SAFE when TC.command == ABORT\n"
    "  transition to END when TIME.elapsed > 30000\n"
    "\n"
    "state SAFE:\n"
    "  on_enter:\n"
    "    EVENT.info \"SAFE MODE\"\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"END\"\n";

/**
 * Script with a fallback transition.
 *
 * Flow:  WAIT --(LAUNCH)--> FLIGHT --(fallback after 2000 ms)--> SAFE
 *
 * Used to verify that the fallback fires unconditionally when no regular
 * transition has fired within the declared timeout (AMS-4.9.2).
 */
static const char kScriptFallback[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  on_enter:\n"
    "    EVENT.info \"FLIGHT\"\n"
    "  every 2000ms:\n"
    "    HK.report {\n"
    "      baro_alt: BARO.alt\n"
    "    }\n"
    "  fallback transition to SAFE after 2000ms\n"
    "\n"
    "state SAFE:\n"
    "  on_enter:\n"
    "    EVENT.info \"SAFE\"\n";

/**
 * Script whose FLIGHT state has a guard condition that fails at ground level.
 *
 * Guard: BARO.alt > 1000 (holds when alt > 1000 m; violated at ground).
 *
 * Used to verify that a guard violation forces EngineStatus::ERROR (AMS-4.7).
 */
static const char kScriptGuardViolation[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  conditions:\n"
    "    BARO.alt > 1000\n"
    "  on_error:\n"
    "    EVENT.error \"GUARD FAIL\"\n"
    "  every 2000ms:\n"
    "    HK.report {\n"
    "      baro_alt: BARO.alt\n"
    "    }\n"
    "  transition to END when TIME.elapsed > 30000\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"END\"\n";

/**
 * Script whose FLIGHT on_error block redirects to SAFE instead of halting.
 *
 * Same guard as kScriptGuardViolation but adds the AMS-4.10.2 recovery
 * transition so the engine enters SAFE rather than ERROR status.
 */
static const char kScriptGuardRecovery[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  conditions:\n"
    "    BARO.alt > 1000\n"
    "  on_error:\n"
    "    EVENT.error \"GUARD FAIL\"\n"
    "    transition to SAFE\n"
    "  every 2000ms:\n"
    "    HK.report {\n"
    "      baro_alt: BARO.alt\n"
    "    }\n"
    "  transition to END when TIME.elapsed > 30000\n"
    "\n"
    "state SAFE:\n"
    "  on_enter:\n"
    "    EVENT.info \"SAFE\"\n"
    "\n"
    "state END:\n"
    "  on_enter:\n"
    "    EVENT.info \"END\"\n";

/**
 * Script with a persistence hold window on the altitude transition.
 *
 * Flow:  WAIT --(LAUNCH)--> FLIGHT --(BARO.alt > 50 for 1000 ms)--> HIGH_ALT
 *
 * With the sim flight profile, BARO.alt exceeds 50 m at approximately 2500 ms
 * and the 1000 ms hold window closes around 3500 ms.
 * Used to verify `for Nms` semantics (AMS-4.6.1) and the Bug #5 regression
 * (hold window must be cleared when the engine is paused).
 */
static const char kScriptHoldWindow[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FLIGHT when TC.command == LAUNCH\n"
    "\n"
    "state FLIGHT:\n"
    "  on_enter:\n"
    "    EVENT.info \"FLIGHT\"\n"
    "  every 2000ms:\n"
    "    HK.report {\n"
    "      baro_alt: BARO.alt\n"
    "    }\n"
    "  transition to HIGH_ALT when BARO.alt > 50 for 1000ms\n"
    "\n"
    "state HIGH_ALT:\n"
    "  on_enter:\n"
    "    EVENT.info \"HIGH_ALT\"\n";

/**
 * Script that requires the LAUNCH TC to be injected twice before firing.
 *
 * Uses the CONFIRM N debounce modifier (AMS-4.11.3).
 * Flow:  WAIT --(LAUNCH confirm 2)--> FLIGHT
 *
 * A single injectTcCommand("LAUNCH") is insufficient; two are required.
 */
static const char kScriptConfirmLaunch[] =
    "include SIM_GPS as GPS\n"
    "include SIM_BARO as BARO\n"
    "include SIM_COM as COM\n"
    "include SIM_IMU as IMU\n"
    "\n"
    "pus.apid = 1\n"
    "\n"
    "pus.service 3 as HK\n"
    "pus.service 5 as EVENT\n"
    "pus.service 1 as TC\n"
    "\n"
    "state WAIT:\n"
    "  on_enter:\n"
    "    EVENT.info \"WAITING\"\n"
    "  transition to FLIGHT when TC.command == LAUNCH confirm 2\n"
    "\n"
    "state FLIGHT:\n"
    "  on_enter:\n"
    "    EVENT.info \"FLIGHT\"\n";

} // namespace sim
} // namespace ares
