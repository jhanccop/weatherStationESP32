# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

PlatformIO/Arduino firmware for a battery-powered weather station built on a generic ESP32 Dev Module + SIM800 LTE/GPRS modem (via hardware `Serial2`, RX2=16/TX2=17), DHT22 (temp/humidity, GPIO4), MT6701 magnetic encoder over I2C (wind direction, SDA=21/SCL=22), pulse-counting anemometer (GPIO2), a rain bucket on an interrupt-capable input-only pin (GPIO39, EXT1 wakeup), an SD card on the default VSPI pins (MISO19/MOSI23/SCK18/CS5), and a resistive battery divider on an ADC pin (GPIO34, 330k/100k). Modem and sensor power rails are each gated by a P-channel MOSFET (GPIO15 for the modem, GPIO13 for sensors — LOW=ON, HIGH=OFF). All logic lives in the single file `src/main.cpp`. There is no test suite (`test/` is the empty PlatformIO scaffold) and no application-level lint config.

An earlier hardware revision used a LilyGO T-SIM7000G with different pins, RS485 solar radiation, and an SHT30 sensor — that variant has been fully replaced; don't assume old pin numbers or sensors from git history/memory still apply.

## Build / common commands

This is a PlatformIO project (not plain Arduino IDE), env name `esp32dev` defined in `platformio.ini`.

```bash
pio run                  # build
pio run -t upload        # build + flash over USB
pio run -t upload --upload-port /dev/ttyUSB0   # flash to a specific port
pio device monitor -b 115200   # serial monitor (matches Serial.begin(115200) in setup())
pio run -t clean         # clean build
```

There are no unit tests; validation is done by flashing and observing serial logs / the on-device web UI.

### Library dependencies live in `platformio.ini`

All third-party libraries are declared as `lib_deps` in `platformio.ini` and resolve from the PlatformIO registry — the project does **not** rely on `~/Arduino/libraries/` (an earlier revision did, informally, and the build was actually broken because of it; `pio run` failed with a missing `ArduinoJson.h` until `lib_deps` was added). Notable pins:
- `ArduinoJson @ ^6.21.5` — pinned to v6 because the code uses the v6 API (`StaticJsonDocument`, `DynamicJsonDocument`, `.containsKey()`), which v7 removed/deprecated.
- `me-no-dev/AsyncTCP @ ^1.1.1` + `me-no-dev/ESPAsyncWebServer @ ^1.2.3` — pinned to the old 1.2.x line, which predates the v3 rewrite that requires ArduinoJson 7.
- `i-am-engineer/MT6701-arduino` — same library that used to be installed globally; now pulled from the registry instead.

If you add a new sensor library, prefer adding it to `lib_deps` over installing it globally, so the build stays reproducible on a fresh machine. Verify with `pio run` after any dependency change — version mismatches between PubSubClient/ArduinoJson/ESPAsyncWebServer are a common source of build breaks here.

## Architecture

Single-file event-driven firmware structured around ESP32 deep sleep cycles. Read top-to-bottom in `src/main.cpp`, sections are marked with `/* ====== ... ====== */` banners.

### Power cycle (the core design)

The device spends almost all its time in `esp_deep_sleep_start()`. Each wake does one of three things, dispatched from `wakeup_reason()` (called from `setup()`):

- **Timer wakeup** → `init_running()`: normal cycle — read sensors, send data, go back to sleep.
- **EXT1 wakeup (GPIO39, rain bucket pulse)** → `runRainCounter()`: increments a SPIFFS-persisted rain counter, computes *remaining* sleep time so the next scheduled timer wake stays on schedule, then sleeps again. Uses `RTC_DATA_ATTR cycle_start_rtc` / `cycle_active` (survive deep sleep) plus `getRtcTimeSeconds()` (reads the ESP32 RTC slow clock directly) to track elapsed time across sleep/wake without a wall clock.
- **First boot / undefined wakeup** → also routed through `init_running()`.

`init_running()` itself branches on a hardware pin (`pinConfigAP`, GPIO36): if pulled low at boot, the device enters **config mode** (`setting()` — starts a WiFi AP + `ESPAsyncWebServer` config portal) instead of the sensor/send/sleep cycle.

GPIO state (the two power MOSFETs on GPIO13/GPIO15, the LED) is preserved across deep sleep using `gpio_hold_en()`/`gpio_hold_dis()` + `gpio_deep_sleep_hold_en()`. Any new code that drives those pins must release the hold before changing the pin and re-enable it before sleeping, or the new state won't stick / will conflict with prior state.

### Connectivity is dual-mode: WiFi or LTE

`connectionType` (persisted in SPIFFS as `/connType.txt`) selects between two parallel code paths everywhere data is sent or config is received:
- WiFi: `WiFi.h` + `wificlient` (`PubSubClient` over `WiFiClient`) — `sendDataWiFi()`, `wifiReconnect()`.
- LTE: `TinyGsmClient` (SIM800 via `SerialAT`/`Serial2`, RX2=16/TX2=17) + `simclient` (`PubSubClient` over `TinyGsmClient`) — `sendDataLTE()`, `simReconnect()`, plus modem power management (`modemPowerOn/Off/Restart`). Unlike the previous SIM7000G revision, the modem has no PWRKEY pulse logic — its entire power rail is gated by the P-channel MOSFET on GPIO15, so "off" is just driving that pin HIGH.

Both paths converge on the same MQTT topics (`jhpAI/FAUNA/data` publish, `jhpAI/FAUNA/settings` subscribe) and the same `callback()` handler for receiving remote config (the topic includes the device MAC as a suffix so each device only acts on its own settings push). When adding a feature that touches networking, mirror the change in both the WiFi and LTE variants.

### Sensors

Sensors are individually toggled via boolean flags (`A_TH`, `A_WS`, `A_PR` — temp/humidity, wind, rain) persisted in SPIFFS and configurable from the web UI or remote MQTT config push. `readSensors()` builds the JSON payload sent over MQTT; `updateSensCache()` is the parallel path that refreshes `sensCache` for the live `/sensors` JSON endpoint used by the config-mode web UI (non-blocking polling, refreshed every 15 s from `loop()`).

- DHT22 (temp/humidity, GPIO4) is a single-wire digital sensor — it does *not* share the I2C bus. Only the MT6701 (wind direction) uses I2C (SDA=21/SCL=22); `Wire.begin()` is gated behind `A_WS`, not `A_TH`.
- Wind speed (`WindVelocity()`) is a blocking ~3 s pulse-counting loop on GPIO2.
- Rain is a pulse counter on GPIO39 (EXT1 wakeup source, input-only RTC GPIO with no internal pull resistor), persisted and reset to 0 once read.
- Sensor power (DHT, wind, MT6701) is gated by a P-channel MOSFET on GPIO13; modem power by a separate one on GPIO15. Logic is inverted on both (`HIGH` = OFF, `LOW` = ON).
- An SD card on the default VSPI pins (MISO19/MOSI23/SCK18/CS5) is mounted at boot (`initSD()`) and used as a local backup log (`logToSD()` appends the JSON payload to `/datalog.txt` on every cycle in `init_running()`, independent of whether the MQTT send succeeds).

### Persistent config (SPIFFS)

All runtime settings (WiFi creds, MQTT broker/topics, sleep period, sensor enable flags, connection type, rain counter) are individual flat files in SPIFFS (e.g. `/timesleep.txt`, `/A_TH.txt`), read/written via the `readFile`/`writeFile`/`saveSetting` helpers. `createDefaultConfigs()` seeds defaults on first boot. There is no structured config blob (no single JSON config file) — each setting is its own file, written only when changed (`saveSetting` short-circuits on no-op writes to limit flash wear).

### Config-mode web UI

`index_html` (a single inlined `PROGMEM` HTML/CSS/JS string) is templated via `ESPAsyncWebServer`'s `processor()` callback (`%PLACEHOLDER%` substitution reading directly from SPIFFS files) and polls `/sensors` (JSON) every 5 s client-side. Form submits go through `/get` with query params; saving any setting redirects back to `/`. There's no separate static asset pipeline — editing the UI means editing the raw HTML string in `main.cpp`.
