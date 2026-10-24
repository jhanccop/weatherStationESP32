# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

PlatformIO/Arduino firmware for a battery-powered weather station built on a **LilyGO TTGO T-SIM7000G** (ESP32 + on-board SIM7000G LTE CAT-M/NB-IoT/GSM modem on `Serial1`, RX=26/TX=27, PWRKEY=4), a DHT21/AM2301 temp/humidity sensor (`DHT_TYPE DHT21`, GPIO23), a wind-direction encoder selectable in software between MT6701 and AS5600 over I2C (SDA=21/SCL=22), a Hall-effect pulse-counting anemometer (GPIO35, input-only, external 10k pull-up to vext1), a Hall-effect rain bucket (GPIO34, input-only, external 10k pull-up to 3V3, EXT1 wakeup), an RS485 solar-radiation sensor on its own UART (`Serial2`, TX=32/RX=33), the board's built-in SD slot (MISO2/MOSI15/SCK14/CS13), and a resistive battery divider on GPIO39 (390k/100k). Two P-channel MOSFETs (AO3401) gate the peripheral rails, both LOW=ON/HIGH=OFF: GPIO18 (`vext_ctrl_1`) → vext1 (DHT, anemometer, encoder, ADS1115) and GPIO19 (`vext_ctrl_2`) → vext2 (3.7→5V boost for the radiation sensor + RS485 module). The schematic also has an ADS1115 on I2C with two analog inputs (A0/A1) that the firmware does not read yet. All logic lives in the single file `src/main.cpp`. There is no test suite (`test/` is the empty PlatformIO scaffold) and no application-level lint config. Code comments are in Spanish.

This is the `main-sim7000` branch. `main` targets a different board (generic ESP32 Dev Module + external SIM800 behind a MOSFET, with different pins) — don't carry pin numbers or modem power logic between branches. Trust the `#define`s and the pinout comment block at the top of `main.cpp` over older memory of this file.

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
- `i-am-engineer/MT6701-arduino` + `robtillaart/AS5600` — both wind-direction encoder drivers are always compiled in; which one is actually read at runtime is chosen by `dirSensorType`, not by a build flag.

If you add a new sensor library, prefer adding it to `lib_deps` over installing it globally, so the build stays reproducible on a fresh machine. Verify with `pio run` after any dependency change — version mismatches between PubSubClient/ArduinoJson/ESPAsyncWebServer are a common source of build breaks here.

## Architecture

Single-file event-driven firmware structured around ESP32 deep sleep cycles. Read top-to-bottom in `src/main.cpp`, sections are marked with `/* ====== ... ====== */` banners.

### Power cycle (the core design)

The device spends almost all its time in `esp_deep_sleep_start()`. Each wake does one of three things, dispatched from `wakeup_reason()` (called from `setup()`):

- **Timer wakeup** → `init_running()`: normal cycle — read sensors, send data, go back to sleep.
- **EXT1 wakeup (GPIO34, rain bucket pulse)** → `runRainCounter()`: increments a SPIFFS-persisted rain counter, computes *remaining* sleep time so the next scheduled timer wake stays on schedule, then sleeps again. Uses `RTC_DATA_ATTR cycle_start_rtc` / `cycle_active` (survive deep sleep) plus `getRtcTimeSeconds()` (reads the ESP32 RTC slow clock directly) to track elapsed time across sleep/wake without a wall clock. Since EXT1 only fires when the pin was already LOW (field present), the wake itself is treated as ground truth — the only remaining filter is `RTC_DATA_ATTR lastRainPulseRtcTicks`, which enforces a 300 ms minimum interval (`RAIN_PULSE_MIN_INTERVAL_MS`) between counted tips so wakeup re-arming on a bouncing pin doesn't double-count one real tip. If the configured sleep period has already elapsed by the time a rain wake is serviced, it falls through to running the pending `init_running()` cycle immediately instead of just rescheduling — otherwise a noisy rain pin could keep deferring the real sensor/send cycle indefinitely.
- **First boot / undefined wakeup** → also routed through `init_running()`.

`init_running()` itself branches on a hardware pin (`pinConfigAP`, GPIO36 — input-only, no internal pull, requires an external pull-up to 3.3V; the config button pulls it to GND): if held LOW across 10 debounced reads at boot, the device enters **config mode** (`setting()` — starts a WiFi AP + `ESPAsyncWebServer` config portal) instead of the sensor/send/sleep cycle.

GPIO state (the two rail MOSFET gates on GPIO18/GPIO19, the modem PWRKEY on GPIO4, and the RS485 TX/RX lines on 32/33) is preserved across deep sleep using `gpio_hold_en()`/`gpio_hold_dis()` + `gpio_deep_sleep_hold_en()`. Any new code that drives those pins must release the hold before changing the pin and re-enable it before sleeping, or the new state won't stick / will conflict with prior state. `rs485LinesOff()` forces the RS485 TX/RX to output-LOW-and-held whenever vext2 is off, so the unpowered module isn't backfed through its UART pins. `modemLinesOff()` holds PWRKEY LOW (idle) and leaves the modem UART pins as inputs — they are *not* driven LOW, since the board's level shifters have pull-ups and a held LOW would leak current all sleep.

### Connectivity is dual-mode: WiFi or LTE

`connectionType` (persisted in SPIFFS as `/connType.txt`) selects between two parallel code paths everywhere data is sent or config is received:
- WiFi: `WiFi.h` + `wificlient` (`PubSubClient` over `WiFiClient`) — `sendDataWiFi()`, `wifiReconnect()`.
- LTE: `TinyGsmClient` (`TINY_GSM_MODEM_SIM7000`, on-board SIM7000G via `SerialAT`/`Serial1`, RX=26/TX=27, 115200 baud) + `simclient` (`PubSubClient` over `TinyGsmClient`) — `sendDataLTE()`, `simReconnect()`, plus modem power management (`modemPowerOn/Off/Restart`). The modem is powered straight from the battery (no MOSFET), so power is controlled only via PWRKEY (GPIO4, pulse HIGH ~1 s through the board's transistor) and `modem.poweroff()` (AT+CPOWD). PWRKEY is a *toggle*: `modemPowerOn()` first checks `testAT()` and only pulses if the modem isn't already answering, otherwise a stale-on modem would be switched off. `modemPowerOff()` must run before every sleep in which the modem was used, or it stays registered and drains the battery; it falls back to a PWRKEY pulse if AT+CPOWD gets no reply. `sendDataLTE()` sets network mode auto (GSM/LTE) and CAT-M+NB-IoT preference.

Both paths converge on the same MQTT topics (`jhpAI/FAUNA/data` publish, `jhpAI/FAUNA/settings` subscribe) and the same `callback()` handler for receiving remote config (the topic includes the device MAC as a suffix so each device only acts on its own settings push). When adding a feature that touches networking, mirror the change in both the WiFi and LTE variants.

**Offline resilience:** if a publish fails, `init_running()` calls `appendPendingRecord()` to append the JSON payload as a line to `/pending.jsonl` in SPIFFS instead of dropping it. Before publishing the *current* reading, both `sendDataWiFi()` and `sendDataLTE()` call `flushPendingData()`, which replays pending lines in order over the already-connected MQTT client; on the first failed publish it stops and rewrites the file with only the unsent lines (via a `/pending.tmp` swap) so ordering is preserved and nothing is retried out of order.

**Time sync without an RTC chip:** this hardware has no battery-backed RTC, GPS, or NTP path, so real (epoch) time is bootstrapped from whatever device connects to the config-mode AP — the config page posts the browser's clock to `/settime`, which anchors `timeBaseEpoch`/`timeBaseRtc` against `getRtcTimeSeconds()` (the RTC slow-clock, which does survive deep sleep). Every later timestamp is computed by adding elapsed RTC-clock seconds to that anchor, so it keeps working across sleep cycles without needing a live network connection each time. `syncTimeFromInternetWiFi()`/`syncTimeFromInternetLTE()` opportunistically re-anchor after a successful publish when network time is available.

### Sensors

Sensors are individually toggled via boolean flags (`A_TH`, `A_WS`, `A_PR` — temp/humidity, wind, rain) persisted in SPIFFS and configurable from the web UI or remote MQTT config push. `readSensors()` builds the JSON payload sent over MQTT; `updateSensCache()` is the parallel path that refreshes `sensCache` for the live `/sensors` JSON endpoint used by the config-mode web UI (non-blocking polling, refreshed every 15 s from `loop()`).

- The temp/humidity sensor (GPIO23) is compiled as `DHT_TYPE DHT21` (AM2301) — a single-wire digital sensor that does *not* share the I2C bus. Readings outside the DHT22/AM2301 physical range (`DHT22_TEMP_MIN/MAX`, `DHT22_HUM_MIN/MAX`) are discarded rather than propagated.
- Wind direction is read from whichever encoder `dirSensorType` selects (`"MT6701"` or `"AS5600"`, persisted in `/dirSensor.txt`, settable from the web UI or MQTT config push) — both drivers are always initialized/compiled, `WindDirection()` just branches at read time. Only this subsystem uses I2C (SDA=21/SCL=22); `Wire.begin()` is gated behind `A_WS`, not `A_TH`.
- Wind speed (`WindVelocity()`) is a blocking ~3 s falling-edge pulse-counting loop on GPIO35 (Hall sensor, push-pull; GPIO35 has no internal pull, so the pin is plain `INPUT` and the external 10k pull-up to vext1 holds a solid HIGH when disconnected). Pulses are converted via `WIND_SPEED_PER_PULSE` (cup radius + empirically wind-tunnel-derived `WIND_CAL_FACTOR`), debounced by `WIND_PULSE_DEBOUNCE_US` (8 ms — a stationary magnet sitting on the sensor can chatter across the comparator threshold), and any result above `MAX_WIND_SPEED_MS` is discarded as sensor noise rather than reported. `updateSensCache()`'s non-blocking config-mode variant uses the same debounce constant via `windPulseISR()`.
- Rain is a Hall sensor on GPIO34 (EXT1 wakeup source, input-only RTC GPIO with no internal pull resistor; the board provides a 10k pull-up to 3V3). Inherited from the earlier pull-less wiring, code prefers reading raw ADC millivolts (`RAIN_ADC_FIELD_MV`/`RAIN_ADC_IDLE_MV`, with a 500–1500 mV dead zone) over `digitalRead()` where possible; the persisted count is reset to 0 once read. See `runRainCounter()` (deep-sleep path) and `rainPulseISR()` (config-mode live-counting path, which re-implements the same two filters since config mode never sleeps).
- Peripheral power is split across two P-channel MOSFET rails (`HIGH` = OFF, `LOW` = ON): vext1 via GPIO18 (`SENS_PWR_PIN`: DHT, anemometer, wind direction encoder, ADS1115) and vext2 via GPIO19 (`RS485_PWR_PIN`: 5V boost feeding the radiation sensor and RS485 module). The RS485 radiation sensor (`A_RAD`) is Modbus RTU on `Serial2` (TX=32/RX=33, 4800 baud), switched on/off by `initRS485()`/`deinitRS485()`.
- Battery voltage (`vBat()`, GPIO39) reads `analogReadMilliVolts()` through a 390k/100k divider (×4.9) plus a 2-point calibration (`BAT_CAL_SLOPE`/`BAT_CAL_OFFSET`). On this board those constants are still the theoretical placeholders (4.9 / 0) — recalibrate against a multimeter.
- The board's built-in SD slot (MISO2/MOSI15/SCK14/CS13) is mounted at boot (`initSD()`) and used as a local backup log (`logToSD()` appends the JSON payload to `/datalog.csv` on every cycle in `init_running()`, independent of whether the MQTT send succeeds). `formatSD()` uses the ESP-IDF FAT VFS APIs directly (not the Arduino `SD.h` API, which has no format operation) to recover cards without a valid filesystem.

### Persistent config (SPIFFS)

All runtime settings (WiFi creds, MQTT broker/topics, sleep period, sensor enable flags, wind-direction sensor selection, connection type, APN, rain counter, time-base anchor) are individual flat files in SPIFFS (e.g. `/timesleep.txt`, `/A_TH.txt`, `/dirSensor.txt`), read/written via the `readFile`/`writeFile`/`saveSetting` helpers. `createDefaultConfigs()` seeds defaults on first boot. There is no structured config blob (no single JSON config file) — each setting is its own file, written only when changed (`saveSetting` short-circuits on no-op writes to limit flash wear).

### Config-mode web UI

`index_html` (a single inlined `PROGMEM` HTML/CSS/JS string) is templated via `ESPAsyncWebServer`'s `processor()` callback (`%PLACEHOLDER%` substitution reading directly from SPIFFS files) and polls `/sensors` (JSON) every 5 s client-side. Form submits go through `/get` with query params; saving any setting redirects back to `/`. The page also posts the browser's clock to `/settime` on load to anchor real time (see time-sync note above). There's no separate static asset pipeline — editing the UI means editing the raw HTML string in `main.cpp`.
