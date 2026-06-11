# Solar Weather Station - Low Power Update Notes

## Major changes

- Moved WiFi/MQTT credentials from `src/main.cpp` to `include/secrets.h`.
- Added adaptive deep sleep intervals:
  - Normal: 10 minutes
  - Low battery: 60 minutes
  - Empty battery: 240 minutes
- Added `RTC_DATA_ATTR wakeCount` so PMS3003 is read only every 6 wake cycles.
  - Default: 10 minutes x 6 = once per 60 minutes.
- PMS3003 is skipped when battery SoC is at or below 30%.
- Added WiFi/MQTT timeout handling and radio shutdown before deep sleep.
- Changed battery SoC calculation from linear math to a basic 1S Li-ion lookup table.
- Added battery ADC calibration constants.
- Fixed `CHARGE_EN` behavior by disabling it unless explicitly enabled.
- Fixed invalid percent formatting in Serial output.
- PMS UART is ended and pins are set to input before sleep to reduce back-power risk.

## Required hardware check

The two 18650 cells must be connected as 1S2P parallel pack:

- Nominal voltage: 3.7 V
- Full charge: 4.2 V

Do not connect two 18650 cells in series with this CN3791 1S charging setup.

## Important hardware recommendation

The current diagram uses a BC337 low-side GND switch for sensors. For better deep sleep current and fewer back-power issues, change PMS3003 power control to a high-side P-channel MOSFET or a load switch IC.

## Calibration

Compare `vBatt` from Serial output with a multimeter reading and adjust this constant in `src/main.cpp`:

```cpp
#define BATT_CALIBRATION 1.00f
```

Example: if Serial shows 3.90 V but multimeter shows 4.00 V:

```cpp
#define BATT_CALIBRATION 1.026f
```
