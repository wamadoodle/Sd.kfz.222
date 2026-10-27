# SdKfz222 Project Handoff Prompt

You are taking over the SdKfz222 ESP32 robot project in this workspace. Read the current source and configuration before changing anything. Treat the hardware history below as user-reported evidence, not as proof of component condition. Continue carefully, explain what a test proves and does not prove, and make small, verified changes.

## Project files and build

- Main firmware: `SdKfz222.cpp`
- Panzerlied note data: `PanzerliedMelody.h`
- PlatformIO configuration: `platformio.ini`
- Separate Erika example: `ERIKA_EXAMPLE/ERIKA_EXAMPLE.ino`; it is not part of the main firmware build.
- PlatformIO environment: `esp32dev` (main robot firmware)
- Commands from the project root:
  - Build main: `pio run -e esp32dev`
  - Upload main: `pio run -e esp32dev -t upload`
  - Serial monitor: 115200 baud; close it before uploading to release COM.
- The stray `the rte` prefix that was previously present before `[platformio]` in `platformio.ini` has been removed. The current file begins correctly with `[platformio]`.
- Rebuild `esp32dev` after source or configuration changes.

## Board, pins, and wiring

The user confirmed the ESP32 module label is **ESP-WROOM-32**. GPIO16/17 are normally usable on WROOM-32; do not assume they are flash/PSRAM pins. Avoid remapping to GPIO2/GPIO12 without a board-specific reason because they are boot-strapping pins.

Current firmware assignments in `SdKfz222.cpp`:

| Function | ESP32 GPIO |
|---|---:|
| L298N left IN1, IN2 | 26, 27 |
| L298N right IN3, IN4 | 16, 17 |
| Laser control output | 23 |
| Buzzer | 19 |
| VL53L1X SDA, SCL | 21, 22 |

L298N notes:

- The project now uses fixed-speed motor drive: install both ENA and ENB jumpers, and do not connect ESP32 GPIOs to either enable input. GPIO25/GPIO13 are not motor-enable pins in the current firmware.
- Motor command magnitude no longer controls speed. Nonzero commands outside the +/-8 deadband run at the driver's fixed enabled speed; command sign selects direction. Treat this as full-speed operation and test with wheels raised.
- Motor supply goes to the driver's motor supply input. ESP32, driver, and motor-supply grounds need a shared reference.
- Do not join L298N/converter 5 V to ESP32 5V/VIN while USB is connected unless the exact board explicitly supports that arrangement.
- The user reports an earlier suspected L298N short. Its power LED does not prove both H-bridge channels work.
- Power off and disconnect USB/battery before moving motor, driver, jumper, or converter wiring. Raise the chassis/wheels for tests. Stop for rapid heating, burning smell, smoke, or unstable power.

Sensor:

- VL53L1X has initialized successfully and reported distances, but logs sometimes include `NA` / timeouts.
- I2C uses SDA GPIO21, SCL GPIO22.
- Telemetry logs have shown both close readings (~90–100 mm) and clear readings (~2.7–3.0 m), depending on setup.

## Current firmware behavior and BLE protocol

BLE advertises as `SdKfz222`.

- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- RX/write: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- TX/notify: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- App used: nRF Connect for Mobile / Serial Bluetooth Terminal has connected and shown Nordic UART RX/TX. Values are text, not hex.
- TX format: `T,<millisSinceBoot>,<distanceMm or NA>,<sensor status>`; notifications are around every 250 ms. Some apps concatenate notifications without line breaks.

Accepted RX commands:

- `D,<left>,<right>`: left/right direction values -255..255. With fixed ENA/ENB jumpers, command magnitude is ignored (except the +/-8 stop deadband); any larger nonzero magnitude runs at fixed speed. The motor command timeout is 500 ms, so manual driving must be refreshed; send `S` on release.
- `S`: stop motors, turn laser off, disable autonomous mode, enter Hold.
- `A,1`: enable autonomous mode once; requires valid ToF sensor and persistent BLE connection.
- `A,0`: disable autonomous mode and stop drive.
- `MODE,RC`: select phone-controlled RC mode and stop current drive.
- `MODE,AUTO`: select autonomous mode; requires a valid ToF sensor.
- `J,<throttle>,<steering>`: RC drive joystick axes, each -100..100; positive throttle is forward and positive steering turns right. Firmware mixes axes as left=throttle+steering and right=throttle-steering, clamped to -100..100, then uses each sign for fixed-speed direction. With ENA/ENB jumpers installed, this is not proportional speed control. RC remains manual at the 400 mm Auto trigger threshold; the close-range hard stop still blocks movement.
- `J,...` or `D,...` overrides autonomous mode and returns to manual.
- RC mode is fully joystick controlled and does not automatically scan or steer around obstacles. At the 400 mm Auto trigger distance, joystick input still works; the separate close-range hard stop remains active at about 150 mm clearance.
- `M,1`: start looping Panzerlied melody; `M,0`: stop.
- MIDI upload from the webpage uses `MU,BEGIN,<count>`, sequential `MU,N,<index>,<frequencyHz>,<durationMs>` events, then `MU,END`; TX acknowledges `MU,OK,<count>` or `MU,ERR,<received>,<expected>`. Web parser supports Standard MIDI format 0/1 with PPQN timing, lets user choose a note track, converts it to monophonic by selecting the highest active pitch, and limits upload to 256 events (20–5000 ms/event, 20–4000 Hz; pitches above 4 kHz are capped). Upload replaces the built-in melody in RAM until reset and starts playback on success; upload again after reboot. Passive buzzer produces single notes, not chords or sampled audio.
- `L,1`: laser on with a maximum 5 s timed window, not extended by repeated L,1; `L,0`: off; `S` and BLE disconnect also turn it off. Retain the timeout; the control pin voltage does not determine the laser's optical safety.
- BLE disconnect stops motors and laser.
- Phone control webpage: `web/index.html`. It uses Android Chrome Web Bluetooth with the existing service/characteristics; host it from an HTTPS origin (or a browser-recognized secure origin). It provides RC/Auto mode buttons, a drive joystick, laser/music toggles, a STOP button, and ToF telemetry. The user confirmed rights to publish the reference image; `web/background.png` is used as the page background with a dark overlay. On connect the page selects RC and sends safe off commands.
- The webpage sends drive updates every 120 ms while the drive joystick is held; the firmware's 500 ms timeout stops manual drive if updates cease. MIDI upload writes events sequentially and waits for a firmware commit notification.

The receiver normalizes common Unicode dash/minus characters into ASCII `-`. For clarity, enter reverse commands with the ordinary hyphen:

- Left forward: `D,80,0`
- Left reverse: `D,-80,0`
- Right forward: `D,0,80`
- Right reverse: `D,0,-80`
- Both stopped: `D,0,0`

Sensor/route constants:

- sensor-to-tip offset 185 mm
- obstacle trigger clearance 400 mm from the robot tip; corresponding raw sensor threshold is 585 mm
- hard stop raw sensor threshold `<335 mm`; release when `>=375 mm`
- Auto requires a fresh VL53L1X reading. It drives forward until the forward-facing sensor detects an obstacle at or inside the 585 mm raw sensor trigger threshold (400 mm clearance from the tip). It stops, alarms, makes an approximate 90-degree in-place turn alternating right/left on successive attempts, waits for a new forward reading, then advances briefly if the way ahead is clear; otherwise it turns the other way and reassesses. Since the sensor is fixed forward, Auto cannot scan side routes before turning.
- Turn timing is open-loop (`AUTO_TURN_MS_PER_DEG` currently 7 ms/degree), not precision navigation; calibrate in a safe area. Fixed jumper speed affects the actual turn angle. The front sensor cannot see behind the robot.
- RC mode does not run obstacle-triggered scans; user joystick control remains active within the Auto trigger threshold. The hard collision stop still blocks drive commands at the configured close-range threshold.
- Obstacle alarm is 2200 Hz for 350 ms; melody note timing pauses during the alarm and resumes afterward.
- autonomous turn and forward movement both use fixed enabled motor speed; no PWM speed control is available in jumper mode
- Auto obstacle behavior is implemented in `updateAutoDrive()` and related functions.
- Autonomous mode commands both motor channels; steering cannot work properly with only one motor connected.

PWM/music:

- Main source includes `PanzerliedMelody.h`; Erika is in the excluded example only.
- User supplied a 56-note Panzerlied sequence; it is encoded in the header with frequency, note duration, and rest.
- Melody now loops until `M,0`.
- `tone()` uses `setToneChannel(Config::BUZZER_PWM_CHANNEL)` with channel 4 for buzzer PWM. User reported the Panzerlied now sounds right and buzzer no longer apparently causes voltage drop.

## Motor fault investigation to date

User reports:

- Left motor previously moved forward.
- Right motor did not move in manual tests despite wiring being reported correct.
- User then reported **neither left nor right motor moves in reverse; only forward works**.
- Latest relevant pasted log showed valid ASCII commands:
  - `D,-80,0,90`
  - `D,0,-80,90`
  - forward commands `D,80,0,90`, `D,0,80,90`
- In that log, sensor telemetry was generally about 2.7–2.9 m, so obstacle scan was not active during those particular command samples; occasional `NA` readings appeared.
- BLE disconnected later; disconnection intentionally stops the motors.
- Earlier log used a nonbreaking Unicode minus; that is why normalization was added. But a later log used ordinary ASCII `-`, so do not continue blaming the minus character without new evidence.
- User continuity-tested Dupont wires and says all cables work. Resistance across a motor's leads changes as wheel is rotated; this can be normal for brushed DC motors and does not prove a motor good.
- User has not yet given verified voltage measurements of IN/EN pins or confirmed a known-good motor powered through OUT3/OUT4.

Current motor-test findings:

- A replacement L298N's right channel ran a motor when the ENB jumper was installed, but did not run it with ESP32 PWM enable wiring moved among GPIO33, GPIO32, and GPIO13. This supports using jumper-fixed enable for now; it does not establish why the PWM enable path failed.
- User corrected the right motor wiring/polarity; both motors on the right channel had previously rotated in opposing physical directions because of polarity.
- Current firmware no longer attaches PWM to ENA/ENB. Install the ENA and ENB jumpers and disconnect any GPIO25/GPIO13 enable wires. Direction inputs remain GPIO26/27 and GPIO16/17.
- Test with wheels raised and one motor per channel first. Any nonzero command outside the deadband is fixed-speed; send `S` to stop. Confirm current loaded motor supply and driver temperature before prolonged operation.

## Collaboration instructions

- Do not tell the user a fault is proven from an LED, continuity beep, code build, or a BLE log alone. Explain what each result establishes.
- Avoid unsafe simultaneous USB and external 5 V/VIN supplies.
- The user has replaced the KY-008 with a different lower-power laser emitter, but its optical power/class is not verified. Keep the 5-second automatic laser-off limit, `S` shutdown, and disconnect shutdown; a 3.3 V control signal does not prove eye safety.
- Separate hardware faults from firmware/state issues. Check ToF threshold, Auto route state, BLE connection, and 500 ms manual-drive timeout before concluding the motor driver is defective.
- The user benefits from concise, stepwise instructions and asks for direct copyable BLE values.
- When reporting file links, use Markdown file links with absolute workspace paths.
