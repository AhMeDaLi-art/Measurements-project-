# Measurements Project – Motor Monitoring Station

An ESP32-based station that **measures** the current and speed (RPM) of a DC motor, **controls** it remotely (on/off, speed, direction), and shows everything live in a **web dashboard** over MQTT.

```
 DC motor ── H-bridge ── ESP32 ── Wi-Fi ── MQTT broker ── MotorStation_App.html (browser)
              ▲            ▲
        20 A current   IR sensor
          sensor       (RPM)
```

## Features

- **Current measurement** with a 20 A Hall-effect sensor (100 mV/A), averaged and filtered, with a stored calibration gain.
- **Reads 0 mA when the motor is stopped** (zero is forced after the PWM has been off for 1 s, and the zero point is re-measured automatically while idle).
- **RPM measurement** with an IR sensor and a 4-arm fan, averaged over a sliding 30 s window.
- **Remote control** of the motor: on/off, duty cycle (0–100 %), direction.
- **Secure owner control**: commands are signed with HMAC-SHA256 and protected against replay. Only one owner at a time, and the motor stops if the owner session is lost.
- **Live web dashboard** (single HTML file, no installation) with status, controls, event log and **CSV recording**.

## Repository contents

| Path | Description |
|---|---|
| `Measurments_proj/` | ESP32 firmware (Arduino sketch `Measurments_proj.ino`) |
| `MotorStation_App.html` | Web dashboard – open it in any modern browser |
| `LICENSE` | MIT license |

## Hardware

- ESP32 development board
- 20 A current sensor (100 mV/A, 2.5 V at zero current, e.g. ACS712-20A)
- H-bridge motor driver (ENA / IN1 / IN2)
- DC motor
- IR sensor + 4-arm fan on the motor shaft (for RPM)
- Resistors for the current-sensor voltage divider: 14.5 kΩ and 22 kΩ (plus 1 kΩ in series with the ADC pin)

### Pin mapping

| Function | ESP32 pin |
|---|---|
| Current sensor (after divider) | GPIO 36 (ADC1) |
| IR sensor (RPM) | GPIO 39 |
| Motor PWM (H-bridge ENA) | GPIO 22 |
| Motor direction IN1 | GPIO 26 |
| Motor direction IN2 | GPIO 27 |

The sensor output (about 2.5 V at zero current when powered from 5 V) is scaled down by the 14.5 kΩ / 22 kΩ divider (ratio ≈ 0.603) so it stays within the ESP32 ADC range.

## Firmware setup

1. Install the **ESP32 board package** in the Arduino IDE (a recent version – the sketch uses `analogWrite`).
2. Install the **PubSubClient** library (by Nick O'Leary) from the Library Manager. `Preferences` and `mbedtls` are built into the ESP32 core.
3. Open `Measurments_proj/Measurments_proj.ino` and edit these lines:

   ```cpp
   const char* WIFI_SSID     = "your-wifi-name";
   const char* WIFI_PASSWORD = "your-wifi-password";
   const char* OWNER_KEY     = "a-long-random-secret";   // 16+ characters recommended
   ```

   If `OWNER_KEY` is left empty (`""`), the motor cannot be controlled remotely (monitoring only).
4. Select your ESP32 board, upload, and open the Serial Monitor at **115200 baud**.

> ⚠️ **Do not publish your Wi-Fi password or owner key.** Replace them with placeholders before making this repository public.

## Web dashboard

1. Open `MotorStation_App.html` in your browser.
2. Press **Connect** – the page connects to the MQTT broker and shows live values.
3. To control the motor, enter the same `OWNER_KEY` you set in the firmware to take control.
4. Use **Record** to log the readings and download them as a CSV file when you stop.

The dashboard marks data as **stale** if no message arrives for 12 s (the firmware publishes every 4 s).

### CSV columns

`timestamp, elapsed_s, state, direction, command_pwm_pct, applied_pwm_pct, speed_rpm, current_mA, vibration_mg, sensor_fault`

(`vibration_mg` is empty – there is no vibration sensor on this board.)

## MQTT

The firmware and the dashboard must use the same broker and topics. By default they use the public test broker `broker.emqx.io` (ESP32: TCP port 1883, browser: secure WebSocket port 8084).

| Topic | Direction | Content |
|---|---|---|
| `motorstation/demo-k7q2/telemetry` | ESP32 → app | JSON with current, RPM, duty, direction, state |
| `motorstation/demo-k7q2/status` | ESP32 → app | `online` / `offline` (last will) |
| `motorstation/demo-k7q2/cmd` | app → ESP32 | signed owner commands |

> The broker is public. Change `demo-k7q2` to your own unique string **in both** the sketch and `MotorStation_App.html` so nobody else uses your topics.

### Telemetry example

```json
{"rpm":1200,"rw":0,"cur":73,"duty":100,"mode":0,"dir":0,"flags":0,"target":0,
 "on":1,"cd":100,"own":1,"n":"1a2b3c4d","ack":1001,"res":0}
```

`cur` is the current in mA, `rpm` is `null` while the 30 s RPM window is still filling (`rw` = seconds left), and `flags` bit 0 means the current sensor is out of range (wiring problem).

### Owner commands

Format: `id|sid|op|val|nonce|hmac`, where `hmac` is HMAC-SHA256 of the first five fields using `OWNER_KEY`. The key itself is never sent, and the nonce changes after every accepted command so captured commands cannot be replayed.

Operations: `hb` (heartbeat / login), `rel` (release control), `on`, `off`, `duty` (0–100), `dir` (0 = forward, 1 = reverse).

Safety behaviour:

- Owner session expires after **12 s** without a command or heartbeat → the motor stops.
- After a direction change the motor pauses for **1.5 s** (it is never reversed while spinning).

## How the measurements work

### Current

- The ADC is sampled every 200 µs; 100 samples form a 20 ms window (trimmed mean – the 20 lowest and 20 highest samples are dropped).
- The 20 ms values are collected for **4 s**, trimmed again (20 % at each end), converted to current using 100 mV/A, and multiplied by the calibration gain.
- The zero point is measured at power-up (motor stopped) and re-tracked while the motor is idle.
- Readings below **10 mA** are reported as 0, and the reading is forced to 0 mA once the PWM has been off for 1 s.

### RPM

- One pulse per fan arm (4 per revolution), with 2 ms debounce.
- RPM is averaged over the last **30 s**; it shows as "warming up" until 30 s of pulses are available.
- No pulse for 0.5 s → RPM = 0.

## Calibrating the current reading

The default gain (`CURRENT_GAIN_DEFAULT = 1.81`) was set from a reference measurement of the motor. To fine-tune it with your own setup, use these Serial Monitor commands (line ending: **New Line**):

| Command | Action |
|---|---|
| `g73` | With the motor running at a known load, make the reading equal 73 mA (use your multimeter value). Wait ~8 s after starting the motor. The gain is saved in flash. |
| `gr` | Reset the gain to the default |
| `z` | Re-measure the zero point (motor must be stopped) |

At startup the Serial Monitor prints the gain currently in use.

## License

Released under the [MIT License](LICENSE).