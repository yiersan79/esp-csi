# ESP Wi-Fi Sensing Demo

This directory contains a self-contained ESP-IDF demo used to exercise the
`esp_wifi_sensing` component in a realistic bring-up flow.

The example combines:

- Wi-Fi STA connection and sensing FSM startup
- Three monitored channels: the connected AP plus two fixed peer MAC addresses
- A status LED policy for quick visual feedback
- A lightweight serial protocol that can be consumed by the browser-based Web
  Serial monitor in `tools/web_serial_monitor.html`

The example follows the same public workflow recommended by the component:

1. Create the FSM with `esp_wifi_sensing_fsm_create()`.
2. Add the AP channel and two extra peer channels.
3. Register ACTIVE / INACTIVE callbacks.
4. Start the FSM with `esp_wifi_sensing_fsm_control(..., ESP_WIFI_SENSING_FSM_CTRL_START, NULL)`.
5. Optionally keep CSI traffic flowing with `esp_wifi_sensing_fsm_ping_router_start()`.

## Directory Layout

- `main/app_main.c`: example entry point and sensing FSM wiring
- `main/led_control.*`: LED backend and state mapping
- `main/web_serial_monitor.*`: line-oriented serial protocol for diagnostics and runtime tuning
- `tools/web_serial_monitor.html`: browser UI for charts, live diagnostics, and configuration

## Build and Run

Open the `wifi_sensing_demo` directory as an ESP-IDF project, then build and
flash it in the usual way:

```bash
idf.py set-target <chip>
idf.py flash monitor
```

The example uses `protocol_examples_common` for Wi-Fi provisioning, so configure
your Wi-Fi credentials before flashing if your local workflow requires it.

## Web Serial Monitor

When `CONFIG_ESP_WIFI_SENSING_WEB_SERIAL_ENABLE=y`, the firmware emits
line-delimited JSON messages prefixed with `HMS:` and accepts commands prefixed
with `HMSCMD `. This allows ordinary ESP logs and structured diagnostics to
share the same serial port.

The browser UI currently focuses on the minimum public diagnostics exported by
the component: `jitter_value`, `smooth_scaled`, `enter_level_scaled`,
`exit_level_scaled`, `state`, and `init_stage`.

The serial monitor also exposes the main runtime controls used by the demo:

- `HMSCMD HELLO`
- `HMSCMD START_STREAM`
- `HMSCMD STOP_STREAM`
- `HMSCMD FSM_START`
- `HMSCMD FSM_STOP`
- `HMSCMD RESET_BASELINE`
- `HMSCMD GET_CFG ALL`
- `HMSCMD GET_CFG <peer>`
- `HMSCMD GET_RUNTIME`
- `HMSCMD SET_AMPLITUDE_LOG on`
- `HMSCMD SET_AMPLITUDE_LOG off`
- `HMSCMD SET_CFG <peer> motion_sensitivity=<float> active_jitter_min=<float> active_filter_ms=<ms>`

Here `<peer>` can be either the demo name (`AP`, `MAC_1`, `MAC_2`) or the peer
MAC string.

To use the browser monitor:

1. Build and run the firmware.
2. Open `tools/web_serial_monitor.html` in a Chromium-based browser.
3. Connect to the board through Web Serial.
4. Inspect live channel diagnostics or update channel parameters from the page.

If the browser blocks `file://` access for Web Serial, serve the directory
locally:

```bash
cd tools
python3 -m http.server
```

Then open `http://127.0.0.1:8000/web_serial_monitor.html`.

## Demo Behavior

- The AP BSSID channel is treated as the primary channel for LED feedback.
- The LED blinks while Wi-Fi is disconnected.
- When the AP channel becomes ACTIVE, the LED turns fully on.
- When the AP channel returns to INACTIVE, the LED fades out.
- The two fixed peer channels remain visible in logs and the browser monitor as
  extra sensing references.

