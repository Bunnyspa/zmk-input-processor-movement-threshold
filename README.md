# zmk-input-processor-movement-threshold

A [ZMK](https://zmk.dev) input processor that gates pointing device output until enough movement has accumulated to confirm intentional input. Accidental contact — a finger resting on or brushing the device — is silently discarded. Once the accumulated movement crosses the threshold, normal output resumes until the device goes idle.

## Installation

Add this module to your `config/west.yml`:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    # --- copy from here ---
    - name: bunnyspa
      url-base: https://github.com/bunnyspa
    # --- to here ---
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    # --- copy from here ---
    - name: zmk-input-processor-movement-threshold
      remote: bunnyspa
      revision: main
    # --- to here ---
  self:
    path: config
```

## Prerequisites

Add to your `<keyboard>.conf`:

```ini
CONFIG_ZMK_POINTING=y
CONFIG_ZMK_INPUT_PROCESSOR_MOVEMENT_THRESHOLD=y
```

## Usage

Add the following to your `<keyboard>.overlay` or `<keyboard>.dtsi`:

```c
#include <input/processors/movement_threshold.dtsi>

/ {
    pointing_device_listener {
        compatible = "zmk,input-listener";
        device = <&pointing_device>;
        input-processors = <&movement_threshold 100 2000>;
    };
};
```

Replace `pointing_device_listener` and `&pointing_device` with your actual listener and device labels (e.g. `trackball_listener` and `&trackball`). If your board or shield already defines an input-listener node, reference it instead of creating a new one:

```c
&defined_listener_label {
    input-processors = <&movement_threshold 100 2000>;
};
```

`<&movement_threshold param1 param2>`

**`param1` — threshold (raw sensor counts)**

The values below are calculated and may not reflect real-world sensor behavior. Use them as a starting point and adjust to feel.

| Sensor CPI | 15° turn (34mm ball) | 5 mm (trackpad) |
|---|---|---|
| 400 | 70 | 79 |
| 600 | 105 | 118 |
| 800 | 140 | 157 |
| 1200 | 210 | 236 |
| 1600 | 280 | 315 |

**`param2` — idle-ms**

How long (in milliseconds) the device must be still before the gate resets. If the gate resets while you are still moving, increase it. If accidental touches are not filtered after lifting your hand, decrease it.

## Chaining with other processors

Place `movement_threshold` first. Processors after it only receive events once intentional movement is confirmed — accidental contact is silently dropped before it reaches them.

If you need a processor to activate on any touch regardless of the gate, place it before `movement_threshold`.

```c
input-processors = <
    // processors here see all events, including accidental contact
    &movement_threshold 100 2000
    // processors here only see intentional movement
>;
```

A common setup with `zip_temp_layer`, where layer 2 is a mouse click layer that activates only during intentional movement:

```c
input-processors = <
    &movement_threshold 100 2000
    &zip_temp_layer 2 2000
    &zip_xy_scaler 1 2
>;
```

## How it works

Every REL X/Y input event increments an internal accumulator by `|value|`. While the accumulator is below the threshold, all events are blocked. Once the threshold is crossed, the gate opens and events flow through normally. After the device is idle for `idle-ms` milliseconds, the accumulator resets and the gate closes again.
