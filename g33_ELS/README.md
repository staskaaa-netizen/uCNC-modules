# g33_ELS

`g33_ELS` is an experimental electronic leadscrew implementation for `G33`.

It keeps uCNC's normal parser handling for the command, target coordinates,
offsets, and modal state, but it does not run the thread pass through the
normal feed planner. After the next spindle index pulse, the spindle encoder
count becomes the master position source and the module emits step/dir pulses
directly to follow the commanded pitch.

Do not load this module together with the regular `g33` module. Both implement
the same `G33` command.

## Requirements

- `ENABLE_PARSER_MODULES`
- An encoder configured as `G33_ENCODER`
- A working spindle index hook for that encoder, for example:

```c
#define G33_ENCODER ENC0
#define G33_INDEX_PIN DIN5
```

On ESP32 the `esp32_pcnt_encoder` module can provide this index hook from a
PCNT index counter, without a GPIO ISR.

## Loading

Add the module to your module loader:

```c
LOAD_MODULE(g33_ELS);
```

Example:

```c
#define LOAD_MODULES_OVERRIDE() ({ \
    LOAD_MODULE(esp32_pcnt_encoder); \
    LOAD_MODULE(g33_ELS); \
})
```

## Command

```gcode
G33 Z-10 K1
```

`K` is pitch in machine units per spindle revolution. With millimetres active,
`K1` means 1 mm per spindle revolution.

For example, with:

```text
$102 = 200    ; Z steps/mm
$150 = 4000   ; encoder counts/rev for ENC0
```

`G33 Z-10 K1` produces:

```text
Z steps per rev = 1 * 200 = 200
EC counts per rev = 4000
Ratio = 200 / 4000 = 0.05 Z steps per encoder count
```

So every 20 encoder counts releases one Z step. A 10 mm pass takes 2000 Z
steps, or 40000 encoder counts.

## Behaviour

1. The parser resolves the target just like a normal motion command.
2. The module waits for the next spindle index pulse.
3. The encoder count at that index becomes the thread phase zero.
4. Live encoder delta is converted to desired motion steps.
5. Missing steps are emitted directly as step/dir pulses.
6. uCNC realtime position is updated as pulses are emitted.
7. When the target is reached, G33 exits and restores modal motion to `G1`.

If the spindle stops before the target, motion stops. If the spindle reverses
before the target, the axis moves backward along the same thread path. After the
target is reached, further spindle rotation does not move the axis.

## `$0` / max step rate

This module uses:

```c
g_settings.max_step_rate
```

as a safety cap for direct step pulse emission. In the current tested uCNC
configuration this is reported as `$0` and behaves as kHz. For example:

```text
$0 = 8.000
```

caps direct G33 output at about 8 kHz, or roughly 125 us between emitted step
pulses.

This cap only limits the direct ELS pulse generator. Normal planner moves and
jogs still use uCNC's normal planner/interpolator path.

If your uCNC build or branch documents `$0` differently, confirm the units for
`g_settings.max_step_rate` before using high spindle speeds.

## Options

```c
#define G33_ELS_STEP_PULSE_US 5
```

Sets the direct step pulse width in microseconds.

```c
#define G33_ELS_DIR_SETUP_US 5
```

Sets the delay after changing direction before emitting the next step pulse.
This matters when the spindle is reversed during a pass.

```c
#define G33_ELS_DEBUG
#define G33_ELS_DEBUG_INTERVAL_MS 500
```

Enables serial debug messages:

```text
[MSG:G33ELS start EC:... steps:... spr:... enc:... ratio:...]
[MSG:G33ELS EC:... d:... c:... want:... sent:...]
```

`want` is the encoder-derived commanded step position. `sent` is the number of
motion steps already emitted.

## Notes

This module intentionally does not use feed rate `F` for the thread pass. The
spindle encoder is the clock. `K`, target distance, step/mm, and encoder CPR
define the position relationship.

`F` is accepted on a `G33` line for sender/post compatibility, but it is ignored
and is not stored as the next modal `G1` feed. This prevents a command such as
`G33 Z-10 K1 F100` from changing the feed used by the following retract or
return moves.
