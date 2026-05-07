# Changelog

## 2026-05-07

- Added experimental `g33_ELS` module.
- Implements `G33` as an electronic leadscrew pass driven directly by spindle
  encoder counts after the next index pulse.
- Emits step/dir pulses directly and updates uCNC realtime position.
- Supports reversing the spindle before the target is reached, moving the axis
  backward along the same thread path.
- Restores modal motion to `G1` after a completed `G33` pass.

