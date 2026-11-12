# Belt printer Benchy

A whole print, run through the simulated belt printer, with the result compared against the G-code.

`3DBenchy.gcode.gz` is 3DBenchy sliced by ideaMaker 4.0.3 for a 45 degree belt printer (131,000 lines, 293 layers).
Its coordinates are machine coordinates: Y is the nozzle's travel up the tilted gantry (height / sin 45), Z is the belt feed, X is across
the belt. That is the same meaning the simulator's belt model draws, so the file needs no conversion.

The model is [3DBenchy](https://www.3dbenchy.com/) (Creative Commons Attribution-NoDerivatives). Only the sliced G-code is included here.

## Build the simulator

In a Marlin tree (bugfix-2.1.x), with MarlinSimUI as the `MarlinSimUI` lib_dep (see the repository README):

```
use_example_configs Simulator
opt_enable SERIAL_PORT_2 BELTPRINTER COREXY
opt_disable BLTOUCH Z_SAFE_HOMING AUTO_BED_LEVELING_BILINEAR BABYSTEP_ZPROBE_OFFSET BABYSTEP_XY NOZZLE_CLEAN_FEATURE
opt_set Y_MIN_POS -5 Z_MAX_POS 1000000 X_BED_SIZE 220 Y_BED_SIZE 250
opt_set DEFAULT_AXIS_STEPS_PER_UNIT "{ 80, 80, 400, 137.6 }"
opt_set DEFAULT_MAX_FEEDRATE "{ 300, 300, 100, 25 }"
opt_set DEFAULT_MAX_ACCELERATION "{ 3000, 3000, 1000, 10000 }"
pio run -e simulator_macos_debug
```

- The E steps per mm must match the file's `M92 E137.6`: the simulator converts steps to millimeters with the compiled value.
- `Z_MAX_POS 1000000` (not the CR-30's 20,000,000) keeps the full `G28` in the start G-code finishing in the native sim.
- Use the binary under `.pio/build/<env>/debug/`.

## Run

```
test/belt_benchy/run.py --sim .pio/build/simulator_macos_debug/debug/MarlinSimulator
```

It starts the sim with `--machine belt` on agent port 8130 and serial port 8230 (`--agent-port`, `--serial-port` to change),
streams the file, and exits 0 when every check passes. A full run takes about ten minutes of wall time at the default
`--scale 100`. `--max-lines N` runs a quick prefix, `--attach` uses a sim you already started, and `--keep` leaves the sim running afterwards.

## What it checks

Ground truth is the simulated steppers (`GET /state/CoreXY Kinematic System`), not Marlin's position report.

- Every line is accepted: no `Unknown command`, `Error:` or halt.
- At each checkpoint (every 30 layers, and the end of the print) the nozzle is where the G-code last sent it: X and Y within 0.05 mm, and the
  belt feed (Z, measured from where `G92 Z0` ran) within 0.05 mm. The E steppers have delivered the filament the G-code asked for, after `M221`
  flow, within 1%. A planner that truncates the flow-scaled E steps (`M221 S94` on this file) delivers 13% too little and fails. A move that Marlin clamped, dropped or mis-stepped shows up here.
- The end G-code runs: it cools down, homes, and parks at Y50.
- The simulated time is printed next to the slicer's estimate (not judged: it includes heating and cooling).

Pictures of the finished print (front, side, iso, top) go to the output folder (`--out`) for a person to look at; they are not judged by the script.
