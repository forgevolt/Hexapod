# Software

Four sketch trees. Only the first one is the robot; the rest are bring-up tools that exist
because an 18-servo machine is painful to debug all at once.

| Path | What |
|---|---|
| `Hexapod/` | The firmware. This is the one you flash |
| `DynamixelUtilities/` | Servo setup and measurement, run before first assembly |
| `IKMath/` | The inverse kinematics on its own, no robot required |
| `HWTests/` | Per-peripheral smoke tests |

## `Hexapod/` - the firmware

| Files | Responsibility |
| --- | --- |
| `Hexapod.ino`, `Hexapod.*` | state machine, task setup, telemetry, and the top-level `step()` |
| `Leg.*`, `MathUtilities.*` | per-leg inverse and forward kinematics, vector maths |
| `GaitEngine.*`, `Gaits.*` | gait scheduling, and the park, stand, pose, level and walking gaits |
| `ServoBus.*` | the Dynamixel bus: sync read/write, joint limits, torque |
| `Receiver.*`, `RCProtocol.h` | the ESP-NOW link, control data in and telemetry out |
| `StatusDisplay.*` | the eyes and the fault display |
| `IndicatorLeds.*` | the LED strip effects |
| `PinMap.h` | every GPIO in the project |
| `build_opt.h` | re-enables `-Wsign-compare`, which the core suppresses |
| `partitions.csv` | 6 MB app + 9.8 MB LittleFS; overrides the Tools menu |

Audio and radio transport are not here - they are the `SoundEngine` and `ESPNowUtilities`
libraries, installed through the Library Manager. `RCProtocol.h` is a copy of the
transmitter's master; see the note in the top-level README.

## `DynamixelUtilities/` - servo setup

Each sketch talks to the servo bus and nothing else. Out of the box a Dynamixel is ID 1 at
57600 baud, so a new leg has to be set up one servo at a time, with only that servo on the
bus.

Run them roughly in this order when building a leg:

| Sketch | What it does |
|---|---|
| `SetBaudRate/` | Scans the bus, detects the current baud rate, sets it to the target. One servo at a time |
| `SetID/` | Same scan, then assigns the servo its ID. One servo at a time |
| `DefaultPosition/` | Drives every servo on the bus to 2048, the mechanical centre. Assemble the joint in this position |
| `PresentPosition/` | Prints the live positions of one leg's three servos, and the min/max seen. This is how the joint limits in `ServoBus` were measured |
| `Position1/`, `Position2/` | Two approaches to smooth point-to-point motion: way points with a blend threshold, versus streaming goal positions at 100-200 Hz. The firmware uses the second |

**Set the baud rate and the ID with one servo connected.** Both sketches scan and write to
whatever they find, so a whole bus will end up with the same ID.

## `IKMath/` - kinematics without the robot

`IKMath.ino` runs the same `Leg` and `MathUtilities` as the firmware, decoupled from the
state machine, the radio and the gaits. It made the maths debuggable in isolation, and it is
the sketch to reach for when changing the kinematics. `Leg.cpp` and `MathUtilities.cpp` here
are copies of the firmware's, for the same Arduino include reason as `RCProtocol.h` - keep
them in step.

## `HWTests/`

| Sketch | Needs | Checks |
|---|---|---|
| `StatusDisplayTest/` | an SH1106 OLED on I2C | Walks the whole `StatusDisplay` API - eyes, moods, blinking, the fault face - holding each step long enough to watch and logging what it is showing |
| `tone/` | the I2S amplifier | Plays C5-E5-G5. Enough to tell wiring from software |
