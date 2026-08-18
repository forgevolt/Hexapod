# Documentation

| Path | Contents |
|---|---|
| [`QUICKSTART.md`](QUICKSTART.md) | Quick guide - one page, for anyone handed the robot |
| [`MANUAL.md`](MANUAL.md) | Operating manual - servo setup, every control, troubleshooting |
| `transmitter-controls.jpg` | The annotated transmitter photo the quick guide shows |
| `hexapod.jpg` | The photograph the top-level README shows |
| `PCB.jpg` | The assembled controller board, shown in `hardware/README.md` |
| `test-harness1.jpg`, `test-harness2.jpg` | The bring-up harness, shown in `hardware/README.md` |
| `Hexapod-IKMath.pdf` | The inverse-kinematics derivation for this geometry |

## `Hexapod-IKMath.pdf`

The maths behind `software/Hexapod/Leg.cpp`, worked through for this robot's link lengths
and mounting angles: from a target foot position in body coordinates, to coxa, femur and
tibia angles.

`Leg.cpp` follows the approach in Jakob Leander's
[tutorial](https://youtu.be/WAsMAeKDc4U); this document carries it through for the
dimensions in `mechanics/`, which is what makes the constants in the code readable rather
than magic.

`software/IKMath/` is the runnable companion - the same `Leg` and `MathUtilities`, with no
robot attached.
