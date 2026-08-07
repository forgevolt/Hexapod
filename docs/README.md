# Documentation

| Path | Contents |
|---|---|
| `hexapod.jpg` | The photograph the top-level README shows |
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
