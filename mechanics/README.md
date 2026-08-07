# Mechanics

The chassis and legs: Fusion source, print-ready meshes, and the dimensioned drawings.

| Path | Contents |
|---|---|
| `cad/Hexapod.f3z` | The Fusion source, as an archive containing the whole assembly |
| `stl/` | Print-ready meshes, 18 parts |
| `dimensions-top.png`, `dimensions-side.png` | Overall dimensions |

## Leg naming

The code names each leg's three links after an insect's, and the STLs follow:

| Link | Joint it rotates about | Moves |
|---|---|---|
| **Coxa** | vertical, at the body | the leg left and right, swinging it around the chassis |
| **Femur** | horizontal, at the coxa | the leg up and down |
| **Tibia** | horizontal, at the femur's far end | the foot in and out, setting how far the leg reaches |

`Leg.cpp` solves for the three angles in that order, and `PresentPosition/` prints them in
it too.

## The parts

| Group | Parts | Per robot |
|---|---|---|
| Centre body | `CenterBody-Main`, `-TopPlate`, `-CoverWithDiffusor`, `-CoverLedHolder`, `-ConnectorTopPlate`, `-ConnectorCover`, `-DisplayHolderLeft`, `-DisplayHolderRight` | 1 each |
| Cable entries | `CenterBody-Grommet-1Cable`, `-Grommet-2Cable`, `Femur-Groomet`, `CoxaTibia-CableFixture` | as needed |
| Leg | `Coxa-Upper`, `Coxa-Lower`, `Femur-Bracket1`, `Femur-Bracket2`, `Femur-ConnectingBracket`, `Tibia` | 6 each |

Six legs means the leg parts print six times over. Mirroring is not required - the legs are
identical and their difference is in the firmware's per-leg mounting angles.

## Opening the CAD

`Hexapod.f3z` is a Fusion archive rather than a single `.f3d` part - open it with
*File → Open* in Fusion, which unpacks the design and its components.

There is no STEP export here. If you want to modify the chassis without a Fusion licence, a
STEP export is the thing to ask for; the STLs are fine for printing but carry no editable
geometry.

## Printing

No slicer projects are included, so use your own profile. The parts that carry load are the
coxa and femur brackets and the centre body's top plate - a leg's own weight plus the
servo's stall torque goes through them, so treat infill and wall count there as structural
rather than cosmetic. The cover with the diffusor is the exception in the other direction:
it is meant to pass light from the LED strip.

## Servo and bracket geometry

The printed parts are dimensioned around two ROBOTIS components:

| Part | Role |
|---|---|
| **XC430-T240BB-T** | the servo, three per leg |
| **FR12-H101** | the horn bracket the next link bolts to |

Their drawings and STEP models are not reproduced here - they are ROBOTIS's documents, and
the current versions are on the
[ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xc430-t240/). Download them
from there if you need to check a fit or modify a bracket.
