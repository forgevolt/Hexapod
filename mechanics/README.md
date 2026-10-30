# Mechanics

The chassis and legs: Fusion source, print-ready meshes, print profiles, and the dimensioned
drawings.

| Path | Contents |
|---|---|
| `cad/Hexapod.f3z` | The Fusion source, as an archive containing the whole assembly |
| `stl/` | Print-ready meshes, 18 parts |
| `print-profiles/` | Bambu Studio projects for the parts that are tricky to print because they need supports |
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

The parts that carry load are the coxa and femur brackets and the centre body's top plate - a
leg's own weight plus the servo's stall torque goes through them, so treat infill and wall count
there as structural rather than cosmetic. The cover with the diffusor is the exception in the
other direction: it is meant to pass light from the LED strip.

### Print settings

Taken from the profiles in `print-profiles/`, all sliced for a **Bambu Lab H2D with a 0.4 mm
nozzle**:

|---|---|
| Layer height | 0.16 mm (first layer 0.2 mm for the coxa, 0.1 mm for the tibia and the top plate) |
| Walls | 4 |
| Infill | 100% for the coxa and the tibia, 15% gyroid for the top plate |
| Supports | painted supports for the coxa, tree supports for the tibia and the top plate |
| Brim | auto |
| Material | ABS-GF; the tibia profile is set up for Polymaker HT-PLA-GF |

Materials that have given good results: **ABS-GF** and **Polymaker HT-PLA-GF**. ABS-GF is
preferred: it is a little more flexible while still stiff.

### Print orientation

| Part | Lies on the plate |
|---|---|
| `Coxa-Upper` | on the flat tab with the bearing hole, the U opening facing up |
| `Tibia` | on its side, the curve flat on the plate (about 38 mm high) |
| `CenterBody-TopPlate` | flat, on its underside |

## About `print-profiles/`

These are Bambu Studio projects: a full copy of the mesh plus the slicer state, specific to one
printer and one filament. They cover only the parts that are tricky to print because they need
supports, and are here as a reference for anyone with the same machine. On any other printer, the
STLs plus the tables above carry the same information in a form your own slicer can use.

**Use the profiles for print direction and settings only.** The meshes inside them may be older
than the STLs in `stl/`, so load the current STL in place of the mesh in the project before
printing.

## Servo and bracket geometry

The printed parts are dimensioned around two ROBOTIS components:

| Part | Role |
|---|---|
| **XC430-T240BB-T** | the servo, three per leg |
| **HN11-I101** | the idler bearing set for DYNAMIXEL XL 430 series |

Their drawings and STEP models are not reproduced here - they are ROBOTIS's documents, and
the current versions are on the
[ROBOTIS e-Manual](https://docs.robotis.com/docs/dxl/model_reference/x_series/xc_series/xc430-t240bb). Download them
from there if you need to check a fit or modify a bracket.
