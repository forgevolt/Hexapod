# PCB

The custom ESP32-S3 controller board: everything needed to have it made, and to change it.

![The hexapod](/docs/hexapod.jpg)

| Path | Contents |
|---|---|
| `easyeda/ProPrj_Hexapod_2026-08-06.epro2` | The editable EasyEDA Pro project - schematic and layout |
| `gerber/Gerber_HexapodPCB_R2_2026-08-06.zip` | Fabrication files for revision R2, zipped as a fab expects them |
| `schematic.pdf` | The schematic as a document, readable without EasyEDA |
| `bom.csv` | Bill of materials, with LCSC part numbers |
| `MPM3610Calculator.xlsx` | Working sheet for the MPM3610 buck regulator - feedback divider and inductor choice |

The board here is **revision R2**. The Gerber archive and the EasyEDA project carry the
revision and the export date in their filenames, so a later revision can sit beside this one
rather than silently replacing it.

## Ordering a board

Upload `gerber/Gerber_HexapodPCB_R2_2026-08-06.zip` as it is - a standard Gerber and drill
set. The BOM lists LCSC part numbers, which suits JLCPCB assembly, but nothing in the design
depends on a particular fab.

If you revise the layout, tag this repository when you order from it. A year later, "which
files made the board in my hand" then has an answer the tree alone cannot give.

## The BOM

`bom.csv` is exported from EasyEDA and is UTF-8. Some manufacturer names carry their Chinese
form alongside the Latin one - `SAMSUNG(三星)`, `KELIKING(凯丽金)` - which is how LCSC lists
them. If those render as mojibake, the reader is guessing the encoding rather than the file
being wrong.

## What is on the board

From the BOM, the parts worth naming:

| Part | Function |
|---|---|
| MPM3610GQV-Z | Step-down module, the +5V rail |
| LD39200PUR | 2 A LDO, the +3V3 rail |
| SMCJ15A | TVS on the battery input |
| 74LVC1G125 / 74LVC1G126 | The direction buffers that make the Dynamixel bus half-duplex |
| MAX98357AETE+T | I2S class-D amplifier |
| MPU-6050 | IMU |

`MPM3610Calculator.xlsx` is the working sheet behind the buck regulator's component values.
The schematic is the authority for what was actually built.
