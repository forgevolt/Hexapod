# Third-party licences

This firmware is MIT licensed (see LICENSE). It builds against the libraries below. None of
them are redistributed here - each is installed through the Arduino library manager and
keeps its own licence.

| Library           | Licence              | Used for                          |
| ----------------- | -------------------- | --------------------------------- |
| FastLED           | MIT                  | the indicator LED strip           |
| MPU6050_light     | MIT                  | the onboard IMU                   |
| Dynamixel2Arduino | Apache-2.0           | the servo bus                     |
| U8g2              | BSD 2-clause (code)  | the OLED display                  |
| Streaming         | LGPL-2.1-or-later    | the `Serial <<` logging           |
| SoundEngine       | MIT                  | I2S mixing and playback           |
| ESPNowUtilities   | MIT                  | the ESP-NOW link to the transmitter |

The last two are written for this project and published separately, under the same MIT
licence as this repository:
[SoundEngine](https://github.com/forgevolt/SoundEngine) and
[ESPNowUtilities](https://github.com/forgevolt/ESPNowUtilities).

## Streaming

`Streaming.h` by Mikal Hart is used for logging throughout the firmware. The LGPL does not
affect the licence of this project, but it asks that anyone who receives a binary can relink
it against a modified version of the library. Complete source is published here and no
pre-built binaries are distributed, so a recipient can modify Streaming and rebuild.

## U8g2 fonts

U8g2's code is BSD 2-clause, but its fonts are licensed separately. StatusDisplay renders
fault text with `u8g2_font_helvR08_tr`, derived from HELVR08.BDF:

    Copyright 1984-1989, 1994 Adobe Systems Incorporated.
    Copyright 1988, 1994 Digital Equipment Corporation.

    Permission to use, copy, modify, distribute and sell this software and its
    documentation for any purpose and without fee is hereby granted, provided that
    the above copyright notices appear in all copies. Adobe Systems and Digital
    Equipment Corporation make no representations about the suitability of this
    software for any purpose. It is provided "as is" without express or implied
    warranty.

The same notice is reproduced at the top of `StatusDisplay.h`.

## StatusDisplay

Written for this project, implemented from a description of the behaviour rather than from
the source of FluxGarage RoboEyes, which inspired it. The acknowledgement is in the header.
