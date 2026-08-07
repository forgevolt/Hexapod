// Single source of truth for every GPIO used in the project
#pragma once

// I2S (audio)
constexpr int cI2S_DOUT = 13;  // I2S Data out pin
constexpr int cI2S_BCLK = 14;  // Bit clock
constexpr int cI2S_LRC  = 21;  // Left/Right clock, also known as Frame clock or word select

// I2C (display + IMU)
constexpr int cI2C_SDA  = 8;
constexpr int cI2C_SCL  = 9;

// Neopixels
constexpr int cLedDataPin = 11; 

// Dynamixel
constexpr int cDXLDirPin = 16; 
constexpr int cDXL_RX    = 18; 
constexpr int cDXL_TX    = 17;

// Misc
constexpr int cOnOffSwitchPin = 48;
constexpr int cUSBSensePin    = 10;

