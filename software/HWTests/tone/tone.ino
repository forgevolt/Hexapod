// ---- I2S test --------------------------------------------------------------------------
// Plays a simple tone sequence to test I2S amplifier 
//

#include <Arduino.h>
#include <driver/i2s_std.h>

#define I2S_LRC  21
#define I2S_BCLK 14
#define I2S_DIN  13

const int amplitude = 500;
const int sampleRate = 8000;

// Startup sound sequence: C5, E5, G5
const int frequencies[] = {523, 659, 784}; 
const int durations[] = {150, 150, 400}; // Durations in milliseconds
const int totalTones = 3;

int currentTone = 0;
unsigned int halfWavelength;

int32_t sample = amplitude;
unsigned int count = 0;

// Handle for the native ESP-IDF I2S channel
i2s_chan_handle_t tx_handle;

unsigned long toneStartTime;
bool isPlaying = true; // Flag to track if the sequence is active

void setup() {
  Serial.begin(115200);

  unsigned long initTime = millis();
  while (!Serial && (millis() - initTime < 1000)) 
    delay(10);

  Serial.println("Direct ESP-IDF I2S startup sound");

  // 1. Allocate I2S TX channel on Port 0
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_handle, NULL);

  // 2. Configure standard I2S architecture (16-bit, Philips format)
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate),
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK,
      .ws   = (gpio_num_t)I2S_LRC,
      .dout = (gpio_num_t)I2S_DIN,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false }
    }
  };

  // 3. Initialize and enable the hardware channel
  i2s_channel_init_std_mode(tx_handle, &std_cfg);
  i2s_channel_enable(tx_handle);

  // Set up the first tone
  halfWavelength = sampleRate / frequencies[currentTone] / 2;
  toneStartTime = millis(); 
}

void loop() {
  // If the sequence is finished, do nothing
  if (!isPlaying) {
    delay(100);
    return;
  }

  // Check if it's time to move to the next tone
  if (millis() - toneStartTime >= durations[currentTone]) {
    currentTone++; // Move to next note

    if (currentTone >= totalTones) {
      // Sequence finished
      isPlaying = false;
      i2s_channel_disable(tx_handle); 
      Serial.println("Startup sequence complete. Tone stopped.");
      return; 
    } else {
      // Configure variables for the new tone
      halfWavelength = sampleRate / frequencies[currentTone] / 2;
      toneStartTime = millis();
      count = 0; // Reset count to align the new square wave phase cleanly
    }
  }

  // --- Normal tone generation code ---
  if (count % halfWavelength == 0) {
    sample = -1 * sample;
  }

  // Pack stereo data: Left channel sample, then Right channel sample
  int16_t stereo_buffer[2];
  stereo_buffer[0] = (int16_t)sample; // Left
  stereo_buffer[1] = (int16_t)sample; // Right

  // Write directly via DMA
  size_t bytes_written;
  i2s_channel_write(tx_handle, stereo_buffer, sizeof(stereo_buffer), &bytes_written, portMAX_DELAY);

  count++;
}