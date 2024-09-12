# ESP32 Matrix Panel with DMX Control

This project demonstrates how to use an ESP32 to drive a HUB75 matrix display while receiving DMX signals for controlling various RGB sectors of the display. The project is implemented using the `ESP32-HUB75-MatrixPanel-I2S-DMA` and `esp_dmx` libraries. This sketch uses 2 HUB75 screens, each one having a 64x32 resolution. The sketch was tested and working with a Lolin S2 Mini, coupled with a Arduino MAX485 module. The sketch uses 9 channels for RGB control, brightness control for 4 sections, 1 for sidescrolling movement and 1 for strobing. 

More or less channels can be added as you want. You can use the Adafruit GFX library to draw text and other animations. Having a GIF repository stored locally on the ESP/external SD for animations could be future work. 

## Requirements

- **ESP32 Board (tested on Lolin S2 Mini)**
- **HUB75 LED Matrix Display** (e.g., 64x32 or 64x64)
- **ESP DMX Library** https://github.com/someweisguy/esp_dmx
- **ESP32-HUB75-MatrixPanel-I2S-DMA Library** https://github.com/tidbyt/ESP32-HUB75-MatrixPanel-I2S-DMA
- **DMX-512 Lighting Controller (or software)**



Dependency: You will need to install Adafruit_GFX from the "Library > Manage Libraries" menu.

## Wiring

Make sure to wire the ESP32 to the HUB75 LED Matrix as per the pin definitions in the code. Here's a summary:

```cpp
#define R1_PIN 39
#define B1_PIN 37
#define R2_PIN 35
#define B2_PIN 33
#define A_PIN 18
#define C_PIN 16
#define CLK_PIN 40
#define OE_PIN 38
#define G1_PIN 1
#define G2_PIN 2
#define B_PIN 3
#define D_PIN 5
#define LAT_PIN 7
#define E_PIN -1 // required for 1/32 scan panels
```

# DMX Setup
The ESP32 will be used as a DMX receiver, receiving data from a DMX controller. Set the appropriate pins for DMX communication:

```cpp
const int tx_pin = 12; // DMX transmit pin
const int rx_pin = 8;  // DMX receive pin
const int rts_pin = 11; // DMX RTS pin
const dmx_port_t dmx_num = DMX_NUM_1; // DMX port 1
```

# Library Imports:
We start by importing the necessary libraries for HUB75 matrix control and DMX communication.

```cpp
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ESP32-VirtualMatrixPanel-I2S-DMA.h>
#include <esp_dmx.h>
```

# Pin Definitions and Matrix Configuration:
Define the pins for the LED matrix and configure the matrix parameters (resolution, chain type, etc.).

```cpp
#define PANEL_RES_X 64      // Number of pixels wide of each panel
#define PANEL_RES_Y 32      // Number of pixels tall of each panel
#define NUM_ROWS 2          // Number of rows of chained panels
#define NUM_COLS 1          // Number of panels per row
#define PANEL_CHAIN NUM_ROWS*NUM_COLS    // Total number of panels in the chain
#define VIRTUAL_MATRIX_CHAIN_TYPE CHAIN_BOTTOM_LEFT_UP
```

# DMX Configuration:
The DMX personality and configuration are set to allow communication between the ESP32 and the DMX controller.

```cpp
dmx_personality_t personalities[] = {
  {1, "Default Personality"}
};
int personality_count = 1;
uint8_t data[10]; // DMX channel data
```

# Setup Function:
In the setup(), we initialize the serial communication, set up DMX pins, and initialize the matrix display.

```cpp
void setup() {
  Serial.begin(9600);
  
  // Install the DMX driver
  dmx_driver_install(dmx_num, &config, personalities, personality_count);
  dmx_set_pin(dmx_num, tx_pin, rx_pin, rts_pin);

  // Set up the matrix panel configuration
  HUB75_I2S_CFG::i2s_pins _pins = {R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN, _pins);
  
  // Initialize the matrix display
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(255);  // Set brightness to maximum (0-255)
  dma_display->clearScreen();
  
  virtualDisp = new VirtualMatrixPanel((*dma_display), NUM_ROWS, NUM_COLS, PANEL_RES_X, PANEL_RES_Y, VIRTUAL_MATRIX_CHAIN_TYPE);
}
```

# Loop Function:
In the loop(), we read DMX data and display it on the matrix. Sections of the display can change color based on DMX channel values.
The loop reads the incoming DMX data and redraws the results to the screen. Once a successful DMX connection is detected, a green block will appear in the corner. This is only for debugging. 
- Initial DMX channels are RGB (1,2,3), 4 subsections (4,5,6,7), Movement speed (8), and Strobe rate (9).

More or less channels can be added as you want. You can use the Adafruit GFX library to draw text and other animations. 
```cpp
void loop() {
  dmx_packet_t packet;
  
  if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK)) {
    if (!packet.err) {
      // Read DMX channel data and apply to sections of the display
      int num_slots_read = dmx_read_offset(dmx_num, offset, data, size);

      // Update sector colors based on DMX channels 1-7
      int sec1 = data[4], sec2 = data[5], sec3 = data[6], sec4 = data[7];
      int red1 = (data[1] * sec1) / 255, green1 = (data[2] * sec1) / 255, blue1 = (data[3] * sec1) / 255;
      int red2 = (data[1] * sec2) / 255, green2 = (data[2] * sec2) / 255, blue2 = (data[3] * sec2) / 255;
      
      // Display these colors on the matrix
      virtualDisp->fillRect(0, 0, 16, 32, virtualDisp->color565(red1, green1, blue1));
      virtualDisp->fillRect(16, 0, 16, 32, virtualDisp->color565(red2, green2, blue2));
    }
  }
}
```

# Scrolling and Strobing Effects:
The sectors of the matrix can move horizontally based on DMX channel 9, and the entire display can strobe based on DMX channel 8.

```cpp
// Move sections based on DMX channel 9 value (speed)
if (movespeed != 0) {
  position = (position % 64) + 2;
  virtualDisp->fillRect(position, 0, 16, 32, virtualDisp->color565(red1, green1, blue1));
}

// Strobe effect based on DMX channel 8
if (strobespeed != 0) {
  if (millis() - strobePreviousMillis >= (256 - strobespeed)) {
    strobeState = !strobeState;
    dma_display->setBrightness8(strobeState ? 255 : 0);
  }
}
```

![image](https://github.com/user-attachments/assets/f2af549d-34bc-402d-9d68-84509e530b41)
