# ESP32-DMX-HUB75-controller
Repository to control a hub75 screen with DMX through a ESP32 controller. 

This Sketch uses 4 additional libraries;
- ESP DXX Library: https://github.com/someweisguy/esp_dmx
- ESP32-HUB75-MatrixPanel-I2S-DMA Library: https://github.com/tidbyt/ESP32-HUB75-MatrixPanel-I2S-DMA

Dependency: You will need to install Adafruit_GFX from the "Library > Manage Libraries" menu.

This repository was tested with a Lolin S2 Mini.

This sketch uses 2 HUB75 screens, each one having a 64x32 resolution. The sketch was tested and working with a Lolin S2 Mini, coupled with a Arduino MAX485 module. The sketch uses 9 channels for RGB control, brightness control for 4 sections, 1 for sidescrolling movement and 1 for strobing. 

More or less channels can be added as you want. You can use the Adafruit GFX library to draw text and other animations. Having a GIF repository stored locally on the ESP/external SD for animations could be future work. 

Usage

Include both the DMA library and esp_dmx library to your code
```markdown
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ESP32-VirtualMatrixPanel-I2S-DMA.h>
#include <esp_dmx.h>
```

Specify the pins for the connected HUB75 Matrice
```markdown
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
#define E_PIN -1 // required for 1/32 scan panels, like 64x64px. Any available pin would do, i.e. IO32

#define PANEL_RES_X 64      // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 32     // Number of pixels tall of each INDIVIDUAL panel module.
#define NUM_ROWS 2 // Number of rows of chained INDIVIDUAL PANELS
#define NUM_COLS 1 // Number of INDIVIDUAL PANELS per ROW
#define PANEL_CHAIN NUM_ROWS*NUM_COLS    // total number of panels chained one to another
#define VIRTUAL_MATRIX_CHAIN_TYPE CHAIN_BOTTOM_LEFT_UP

//MatrixPanel_I2S_DMA dma_display;
MatrixPanel_I2S_DMA *dma_display = nullptr;
VirtualMatrixPanel  *virtualDisp = nullptr;
```

Include settings for your DMX receiver and usage
```markdown
const int tx_pin = 12; //change
const int rx_pin = 8; //change
const int rts_pin = 11; //change
const dmx_port_t dmx_num = DMX_NUM_1; //use ESP port 1
dmx_config_t config = DMX_CONFIG_DEFAULT;
dmx_personality_t personalities[] = {
  {1, "Default Personality"}
};
int personality_count = 1;
bool dmxIsConnected = false;
unsigned long lastUpdate = millis();

const int size = 10;   // The size of this device's DMX footprint.
const int offset = 0;  // The start address of this device.
uint8_t data[size];
```


The setup includes the HUB75 pins, panel dimensions, resolution and layout, and initiates the DMA controller
```markdown
void setup() {
  Serial.begin(9600);

  dmx_driver_install(dmx_num, &config, personalities, personality_count);
  dmx_set_pin(dmx_num, tx_pin, rx_pin, rts_pin);

  HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};

  // Module configuration
  HUB75_I2S_CFG mxconfig(
    PANEL_RES_X,   // module width
    PANEL_RES_Y,   // module height
    PANEL_CHAIN,   // Chain length
    _pins    
  );

  // Display Setup
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(255); //0-255
  dma_display->clearScreen();
  virtualDisp = new VirtualMatrixPanel((*dma_display), NUM_ROWS, NUM_COLS, PANEL_RES_X, PANEL_RES_Y, VIRTUAL_MATRIX_CHAIN_TYPE);

}
```
The loop reads the incoming DMX data and redraws the results to the screen. Once a successful DMX connection is detected, a green block will appear in the corner. This is only for debugging. 
- Initial DMX channels are RGB (1,2,3), 4 subsections (4,5,6,7), Movement speed (8), and Strobe rate (9).

More or less channels can be added as you want. You can use the Adafruit GFX library to draw text and other animations. 
```markdown
void loop() {
  dmx_packet_t packet;
  if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK)) {
    unsigned long now = millis();
    if (!packet.err) {
      if (!dmxIsConnected) {
        Serial.println("DMX is connected!");
        dmxIsConnected = true;
        //add a green block in the right corner if the DMX was sucesfully connected to the ESP32. 
        virtualDisp->fillRect(0, 0, 2, 2, virtualDisp->color444(0, 15, 0));

      }

      int num_slots_read = dmx_read_offset(dmx_num, offset, data, size); //read the incoming DMX channels
      int strobespeed = data[8]; //Channel 8 contains the strobe value
      int movespeed = data[9]; //Channel 9 contains the movement value of the sectors
      int sec1 = data[4], sec2 = data[5], sec3 = data[6], sec4 = data[7]; //Channel 4, 5, 6 and 7 are 4 sections able to control their brightness value
      int red1 = (data[1] * sec1) / 255, green1 = (data[2] * sec1) / 255, blue1 = (data[3] * sec1) / 255; //Channel 1, 2 and 3 are the RGB values 
      int red2 = (data[1] * sec2) / 255, green2 = (data[2] * sec2) / 255, blue2 = (data[3] * sec2) / 255;
      int red3 = (data[1] * sec3) / 255, green3 = (data[2] * sec3) / 255, blue3 = (data[3] * sec3) / 255;
      int red4 = (data[1] * sec4) / 255, green4 = (data[2] * sec4) / 255, blue4 = (data[3] * sec4) / 255;

      static unsigned long movePreviousMillis = 0;
      unsigned long moveCurrentMillis = millis();
      static int position = 0;

      //If the movement speed is not 0, start scrolling the sections at the set speed. 
      if (movespeed != 0) {
        if (moveCurrentMillis - movePreviousMillis >= (256 - movespeed)) {
          movePreviousMillis = moveCurrentMillis;
          position = (position % 64) + 2; // Increment position within the screen width (64 pixels)

          // Calculate positions for each section based on the updated position
          int sec1_pos = position;
          int sec2_pos = (position + 16) % 64;
          int sec3_pos = (position + 32) % 64;
          int sec4_pos = (position + 48) % 64;

          //Draw the sections on the calculated position with said RGB value. 
          virtualDisp->fillRect(sec1_pos, 0, virtualDisp->width() / 4, virtualDisp->height(), virtualDisp->color565(red1, green1, blue1));
          virtualDisp->fillRect(sec2_pos, 0, virtualDisp->width() / 4, virtualDisp->height(), virtualDisp->color565(red2, green2, blue2));
          virtualDisp->fillRect(sec3_pos, 0, virtualDisp->width() / 4, virtualDisp->height(), virtualDisp->color565(red3, green3, blue3));
          virtualDisp->fillRect(sec4_pos, 0, virtualDisp->width() / 4, virtualDisp->height(), virtualDisp->color565(red4, green4, blue4));
        }
      } else {      
          //Draw the sections on with said RGB value. 
          virtualDisp->fillRect(0, 0, virtualDisp->width()/4, virtualDisp->height(), virtualDisp->color565(red1,green1,blue1));
          virtualDisp->fillRect(16, 0, virtualDisp->width()/4, virtualDisp->height(), virtualDisp->color565(red2,green2,blue2));
          virtualDisp->fillRect(32, 0, virtualDisp->width()/4, virtualDisp->height(), virtualDisp->color565(red3,green3,blue3));
          virtualDisp->fillRect(48, 0, virtualDisp->width()/4, virtualDisp->height(), virtualDisp->color565(red4,green4,blue4));
      }
      
      static unsigned long strobePreviousMillis = 0;
      static bool strobeState = false;
      unsigned long strobeCurrentMillis = millis();
      
      //If the strobe value is not 0, start strobing at the set rate. 
      if (strobespeed != 0) {
        if (strobeCurrentMillis - strobePreviousMillis >= (256 - strobespeed)) {
          strobePreviousMillis = strobeCurrentMillis;
          if (strobeState) {
            dma_display->setBrightness8(0); // Turn off
          } else {
            dma_display->setBrightness8(255); // Turn on
          }
          strobeState = !strobeState;
        }
      } else {
        dma_display->setBrightness8(255); // Turn off if strobe speed is 0
      }
      

    } else {
      Serial.println("A DMX error occurred.");
    }
  } else if (dmxIsConnected) {
    Serial.println("DMX was disconnected.");
    dmx_driver_delete(dmx_num);

    /* Stop the program. */
    while (true) yield();
  }
}
```

![image](https://github.com/user-attachments/assets/f2af549d-34bc-402d-9d68-84509e530b41)
