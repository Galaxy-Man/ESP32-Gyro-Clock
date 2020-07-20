/*******************************************************************

This clock not only displays the time but also interacts with our motion and look like they’re affectted by gravity.
This matrix clock displays some of LEDs as little grains of sand which are driven by a MPU6050 (Accelerometer + Gyro)
and NODEMCU-32S.
This program is referenced to the following sources:
https://learn.adafruit.com/matrix-led-sand
https://github.com/witnessmenow/Falling-Sand-Matri...
https://github.com/adafruit/Adafruit_PixelDust
https://github.com/porrey/ledmatrixide

Need to buy a GYRO: https://www.aliexpress.com/item/32401358878.html
Hook up to pins 21,22 on say a Wemos Lolin D32

 *******************************************************************/
//--------------------------------------------------------------------------
#include <Wire.h>
#include "MPU6050.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiConnect.h>   // faptastic library
#include <WiFiUdp.h>
#include <ESP32-RGB64x32MatrixPanel-I2S-DMA.h> // faptastic library
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

#include "imgBufferGFX.h"

#define N_GRAINS     300 // Number of grains of sand
#define WIDTH        64 // Display width in pixels
#define HEIGHT       32 // Display height in pixels
#define MAX_FPS      30 // Maximum redraw rate, frames/second

// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
  uint16_t pos;
  uint16_t colour;
} grain[N_GRAINS];

//#define SERIAL_DEBUG_OUTPUT 1

// NTP Server
#define NTP_OFFSET    60 * 60 * 1          // In seconds - Time offset
#define NTP_INTERVAL  60 * 1000           // In miliseconds
#define NTP_ADDRESS   "1.asia.pool.ntp.org"



// Class init
WiFiConnect wifiManager;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);
RGB64x32MatrixPanel_I2S_DMA display;


#define NUM_COLOURS 5
uint16_t myRED      = display.color565(255, 0, 0);
uint16_t myGREEN    = display.color565(0, 255, 0);
uint16_t myBLUE     = display.color565(0, 0, 255);
uint16_t myMAGENTA  = display.color565(255, 0, 255);
uint16_t myYELLOW   = display.color565(255, 255, 0);
uint16_t myCYAN     = display.color565(0, 255, 255);
uint16_t myBLACK    = display.color565(0, 0, 0);
uint16_t myWHITE    = display.color565(255, 255, 255);

uint16_t myCOLORS[6]={myRED,myGREEN,myCYAN,myMAGENTA,myYELLOW,myBLUE};

uint16_t myBallCOLORS[3]={myRED,myGREEN,myBLUE};

MPU6050 mpu;
uint32_t        prevTime   = 0;      // Used for frames-per-second throttle
uint16_t        backbuffer = 0,      // Index for double-buffered animation
                img[WIDTH * HEIGHT]; // Internal 'map' of pixels

ImgBufferGFX imgWrapper(img, WIDTH, HEIGHT);

float xOffset = -1350; 
float yOffset = -2590;

void pixelTask(void *param) {

  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_4G, MPU6050_ADDRESS, 21, 22)) // SDA - GPIO27, SCL - GPIO26
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }  
  

  Vector accelVector = mpu.readRawAccel();

  float xOffset = (accelVector.XAxis * -1) * -1;
  float yOffset = (accelVector.YAxis * -1) * -1;
  
  while (true) {
  uint32_t t;
  while (((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;

  Vector accelVector = mpu.readRawAccel();

  float accelX = (accelVector.XAxis * -1) + xOffset;
  float accelY = (accelVector.YAxis * -1) + yOffset;
  float accelZ = accelVector.ZAxis;

  int16_t ax = -accelX / 256,       // Transform accelerometer axes
          ay =  -accelY / 256,      // to grain coordinate space
          az = abs(accelZ) / 2048;  // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
  int16_t az2 = az * 2 + 1;         // Range of random motion to add back in

  // ...and apply 2D accel vector to grain velocities...
  int32_t v2; // Velocity squared
  float   v;  // Absolute velocity
  for (int i = 0; i < N_GRAINS; i++) {
    grain[i].vx += ax + random(az2); // A little randomness makes
    grain[i].vy += ay + random(az2); // tall stacks topple better!
    // Terminal velocity (in any direction) is 256 units -- equal to
    // 1 pixel -- which keeps moving grains from passing through each other
    // and other such mayhem.  Though it takes some extra math, velocity is
    // clipped as a 2D vector (not separately-limited X & Y) so that
    // diagonal movement isn't faster
    v2 = (int32_t)grain[i].vx * grain[i].vx + (int32_t)grain[i].vy * grain[i].vy;
    if (v2 > 65536) { // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(256.0 * (float)grain[i].vx / v); // Maintain heading
      grain[i].vy = (int)(256.0 * (float)grain[i].vy / v); // Limit magnitude
    }
  }

  // ...then update position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)

  uint16_t       i, bytes, oldidx, newidx, delta;
  int16_t        newx, newy;

  for (i = 0; i < N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if (newx > MAX_X) {              // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if (newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if (newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if (newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    oldidx = (grain[i].y / 256) * WIDTH + (grain[i].x / 256); // Prior pixel #
    newidx = (newy      / 256) * WIDTH + (newx      / 256); // New pixel #
    if ((oldidx != newidx) &&         // If grain is moving to a new pixel...
        img[newidx]) {                // but if that pixel is already occupied...
      delta = abs(newidx - oldidx);   // What direction when blocked?
      if (delta == 1) {               // 1 pixel left or right)
        newx         = grain[i].x;    // Cancel X motion
        grain[i].vx /= -2;            // and bounce X velocity (Y is OK)
        newidx       = oldidx;        // No pixel change
      } else if (delta == WIDTH) {    // 1 pixel up or down
        newy         = grain[i].y;    // Cancel Y motion
        grain[i].vy /= -2;            // and bounce Y velocity (X is OK)
        newidx       = oldidx;        // No pixel change
      } else { // Diagonal intersection is more tricky...
        // Try skidding along just one axis of motion if possible (start w/
        // faster axis).  Because we've already established that diagonal
        // (both-axis) motion is occurring, moving on either axis alone WILL
        // change the pixel index, no need to check that again.
        if ((abs(grain[i].vx) - abs(grain[i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
          if (!img[newidx]) {             // That pixel's free!  Take it!  But...
            newy         = grain[i].y;    // Cancel Y motion
            grain[i].vy /= -2;            // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
            if (!img[newidx]) {           // Pixel is free, take it, but first...
              newx         = grain[i].x;  // Cancel X motion
              grain[i].vx /= -2;          // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x;  // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;          // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;      // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
          if (!img[newidx]) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x;    // Cancel X motion
            grain[i].vy /= -2;            // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        }
      }
    }
    grain[i].x  = newx; // Update grain position
    grain[i].y  = newy;
    img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
    img[newidx] = 255;  // Set new spot
    grain[i].pos = newidx;
    //Serial.println(newidx);
  }
  }
}

// SETUP - RUNS ONCE AT PROGRAM START --------------------------------------

void setup(void) {

  delay(300);
  Serial.begin(115200);
    
  int i, j, bytes;


/*-------------------- START THE NETWORKING --------------------*/
  wifiManager.setDebug(true);
  wifiManager.setAPName("GyroClock");
  //wifiManager.setAPCallback(APconfigModeCallback);    // Call routine to show infos when in Config Mode
  
  if (!wifiManager.autoConnect()) { // try to connect to wifi
  
    Serial.print(F("Unable to connect to internet! Will start configuration portal."));
  
    wifiManager.startConfigurationPortal(AP_WAIT); 
  }

  timeClient.begin();

  display.begin();
  display.clearScreen();

  memset(img, 0, sizeof(img)); // Clear the img[] array

/*
  timeClient.update();
  unsigned long rawTime = timeClient.getEpochTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  String hhmm = hoursStr + ":" + minuteStr ;

  imgWrapper.setCursor(2, 25);
  imgWrapper.setFont(&FreeSansBold12pt7b);
  imgWrapper.setTextColor(myYELLOW);
  display.setTextSize(1); 
  imgWrapper.print(hhmm);   
  */
  
  for (i = 0; i < N_GRAINS; i++) { // For each sand grain...

    int imgIndex = 0;
    do {
      grain[i].x = random(WIDTH  * 256); // Assign random position within
      grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for (j = 0; (j < i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                              ((grain[i].y / 256) != (grain[j].y / 256))); j++);
      imgIndex = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    } while (img[imgIndex] != 0); // Keep retrying until a clear spot is found
    img[imgIndex] = 255; // Mark it
    grain[i].pos = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
    
    grain[i].colour = myBallCOLORS[i%3]; //display.color565(random(50,250), random(50,250), random(50,250)); //myCOLORS[i%NUM_COLOURS];
  }

  Serial.print("Creating task..");
  TaskHandle_t xHandle = NULL;
  xTaskCreatePinnedToCore(pixelTask, "PixelTask1", 5000, 0, (1 | portPRIVILEGE_BIT), &xHandle, 0);

  Serial.print("setup() running on core ");
  Serial.println(xPortGetCoreID());

  timeClient.update();

}

// MAIN LOOP - RUNS ONCE PER FRAME OF ANIMATION ----------------------------
unsigned long prev_minute = 0;

void loop() {

 // timeClient.update();
  unsigned long rawTime = timeClient.getEpochTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  String hhmm = hoursStr + ":" + minuteStr ;

  if (minutes != prev_minute)
  {
    imgWrapper.fillScreen(0);
  }
  imgWrapper.setCursor(2, 25);
  imgWrapper.setFont(&FreeSansBold12pt7b);
  imgWrapper.setTextColor(myYELLOW);
  display.setTextSize(1); 
  imgWrapper.print(hhmm);    
     
  
  for (int i = 0; i < N_GRAINS; i++) {
    int yPos = grain[i].pos / WIDTH;
    int xPos = grain[i].pos % WIDTH;
    display.drawPixel(xPos , yPos, grain[i].colour);
  }

  display.setCursor(2, 25);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(myYELLOW);
  display.setTextSize(1); 
  display.print(hhmm);

  // Need to print to both DMA buffers to stop flicker. Any ideas why?
  
  display.flipDMABuffer(); 
  display.clearScreen();
    for (int i = 0; i < N_GRAINS; i++) {
    int yPos = grain[i].pos / WIDTH;
    int xPos = grain[i].pos % WIDTH;
    display.drawPixel(xPos , yPos, grain[i].colour);
  }

  display.setCursor(2, 25);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(myYELLOW);
  display.setTextSize(1); 
  display.print(hhmm);
 /*
  display.setCursor(0, 32);
  display.setFont(&FreeSansBold9pt7b);
  display.setTextColor(myBLUE);
  //display.setTextSize(2);  
  display.print("CLOCK");   
*/
  display.showDMABuffer(); 

  prev_minute = minutes;

}
