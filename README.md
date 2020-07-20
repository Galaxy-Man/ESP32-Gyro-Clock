# INTERACTIVE-INTERNET-CLOCK

A blatant copy of tuenhidiy's amazing https://github.com/tuenhidiy/ESP32-INTERACTIVE-INTERNET-CLOCK.

Refactored so it works without issue with RGB64x32MatrixPanel_I2S_DMA (64x32 HUB75 Panel) and WiFiConnectLite libraries - both of which can be obtained from my github account. You will need to connect the GYRO to pins 21 (SCL) and 22 (SDA).

This clock not only displays the time but also interacts with our motion and look like they’re affected by gravity. This matrix clock displays some of LEDs as little grains of sand which are driven by a MPU6050 (Accelerometer + Gyro) and NODEMCU-32S.

Youtube link: https://www.youtube.com/watch?v=lYinMkkyPOc

Instructables: https://www.instructables.com/id/INTERACTIVE-INTERNET-CLOCK/


