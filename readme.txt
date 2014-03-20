This is the code for the Polyideas Outdoor Solar Robot!
More info on this project can be found at http://polyideas.com

This project makes use of several pieces of hardware:

Adafruit Ultimate GPS breakout:http://www.adafruit.com/products/746
Pololu Motor Controller: http://www.pololu.com/product/2502
Sparkfun HMC6532: https://www.sparkfun.com/products/7915

Tested and works great with the Arduino UNO. Several problems exist with the Yun or Leonardo due to pin mappings and availability, but feel free to tackle if you like!  This code requires the Adafruit GPS library.  More details are in the comments in the sketch.


GPS code derived from samples written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above must be included in any redistribution

Place the Adafruit_GPS library folder your <arduinosketchfolder>/libraries/ folder. You may need to create the libraries sub