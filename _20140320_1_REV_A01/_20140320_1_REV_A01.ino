// This program was written by Jay Doscher (jay@polyideas.com)
// More information on this project can be found at http://www.polyideas.com
// A big thank you to all the folks at Adafruit for the encouragement, forums support and of course - cool products! 
// This code makes use of the Adafruit 'parsing' GPS sample
// Software License Agreement (BSD License)
//Copyright (c) 2012, Adafruit Industries
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//1. Redistributions of source code must retain the above copyright
//notice, this list of conditions and the following disclaimer.
//2. Redistributions in binary form must reproduce the above copyright
//notice, this list of conditions and the following disclaimer in the
//documentation and/or other materials provided with the distribution.
//3. Neither the name of the copyright holders nor the
//names of its contributors may be used to endorse or promote products
//derived from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
//EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
//DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// This sketch is for the Arduino UNO only.  Due to pin requirements
// This sketch won't work with the Yun, Leonardo or Duemilanove

// Pololu VNH5019 Controller references
// We use this motor controller- it's overkill but works great and can handle 
 //really big motors, plus it can handle the 12V directly and drop down to the 
 // 9V the Arduino UNO needs.  
 // Beware! Un-jumper the voltage pin if you're trying to use this with an 
 // Arduino Yun- this shield will fry a Yun with 9V instead of the Yun's 5V.
 // http://www.pololu.com/product/2502
#include "DualVNH5019MotorShield.h"

// Adafruit library references
// Use the latest from here: https://github.com/adafruit/Adafruit-GPS-Library
#include <Adafruit_GPS.h>

// Common Arduino library references
#include <SoftwareSerial.h>
#include <math.h>
#include <Wire.h>

//HMC6532 Compass settings
// This code uses the Sparkfun HMC6532 from here: https://www.sparkfun.com/products/7915
const int sensorPin = 0;    // the pin that the potmeter is attached to
int sensorValue     = 0;    // store the value coming from the sensor
int HMC6352SlaveAddress = 0x42;
int HMC6352ReadAddress = 0x41; //"A" in hex, A command is: 
int headingInt;

// magdec is your magnetic declination, which is the delta between true and magnetic north
// Enter this as a positive or negative number
// You can determine your local magnetic declination here: http://www.ngdc.noaa.gov/geomag-web/#declination
int magdec = 5;

// Adafruit GPS work
// This wiring uses pins 5 and 3. Keep in mind these pins won't work on the Arduino Yun, and neither will pins 0,1 on the Yun.
SoftwareSerial mySerial(5,3);  // The GPS must be wired to these pins, not the default 0,1
Adafruit_GPS GPS(&mySerial);
// Turn this on to see the NMEA sentences from the GPS unit.  
// Most of the time you're not going to want this unless you suspect a problem with your GPS
#define GPSECHO  false
// Interrupt stuff for the GPS
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Turning on DEBUGPrint will show you the motor and logic messages
boolean DEBUGPrint  = true;


// Pololu VNH5019 Controller variables
DualVNH5019MotorShield md;
// The feedback from the position sensor on the motor needs to be wired to the 'sensorPin' referenced below.
int mappedValue;            // Mapped integer value for the linear actuator feedback sensor
int motorLinSpeed;          // linear actuator motor speed
int motorRotSpeed;          // rotation motor speed

// Start solar variables, don't mess with these.

float TrueSolarTime;
float EqOfTime;
float GeomMeanAnomSun;
float EccentEarthOrbit;
float GeomMeanLSunInt;
float GeomMeanLongSun;
float SunTrueLong;
float SunTrueAnom;
float SunRadVector;
float SunEqofCtr;
float SunAppLong;
float MeanObliqEcliptic;
float ObliqCorr;
float SunRtAscen;
float SunDeclin;
float VarY;
float HASunrise;
float HourAngle;
float SolarZenithAngle;
float CorrectedSolarElevation;
float ApproxAtmosRefraction;
float SolarElevation;
float SolarAzimuth;

// We need lots of floats for Julian Day calculations
float fltMins;
float fltHrs;
float fltDayDecimal;
long centuries;
long leaps;                        
long leapDays;
long yearDays;
long monthDays;
float JulianResult;
float JulianDay;
float JulianCentury;


// If you want accurate calculations, leave these as floats!
float lonmod; // Modifies GPS longitude from E/W to a negative or positive number
float latmod; // Modifies GPS latitude from N/S to a negative or positive number
float Latitude;
float Longitude;
float calcYear; // GPS gives us a two digit year, so we'll need to add 2000 to it later.
// End solar variables

void setup()  
{


  // GPS setup start
  Serial.begin(115200);                         //GPS needs this to be 115200 
  GPS.begin(9600); //Adafruit GPS is 9600 by default
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate
  useInterrupt(true);
  delay(3000);
  mySerial.println(PMTK_Q_RELEASE);

  // Heading setup start
  HMC6352SlaveAddress = HMC6352SlaveAddress >> 1; // I know 0x42 is less than 127, but this is still required
  Wire.begin();

  // Tilt Read
  mappedValue = map(sensorValue, 5, 804, 0, 92);

  // Motor setup start
  md.init(); // Motor control init

}
// More GPS stuff
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c;
}
void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
uint32_t timer = millis();

// Motor Fault code start
void stopIfFault()
{
  if (md.getM1Fault())
  {
    if (DEBUGPrint == true) {  
      Serial.println("Motor 1 Fault.");
      md.setM1Speed(0);
    }
    while(1);
  }
  if (md.getM2Fault())
  {
    if (DEBUGPrint == true) {  
      Serial.println("Motor 2 Fault.");
      md.setM2Speed(0);
    }
    while(1);
  }
}

// Motor feedback to serial port
void motorEcho()
{
  if (DEBUGPrint == true) {  
    Serial.print(",");
    Serial.print(mappedValue);
    Serial.print(",");
    Serial.print(CorrectedSolarElevation);
    Serial.print(",");
    Serial.print(SolarAzimuth);  
    Serial.print(",");
    Serial.println(headingInt);   
  }
}

// Read the heading from the HMC6532
void getHeading()
{
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.write(HMC6352ReadAddress);              // The "Get Data" command
  Wire.endTransmission();
  delay(6);
  Wire.requestFrom(HMC6352SlaveAddress, 2); //get the two data bytes, MSB and LSB
  byte MSB = Wire.read();
  byte LSB = Wire.read();
  float headingSum = (MSB << 8) + LSB; //(MSB / LSB sum)
  headingInt = ceil(headingSum / 10);
}
// Motor routine to raise the elevation of the solar panel
void raiseAngle()
{
  int sensorValue = analogRead(sensorPin);    // read the input pin
  mappedValue = map(sensorValue, 5, 804, 0, 92);
  motorLinSpeed--;
  //  String motorDirection = "Raise";
  Serial.print("Raise");
  motorEcho();

  if (motorLinSpeed <= -200) {
    motorLinSpeed = -200;
  }
  md.setM1Speed(motorLinSpeed);
}

// Motor routine to lower the elevation of the solar panel
void lowerAngle()
{
  int sensorValue = analogRead(sensorPin);    // read the input pin
  mappedValue = map(sensorValue, 5, 804, 0, 92);
  motorLinSpeed++;
  //  String motorDirection = "Lower";
  Serial.print("Lower");
  motorEcho();

  if (motorLinSpeed >= 200) {
    motorLinSpeed = 200;
  }
  md.setM1Speed(motorLinSpeed);
}

// Motor routine to rotate the motor through positive current
void rotatePos()
{
  delay(100);
  getHeading();
  Serial.print("Positive");
  motorEcho(); 

  motorRotSpeed++;
  if (motorRotSpeed >= 100) {
    motorRotSpeed = 100;
  }
  md.setM2Speed(motorRotSpeed);
}

// Motor routine to rotate the motor through negative current
void rotateNeg()
{
  delay(100);
  getHeading();
  Serial.print("Negative");
  motorEcho();

  motorRotSpeed--;
  if (motorRotSpeed <= -100) {
    motorRotSpeed = -100;
  }
  md.setM2Speed(motorRotSpeed);
}

void loop()                     // The main loop
{
  // Get the compass heading
  getHeading();

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) UDR0 = c;
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); 
    Serial.print(':');
    Serial.print(GPS.minute, DEC); 
    Serial.print(':');
    Serial.print(GPS.seconds, DEC); 
    Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.month, DEC); 
    Serial.print('/');
    Serial.print(GPS.day, DEC); 
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); 
    Serial.print((int)GPS.fix);
    Serial.print(" quality: "); 
    Serial.println((int)GPS.fixquality); 

    // Check for the latitude and longitude symbols so we can create the right math string for coordinates
    if (GPS.lat == 'N') {
      latmod = 1;
    } 
    else {
      latmod = -1;
    }

    if (GPS.lon == 'W') {
      lonmod = -1;
    } 
    else {
      lonmod = 1;
    }
  } 
  // Clean up the GPS strings to make them work with our solar calculations below
  int intYear = GPS.year + 2000; //year
  Longitude = (GPS.longitude * lonmod * .01); //enter longitude here
  Latitude = (GPS.latitude * latmod * .01); //enter latitude here 

  // If you have the DEBUGPRINT boolean value set to true, these variables will echo to the serial console.
  // If you're short on memory or space, you can remove these- but they're helpful if you're doing any GPS debugging.
  // This sketch requires the latest GPS library from Adafruit! This can be found here: https://github.com/adafruit/Adafruit-GPS-Library
  // If the value for time, latitude or longitude comes back as 0, you have a parsing or GPS problem.
  // Turning on the GPS echo can tell you if you're getting GPS NMEA sentences.
  if (DEBUGPrint == true) {  
    Serial.print("Heartbeat:");
    Serial.print(GPS.hour, DEC); 
    Serial.print(':');
    Serial.print(GPS.minute, DEC); 
    Serial.print(",");
    Serial.print(Latitude);
    Serial.print(",");
    Serial.print(Longitude);
    Serial.print(",");
  }

  //Solar Calculations
  // This set of calculations is accurate to within about 1 degree- it would be more accurate 
  // if the Arduino could handle more floating point precision
  // These calculations are pulled from the NOAA calculator here: http://www.esrl.noaa.gov/gmd/grad/solcalc/calcdetails.html
  // You can also find a handy calculator here, but beware of local time- my GPS calc assumes it's ALWAYS GMT.  http://www.sunearthtools.com/dp/tools/pos_sun.php
  fltMins = ((float)GPS.minute / 60)/24;
  fltHrs = (float)GPS.hour / 24;
  fltDayDecimal = fltHrs + fltMins;


// Julian Day calculated thanks to this Instructibles article: http://www.instructables.com/id/Making-a-Mayan-Tzolkin-Calendar/step5/Arduino-code/
  centuries = intYear/100;
  leaps = centuries/4;                            
  leapDays = 2 - centuries + leaps;         // note is negative!!
  yearDays = 365.25 * (intYear + 4716);     // days until 1 jan this year
  monthDays = 30.6001* (GPS.month + 1);    // days until 1st month
  JulianResult = leapDays + GPS.day + monthDays + yearDays -1524.5;
  JulianDay = (float)JulianResult + fltDayDecimal;
  JulianCentury = (JulianDay-2451545)/36525;
  
  // Really, don't mess with these
  GeomMeanLongSun =fmod(280.46646 + JulianCentury * (36000.76983 + JulianCentury * 0.0003032), 360);
  GeomMeanAnomSun = (357.52911 + JulianCentury * (35999.05029 - 0.0001537 * JulianCentury));
  EccentEarthOrbit = (0.016708634 - JulianCentury * (0.000042037 + 0.0000001267 * JulianCentury));
  SunEqofCtr = sin(radians(GeomMeanAnomSun))*(1.914602-JulianCentury*(0.004817+0.000014*JulianCentury))+sin(radians(2*GeomMeanAnomSun))*(0.019993-0.000101*JulianCentury)+sin(radians(3*GeomMeanAnomSun))*0.000289;
  SunTrueLong = GeomMeanLongSun + SunEqofCtr;
  SunTrueAnom = GeomMeanAnomSun + SunEqofCtr;
  SunRadVector = (1.000001018*(1-EccentEarthOrbit*EccentEarthOrbit))/(1+EccentEarthOrbit*cos(radians(SunTrueAnom)));
  SunAppLong = SunTrueLong-0.00569-0.00478*sin(radians(125.04-1934.136*JulianCentury));
  MeanObliqEcliptic = 23+(26+((21.448-JulianCentury*(46.815+JulianCentury*(0.00059-JulianCentury*0.001813))))/60)/60;
  ObliqCorr = MeanObliqEcliptic+0.00256*cos(radians(125.04-1934.136*JulianCentury));
  SunRtAscen = degrees(atan2(cos(radians(23.4352061926769))*sin(radians(13.8366518589196)),cos(radians(13.8366518589196))));
  SunDeclin = degrees(asin(sin(radians(ObliqCorr))*sin(radians(SunAppLong))));
  VarY =tan(radians(ObliqCorr/2))*tan(radians(ObliqCorr/2));
  EqOfTime= 4*degrees(VarY*sin(2*radians(GeomMeanLongSun))-2*EccentEarthOrbit*sin(radians(GeomMeanAnomSun))+4*EccentEarthOrbit*VarY*sin(radians(GeomMeanAnomSun))*cos(2*radians(GeomMeanLongSun))-0.5*VarY*VarY*sin(4*radians(GeomMeanLongSun))-1.25*EccentEarthOrbit*EccentEarthOrbit*sin(2*radians(GeomMeanAnomSun)));
  HASunrise = degrees(acos(cos(radians(90.833))/(cos(radians(Latitude))*cos(radians(SunDeclin)))-tan(radians(Latitude))*tan(radians(SunDeclin))));
  TrueSolarTime = fmod((fltDayDecimal*1440)+EqOfTime+(4*Longitude),1440);

  if (TrueSolarTime/4<0){ 
    HourAngle = (TrueSolarTime/4+180);
  }
  else
  {
    HourAngle = (TrueSolarTime/4-180);
  }
  SolarZenithAngle = degrees(acos(sin(radians(Latitude))*sin(radians(SunDeclin))+cos(radians(Latitude))*cos(radians(SunDeclin))*cos(radians(HourAngle))));
  SolarElevation = 90-SolarZenithAngle;

// Compensate for atmospheric refraction- this has the greatest influence on the angle of the sun when it's closer to the horizon
  if (SolarElevation>85) {
    ApproxAtmosRefraction =0;
  }
  else if (SolarElevation<85) {
    ApproxAtmosRefraction = (58.1/tan(radians(SolarElevation))-0.07/pow(tan(radians(SolarElevation)),3)+0.000086/pow(tan(radians(SolarElevation)),5))/3600;
  }
  else if (SolarElevation>-0.575) {
    ApproxAtmosRefraction = 1735+SolarElevation*(-518.2+SolarElevation*(103.4+SolarElevation*(-12.79+SolarElevation*0.711))),-20.772/tan(radians(SolarElevation))/3600;
  }

  CorrectedSolarElevation = round(SolarElevation+ApproxAtmosRefraction);

  if (HourAngle>0) {
    SolarAzimuth = (fmod(degrees(acos(((sin(radians(Latitude))*cos(radians(SolarZenithAngle)))-sin(radians(SunDeclin)))/(cos(radians(Latitude))*sin(radians(SolarZenithAngle)))))+180,360));
  }	
  else {
    SolarAzimuth = (fmod(540-degrees(acos(((sin(radians(Latitude))*cos(radians(SolarZenithAngle)))-sin(radians(SunDeclin)))/(cos(radians(Latitude))*sin(radians(SolarZenithAngle))))),360));
  }

  //Adjust the magnetic declination
  if (CorrectedSolarElevation > 0)
  {
    if (magdec < 0)
    {
      //      Serial.print(SolarAzimuth);
      //      Serial.print("->");
      if (SolarAzimuth + magdec >= 0)
      {
        SolarAzimuth = SolarAzimuth + magdec;
        //              Serial.println(SolarAzimuth);
      }
      else
      {
        SolarAzimuth = (SolarAzimuth + magdec + 360);
      }
    } 
    else if (magdec > 0)
    {
      //      Serial.print(SolarAzimuth);
      //      Serial.print("->");
      if (SolarAzimuth + magdec <= 360)
      {
        SolarAzimuth = SolarAzimuth + magdec;
        Serial.println(SolarAzimuth);
      }
      else
      {
        SolarAzimuth = (SolarAzimuth + magdec - 360);
        //                      Serial.println(SolarAzimuth);
      }  
    }
  }


  //Before any motor control, read the linear actuator position
  int sensorValue = analogRead(sensorPin);          //Read the input pin
  mappedValue = map(sensorValue, 45, 804, 0, 92);   //Map the readings of the analog input and calibrate to angles

  //Adjust the magnetic declination
  if (CorrectedSolarElevation > 0)
  {
    if (magdec < 0)
    {
      //      Serial.print(SolarAzimuth);
      //      Serial.print("->");
      if (SolarAzimuth + magdec >= 0)
      {
        SolarAzimuth = SolarAzimuth + magdec;
        //              Serial.println(SolarAzimuth);
      }
      else
      {
        SolarAzimuth = (SolarAzimuth + magdec + 360);
      }
    } 
    else if (magdec > 0)
    {
      //      Serial.print(SolarAzimuth);
      //      Serial.print("->");
      if (SolarAzimuth + magdec <= 360)
      {
        SolarAzimuth = SolarAzimuth + magdec;
        Serial.println(SolarAzimuth);
      }
      else
      {
        SolarAzimuth = (SolarAzimuth + magdec - 360);
        //                      Serial.println(SolarAzimuth);
      }  
    }
  }
  // If we have DEBUGPRINT set to true, the serial console will spit out your solar calculations.
  // CorrectedSolarElevation is the angle of the sun from the horizon
  // SolarAzimuth is the compass heading for the sun - remember that this is calculated from true north
  // You will need to factor in magnetic declination based on where you are in the world
  if (DEBUGPrint == true) {  				
    Serial.print(CorrectedSolarElevation, 0); //Print Elevation (Vertical) with no decimal places
    Serial.print(",");
    Serial.println(SolarAzimuth, 0); //Print Azimuth (Horizontal) with no decimal places
  }  

  // Check to see if the sun is below the horizon
  if (CorrectedSolarElevation < 0)
  {
    CorrectedSolarElevation = 0;
    if (DEBUGPrint == true) {
      Serial.println("Passive Mode");
    }
    // Lower the tilt head to zero
    while(mappedValue < 90){
      raiseAngle();
    }
    md.setM1Speed(0);
  }
  // In this section we're rotating the motor number 1 to raise or lower the panel.
  // Remember that for the panel to point at 0 degrees, the linear actuator needs to be fully contracted
  // Lower the angle of the panel

  if (mappedValue = round(CorrectedSolarElevation))
  {
    //Solar panel elevation doesn't need to move, so no need to do anything
  }
  else
  {
    if ((round(CorrectedSolarElevation) > 0) && (round(CorrectedSolarElevation) < 91) && (mappedValue > round(CorrectedSolarElevation))) 
    {
      while(mappedValue > CorrectedSolarElevation){
        lowerAngle();
      }
      md.setM1Speed(0);
    }
    //Raise the angle of the panel
    if ((round(CorrectedSolarElevation) > 0)  && (CorrectedSolarElevation < 91) && (mappedValue < CorrectedSolarElevation))
    {
      while(mappedValue < round(CorrectedSolarElevation)) {
        raiseAngle();
      }
      md.setM1Speed(0);
    }
  }

  getHeading();

  // In this section we're rotating the motor number 2 either positive or negative.
  // First make sure the angle of the sun is between 0 and 90 degrees
  //if (((headingInt - SolarAzimuth) < 2 ) ^ ((SolarAzimuth - headingInt) < 2))
  if (headingInt == round(SolarAzimuth))
  {
    //Heading is good, no need to move
    if (DEBUGPrint == true) {
      Serial.println("Motor: No Movement.");
    }
  }
  else
  {

    if (CorrectedSolarElevation > 0)
    {
      if (headingInt < SolarAzimuth)
      {
        while(((SolarAzimuth-headingInt) <2) ^ ((SolarAzimuth-headingInt) > -2))
        {
          rotatePos();
        }
        md.setM2Speed(0);
        if (DEBUGPrint == true) {
          Serial.println("Motor: Positive Stop.");
        }
      }
      else if (headingInt > SolarAzimuth)
      {
        while(((SolarAzimuth-headingInt) <2) ^ ((SolarAzimuth-headingInt) > -2))
        {
          rotateNeg();
        }
        md.setM2Speed(0);

        if (DEBUGPrint == true) {
          Serial.println("Motor: Negative Stop.");
        }
      }
    }
  }
  delay(120000); //Delay 2 minutes - you can set this to something faster for debugging, otherwise the sun usually doesn't move in the sky that fast :)
}















