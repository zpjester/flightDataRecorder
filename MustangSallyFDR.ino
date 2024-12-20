// Flight computer control software by Zach Jester.
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#define Prep 0
#define Prelaunch 1
#define Ascent 2
#define Descent 3
#define Landed 4

const bool print = false;

const int chipSelect = 10;
int state = Prep;
Sd2Card card;
SdVolume volume;
SdFile root;


String filename = "0.txt";
Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

float initialAlt = 300;
void setup() {
  if(print){Serial.begin(115200);}

  if (!bmp.begin()) {
	Serial.println("noBaro");
	while (1) {}
  }
  initialAlt = bmp.readAltitude();
  if(print){
    Serial.println(initialAlt);
  }
  delay(200);
  if(abs(bmp.readAltitude() - initialAlt) > 2){
    state = Ascent;
  }

  if (!mpu.begin()) {
    Serial.println("noAcc");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  if (state == Prep){
    digitalWrite(4, LOW);
    analogWrite(3, 0);
    delay(3000);
    analogWrite(3, 75);
    delay(1000);
  }
  // int retries = 3;
  // while(retries > 0){
  //   if (!card.init(SPI_HALF_SPEED, chipSelect)) {
  //     if(Serial){Serial.println("Card failed, or not present");}
  //     retries -= 1;
  //     delay(100);
  //     }
  //   else{
  //     break;
  //   }
  // }
  // if(!retries){
  //   analogWrite(3, 0);
  //   while(1){;}
  // }
  bool SDSuccess = SD.begin(chipSelect);
  if(print){
    Serial.print("SD: ");
    Serial.println(SDSuccess);
  }
  if(!SDSuccess){
    while(1){}
  }

  for(int i = 0; i < 4; i++){ // Found card!
      analogWrite(3, 0);
      delay(100);
      analogWrite(3, 255);
      delay(100);
    }

  char i = '0';
  
  //if(Serial){Serial.println(filename);}
  while(SD.exists(filename)){
    i++;
    filename[0] = i;
  }
  if (filename[0] > '9'){
    filename = "overflow.txt";
  }
  //if(Serial){Serial.println(filename);}
  File dataFile = SD.open(filename, FILE_WRITE);
  bool writeSuccess = dataFile.println("BEGIN LOG");
  Serial.print((String) writeSuccess);
  dataFile.close();

}

int logDur = 5000;
int strobePer = 1000;
int strobeDUTY = 500;
int strobeHIGH = 100;
int strobeLOW = 0;
unsigned long lastTime = 0;

unsigned long runs = 0;
unsigned long writes = 0;
float ASL = 300;
float AGL;
float xSP = 0.4;
float lastWriteTime = 0;
unsigned long liftoff = 0;
long flightTime = -1;
float vel = 0;
float lastAlt = 300;
float Kbaro = 0;
float Kvel = 0;
float predictedVel = 0;
float altEst = 300;
float measuredVel = 0;

int descending = 0;
float landed = 0;
void loop() {
  delay(20);
  runs++;
  unsigned long time = millis();
  float dt = (((float) time) - ((float)lastTime)) / 1000;
  lastTime = time;
  float altRead = bmp.readAltitude();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float xAccel = a.acceleration.x - xSP;
  float xRot = g.gyro.x;
  if (liftoff > 0){
    flightTime = time - liftoff;
  }
  
  float thrust;
  if(flightTime > 1000){
    thrust = 0;
  }
  else if(flightTime > 0 && flightTime < 50){
    float ftime = ((float) flightTime) / 1000;
    thrust = pow(2.0, (log10(ftime / .05) / log10(2.718) / 2.0)) * 86.0;
  }
  else if(flightTime < 700){
    thrust = 86.0;
  }
  else if(flightTime < 860){
    float ftime = ((float) flightTime) / 1000;
    thrust = max(pow(8, 8 *(0.7 - ftime)) * 93 - 7, 0);
  }
  else{
    thrust = 0;
  }

  //thrust = 0; // throw test. Remove before flight.

  float mass = 0.17; // kg
  float thrustAcc = thrust / mass; // m/s^2

  // Kalman goes here
  // I'm doing this 1-d in altitude. The accelrometer reports the 2nd derivative so needs some work to get it to an actual altitude measurement

  if(state == Ascent){ // Update predicted velocity based on estimated motor performance. This uses the average of the wet and dry mass of the rocket for simplicity so will be decent but not perfect.
    predictedVel = vel + (thrustAcc - 9.81) * dt;
  }
  
  lastAlt = ASL;  
    altEst = lastAlt + predictedVel * dt; // xn given n-1

  if(state == Ascent){// accelerometer only useful on the way up since it's swinging wildly during descent
    measuredVel = vel + xAccel * dt; // Update based on accelerometer data only
  }

  float baroDisagree = altRead - altEst;
  float baroSD = 5.0; // estimate
  float baroVariance = abs(baroDisagree / baroSD); // not how Gaussian works, but quick, light, and produces a decent response that stays low when near the correct value
  float velDisagree = measuredVel - predictedVel;
  float velSD = 3.0; // estimate
  float velVariance = abs(velDisagree / velSD); // not how Gaussian works, but quick, light, and produces a decent response that stays low when near the correct value

  float estVariance = .8 * (1 - (Kbaro + Kvel) / 2) * estVariance + .2 * 1; // "poisoning" the variance estimate to keep it from going to zero. I know the data has enough errors that this will never truly converge. 
  Kbaro = estVariance / (estVariance + baroVariance);
  Kvel = estVariance / (estVariance + velVariance);

  if(state != Ascent){
    Kvel = 0; // Ignore the accelerometer when known to be stationary or orientation is unstable
  }
  // if(altRead < initialAlt - 3){//Overpressure due to ejection charge
  //   Kbaro = 0; // Ignore the barometer during the overpressure event.
  // }

  float Ktotal = Kvel + Kbaro + .00001; // simple div / 0 solution

  ASL = altEst + pow(Kbaro, 2) / Ktotal * baroDisagree + pow(Kvel, 2) / Ktotal * velDisagree * dt;

  //ASL = altRead;
  vel = (ASL - lastAlt) / dt;
  // Reasoning behind this math here:
  // I have 2 sensors, each with their own kalman gain.
  // If I simply added both deviations multiplied by their kalman gain, then when both had decent readings the change in filtered altitude would be doubled.
  // Instead I also multiply each term by the fraction of the total gain belonging to it, which (for the barometer) is Kbaro /Ktotal
  // This leads to the kalman gain being squared, but it's being divided by itself (summed with the other gain) so it stays at the 1st power.

  AGL = altRead - initialAlt;

  if (flightTime > 300000){ // Simple timeout to force strobe
    state = Landed;
  }
  switch(state){
    case Prep:{ // Advance after a set time
      logDur = 5000;
      strobePer = 2000;
      strobeDUTY = 500;
      strobeHIGH = 100;
      strobeLOW = 0;
      if(time > 30000){
        state = Prelaunch;
      }
      float fruns = runs;
      initialAlt = initialAlt * (fruns / (fruns + 1.0)) + altRead / (fruns + 1.0);
      }
      break;
    case Prelaunch:{ // Advance when accelerometer or barometer sees major activity
      logDur = 500;
      strobePer = 1000;
      strobeDUTY = 500;
      strobeHIGH = 255;
      strobeLOW = 100;
      if((AGL > 10) || (xAccel > 50)){ // Launch detected!
        state = Ascent;
        liftoff = time;
        flightTime = 0;
      }
      }
      break;
    case Ascent:{ // Await apogee
      logDur = 50;
      strobePer = 150;
      strobeDUTY = 50;
      strobeHIGH = 255;
      strobeLOW = 50;
      if (vel < -1){
        descending++;
        if(descending > 5){ // Wait for 5 descent readings in a row
          state = Descent;
        }
      }
      else{
        descending = 0;
      }
      }
      break;
    case Descent:{ // Await touchdown
        logDur = 500;
        strobePer = 500;
        strobeDUTY = 250;
        strobeHIGH = 255;
        strobeLOW = 70;
        if (AGL < 10){
          landed = landed * .99 + .05;
        }
        else{
          landed *= .99;
        }
        if(landed > 1){ // Wait for 5 landed readings in a row
          state = Landed;
        }
      
      else{
        landed = 0;
      }
      }
      break;
    case Landed:{// Blink slowly and slow down data collection to save battery
        logDur = 30000;
        if(time < 100000){
          strobePer = 3000;
          strobeDUTY = 1000;
        }
        else{
          strobePer = 5000;
          strobeDUTY = 150;
        }
        strobeHIGH = 255;
        strobeLOW = 0;
      }
      break;
    default:{
      state = Ascent;
      }
      break;
  }
  
  


  if (time % strobePer < strobeDUTY){
    analogWrite(3,strobeHIGH);
  }
  else{analogWrite(3, strobeLOW);}
  
  if(time - lastWriteTime > logDur){
    lastWriteTime = time;
    String dataString = "";
    dataString += String(time);
    dataString += ",";
    dataString += String(flightTime);
    dataString += ",";
    dataString += String(state);
    dataString += ",";
    dataString += String(altRead);
    dataString += ",";
    dataString += String(ASL);
    dataString += ",";
    dataString += String(AGL);
    dataString += ",";
    dataString += String(vel);
    dataString += ",";
    dataString += String(xAccel);
    dataString += ",";
    dataString += String(xRot);
    dataString += ",";
    dataString += String(Kbaro);
    dataString += ",";
    dataString += String(Kvel);
    File dataFile = SD.open(filename, FILE_WRITE);
    if(print){
      Serial.println(dataString);
      //Serial.println((String) dataFile);
    }

    // if the file is available, write to it:
    if (dataFile) {      
      dataFile.println(dataString);
      dataFile.close();
      
    }
  }
  
}
