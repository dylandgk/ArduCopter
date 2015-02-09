#include "PID.h"
#include <SoftwareSerial.h>// import the serial library

SoftwareSerial bluetooth (10, 11);
double mykp, myki, mykd;

extern unsigned long lastTime, sampleTime;
extern double input, output, setPoint, errorSum, ITerm, lastInput, kp, ki, kd, outMin, outMax;
extern int controllerDirection, PIDMode;

void setup() {
  // Set PID controls.
  setPIDMode (ENABLED);
  mykp = .2;
  myki = 1.2;
  mykd = .02;
  setTuningParameters (mykp, myki, mykd);
  setOutputLimits (0, 255);
  setSampleTime (10);
  
  bluetooth.begin(9600);
  bluetooth.println("Bluetooth On: please press capital letter to increase or small letter to decrease parameter value");
  // Print on serial monitor.
  Serial.begin(9600);
  setPoint = 60;
  pinMode(9, OUTPUT);
  pinMode(A1, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  input = analogRead(A1);
  pollParametersToTune ();
  compute ();
  analogWrite (9, int (output));
  delay(500);
}

void pollParametersToTune() {
   if (bluetooth.available()) {
     int bluetoothData = bluetooth.read();
     if ( bluetoothData == 'P') {
       mykp += .05;
     }
     else if (bluetoothData == 'p') {// if number 0 pressed ....
       mykp -= .05;
       if (mykp < 0) mykp = 0;
     }
     else if ( bluetoothData == 'I') {
       myki += .05;
     }
     else if (bluetoothData == 'i') {// if number 0 pressed ....
       myki -= .05;
       if (myki < 0) myki = 0;
     }
     else if ( bluetoothData == 'D') {
       mykd += .05;
     }
     else if (bluetoothData == 'd') {// if number 0 pressed ....
       mykd -= .05;
       if (mykd < 0) mykd = 0;
     }
     setTuningParameters (mykp, myki, mykd);
   }
}
