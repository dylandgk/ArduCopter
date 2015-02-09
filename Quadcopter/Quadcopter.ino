#include "PID.h"
#include <SoftwareSerial.h>// import the serial library

SoftwareSerial bluetooth (10, 11);
double myKp, myKi, myKd;
float deltaK = 0.01;

extern unsigned long lastTime, sampleTime;
extern double input, output, setPoint, errorSum, ITerm, lastInput, kp, ki, kd, outMin, outMax;
extern int controllerDirection, PIDMode;

void setup() {
  // Set PID controls.
  setPIDMode (ENABLED);
  myKp = .2;
  myKi = 1.2;
  myKd = .02;
  setTuningParameters (myKp, myKi, myKd);
  setOutputLimits (0, 255);
  setSampleTime (1000);
  
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
  delay(250);
}

void pollParametersToTune() {
  if (bluetooth.available()) {
    int bluetoothData = bluetooth.read();
    switch (bluetoothData) {
      case 'P':
        myKp += deltaK;
        break;
      case 'p':
        myKp -= deltaK;
        if (myKp < 0) {
          myKp = 0.0;
        }
        break;
      case 'I':
        myKi += deltaK;
        break;
      case 'i':
        myKi -= deltaK;
        if (myKi < 0) {
          myKi = 0.0;
        }
        break;
      case 'D':
        myKd += deltaK;
        break;
      case 'd':
        myKd -= deltaK;
        if (myKd < 0) {
          myKd = 0.0;
        }
        break;
    }
    setTuningParameters (myKp, myKi, myKd);
  }
}
