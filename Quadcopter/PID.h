#ifndef PID_H
#define PID_H

#include <Arduino.h>

#define ENABLED   1
#define DISABLED  0

#define DIRECT    0
#define REVERSE   1

void compute ();
void setTuningParameters (double, double, double);
void setSampleTime (unsigned long);
void setOutputLimits (double, double);
void initialize ();
void setPIDMode (int);
void setControllerDirection (int);

#endif    // PID_H
