
unsigned long lastTime;
unsigned long sampleTime = 1000;      // 1 second.

double input, output, setPoint;
double errorSum, ITerm, lastInput;
double kp, ki, kd;

#define ENABLED   1
#define DISABLED  0

bool PIDMode = DISABLED;

double outMin, outMax;

void compute () {
  if (!PIDMode) {
    return;
  }
  unsigned long currentTime = millis ();
  unsigned int timeDifference = currentTime - lastTime;
  
  if (timeDifference >= sampleTime) {
    // Compute all the working error variables.
    double error = setPoint - input;
    ITerm += ki * error;
    
    // Clamp the integral term.
    if (ITerm > outMax) {
      ITerm = outMax;
    }
    else if (ITerm < outMin) {
      ITerm = outMin;
    }
    
    double inputDifference = input - lastInput;
    
    // Compute PID output.
    output = kp * error + ITerm - kd * inputDifference;
    
    // Clamp the output.
    if (output > outMax) {
      output = outMax;
    }
    else if (output < outMin) {
      output = outMin;
    }
    
    // Store some variables in the memory for the next call of this function.
    lastInput = input;
    lastTime = currentTime;
  }
}

void setTuningParameters (double Kp, double Ki, double Kd) {
  double sampleTimeInSec = ((double) sampleTime) / 1000;
  kp = Kp;
  ki = Ki * sampleTimeInSec;
  kd = Kd / sampleTimeInSec;
}

void setSampleTime (unsigned long newSampleTime) {
  if (newSampleTime > 0) {
    sampleTime = newSampleTime;
    double sampleTimeRatio = newSampleTime / sampleTime;
    ki = ki * sampleTimeRatio;
    kd = kd / sampleTimeRatio;
  }
}

void setOutputLimits (double minVal, double maxVal) {
  if (maxVal < minVal) {
    return;
  }
  
  if (output > maxVal) {
    output = maxVal;
  }
  else if (output < minVal) {
    output = minVal;
  }
  
  if (ITerm > maxVal) {
    ITerm = maxVal;
  }
  else if (ITerm < minVal) {
    ITerm = minVal;
  }

void setMode (int mode){ {
  newPIDMode = (mode == ENABLED);
  if (newPIDMode && !(PIDMode)) {
    initialize ();
  }
  PIDMode = newPIDMode;
}

void initialize () {
  lastInput = input;
  ITerm = output;
  if (ITerm > outMax) {
    ITerm = outMax;
  }
  else if (ITerm < outMin) {
    ITerm = outMin;
  }

void setup () {
  
