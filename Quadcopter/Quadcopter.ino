void setup() {
  // Set PID controls.
  setPIDMode (ENABLED);
  setTuningParameters (.2, 1.2, .02);
  setOutputLimits (0, 255);
  setSampleTime (10);
  setControllerDirection (REVERSE);
  
  // Print on serial monitor.
  Serial.begin(9600);
  setPoint = 60;
  pinMode(9, OUTPUT);
  pinMode(A1, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  input = analogRead(A1);
  compute ();
  analogWrite (9, int (output));
  delay(5);
}
