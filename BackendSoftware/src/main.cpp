#include "RecordData/DataReporter/DataReporter.h"
#include "RecordData/DataReporter/SimpleDataReporter.h"
#include "Math/Vector.h"

#ifndef NATIVE
#include <Arduino.h>
#include <Wire.h>
#include "Sensors/SAM-M10Q.h"
#endif




// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

#ifdef NATIVE
int main() {
  setup();
  loop();
  return 0;
}
#endif

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
