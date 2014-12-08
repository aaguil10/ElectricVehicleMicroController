#include <PID_v1.h>

int S1pin = 13;
int S2pin = 8;
int joyRefpin = A0;
int steerStatepin = A6;

double joyRef = 0;
double steerState = 0;
double controlVal = 128;

int steerVal;

PID steerPID(&steerState, &controlVal, &joyRef, 1, 0, 0, DIRECT);

void setup() {
  pinMode(S1pin, OUTPUT);
  pinMode(S2pin, OUTPUT);
  steerPID.SetMode(AUTOMATIC);
  Serial.begin(9600);
}

void loop() {
  joyRef = analogRead(joyRefpin);
  steerState = analogRead(steerStatepin);
  
  steerPID.Compute();
  steerVal = (int) controlVal;
  
  Serial.print(joyRef);
  Serial.print("--");
  Serial.print(steerState);
  Serial.print("--");
  Serial.println(controlVal);
  delay(100);
  analogWrite(S1pin, 255 - steerVal);
  analogWrite(S2pin, steerVal);
}

