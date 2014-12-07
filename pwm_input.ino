const int in = 2;
double sensorValue;
double reading;
int pin = 2;

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  sensorValue = takeReading();
  Serial.println(sensorValue);
}

double takeReading() {
  double h = pulseIn(pin,HIGH);
  double l = pulseIn(pin,LOW);
  return h / (h+l);
}
