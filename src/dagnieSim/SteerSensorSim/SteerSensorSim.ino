int ledPin = 9;    // LED connected to digital pin 9
int pwm_1 = 4;
int pwm_2 = 7;

int pwm_out = 3;

class SteerSensor{
   public:
      int angle;   // 0 - 360 degrees
      
      SteerSensor(){
        angle = 0; 
      }
      void move_right(){
        if(angle <= 360){
          angle++;
        }else{
          angle = 0;
        } 
      }
      void move_left(){
        if(angle >= 0){
          angle--;
        }else{
          angle = 360;
        } 
      }
};

SteerSensor* myS = new SteerSensor;

void simulate_steer(int a, int b){
  if(a > b){
    (*myS).move_right();
  }  
  if(b>a){
   (*myS).move_left(); 
  }
}

double takeReading(int pin) {
  double h = pulseIn(pin, HIGH);
  Serial.println(h);
  double l = pulseIn(pin, LOW);
  Serial.println(1);
  return h / (h+l);
}

void setup()  { 
  Serial.begin(9600);
  pinMode(pwm_1, INPUT);
  pinMode(pwm_2, INPUT);
  //SteerSensor* myS = new SteerSensor;
} 

void loop()  { 
  int val_1 = digitalRead(pwm_1);
  int val_2 = digitalRead(pwm_2);
  /*Serial.print("val_1: ");
  Serial.println(val_1);
    Serial.print("val_2: ");
  Serial.println(val_2);*/

  //Reads steer Valves
  double p1 = takeReading(pwm_1);
  double p2 = takeReading(pwm_2);
  Serial.print("p1: ");
  Serial.println(p1);
  Serial.print("p2: ");
  Serial.println(p2);
  
  //steer sensor output
  simulate_steer(p1, p2);
  int output = map((*myS).angle, 0, 360, 0, 255);
  analogWrite(pwm_out, output); 
  delay(1);
}
