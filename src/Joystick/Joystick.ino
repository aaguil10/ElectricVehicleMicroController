int LRpin;
int FBpin;

int LRanalogVal, FBanalogVal;
float joystick_LR, joystick_FB;

//dead zones for the joystick values (to scale with 1)
float LR_DEADZONE = 0.02;
float FB_DEADZONE = 0.08;

//used to adjust left right values such that you have dead zones
//where -1 will still mean max left, and 1 will mean max right
float LR_SCALE = 1/(1-LR_DEADZONE);
float FB_SCALE = 1/(1-FB_DEADZONE);

float STEER_MAX_ANGLE = 22.0;
float STEER_MIN_ANGLE = 15.0;
float LEAN_MAX_ANGLE = 29.0;
float LEAN_MIN_ANGLE = 2.0;

//range of speed values where speed sensative steering/leaning activates
float SPEED_SENS_MAX = 30.0;
float SPEED_SENS_MIN = 4.0;   

//calculations for the amount of degrees you can change per unit of speed/lean
float SPEED_LIMIT_STEER_SLOPE = (STEER_MIN_ANGLE - STEER_MAX_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN);
float SPEED_LIMIT_LEAN_SLOPE = (LEAN_MAX_ANGLE - LEAN_MIN_ANGLE)/(SPEED_SENS_MAX - SPEED_SENS_MIN);

//limits calculated depending on speed
float steerAngleLimit = STEER_MAX_ANGLE;
float leanAngleLimit = LEAN_MIN_ANGLE;

float 


void setup() {
  Serial.begin(9600);
}

void loop() {
  LRanalogVal = analogRead(LRpin);
  //ComputePWMOutputs(); 
  FBanalogVal = analogRead(FBpin);
  //ComputePWMOutputs();
  Serial.print("Left-right analog value: ");
  Serial.println(LRanalogVal);
  Serial.print("Forward-brake analog value: ");
  Serial.println(FBanalogVal);
  
  //convert to [-1,1] value
  joystick_LR = (LRanalogVal / 512.0) - 1.0;
  joystick_FB = (FBanalogVal / 512.0) - 1.0;
  
  //set the deadzones for the left right ranges of the joystick
  if (joystick_LR <= -LR_DEADZONE) joystick_LR = (joystick_LR + LR_DEADZONE) * LR_SCALE;
  else if (joystick_LR > LR_DEADZONE) joystick_LR = joystick_LR + LR_DEADZONE) * LR_SCALE;
  else joystick_LR = 0.0;
  
  //set the deadzones for the foward brake ranges of the joystick
  if (joystick_FB <= -FB_DEADZONE) joystick_FB = (joystick_FB + FB_DEADZONE) * FB_SCALE;
  else if (joystick_LR > FB_DEADZONE) joystick_FB = joystick_FB + FB_DEADZONE) * FB_SCALE;
  else joystick_LR = 0.0;
 
  //you want it so that the faster you go, the less sensative your steer
  steerAngleLimit = SPEED_LIMIT_STEER_SLOPE * speedState + STEER_MAX_ANGLE; 
  if (steerAngleLimit > STEER_MAX_ANGLE) steerAngleLimit = STEER_MAX_ANGLE;
  if (steerAngleLimit < STEER_MIN_ANGLE) steerAngleLimit = STEER_MIN_ANGLE;
  //you want it so that the faster you go, the more you can lean
  leanAngleLimit = SPEED_LIMIT_LEAN_SLOPE * speedState + LEAN_MIN_ANGLE;
  if (leanAngleLimit > LEAN_MAX_ANGLE) leanAngleLimit = LEAN_MAX_ANGLE;
  if (leanAngleLimit < LEAN_MIN_ANGLE) leanAngleLimit = LEAN_MIN_ANGLE;

  //this is not in their code - but I'm wondering why it isn't.
  //it's suppose to account for bottom of the range where your
  //steering and leaning becomes speed sensative. 
  //(delete if unnecessary, or put into an if-else loop if appropriate). 
  if (speedState <= 4.0) {
    steerAngleLimit = STEER_MAX_ANGLE;
    leanAngleLimit = LEAN_MIN_ANGLE;
  } 
}  
