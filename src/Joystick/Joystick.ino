int LRpin;
int FBpin;
int LRanalogVal, FBanalogVal;
float joystick_LR, joystick_FB;
float LR_DEADZONE = 0.02;
float FB_DEADZONE = 0.08;
float LR_SCALE = 1/(1-0.02);
float FB_SCALE = 1/(1-0.08);

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
  
  //calculate the deadzones for the left right ranges of the joystick
  if (joystick_LR <= -LR_DEADZONE) joystick_LR = (joystick_LR + LR_DEADZONE) * LR_SCALE;
  else if (joystick_LR > LR_DEADZONE) joystick_LR = joystick_LR + LR_DEADZONE) * LR_SCALE;
  else joystick_LR = 0.0;
  
  //calculate the deadzones for the foward brake ranges of the joystick
  if (joystick_FB <= -FB_DEADZONE) joystick_FB = (joystick_FB + FB_DEADZONE) * FB_SCALE;
  else if (joystick_LR > FB_DEADZONE) joystick_FB = joystick_FB + FB_DEADZONE) * FB_SCALE;
  else joystick_LR = 0.0;
  
}
