//Includes
#include <math.h>
#include <Haptics.h>

Jacobian j;
WorkSpace force_p;
JointSpace force_q, pwm;

int solenoid1_enable = 8;           // Main Solenoid Enable
int solenoid1_input_1 = 22;         // Input 1 for Main Solenoid
int solenoid1_input_2 = 24;         // Input 2 for Main Solenoid
int solenoid2_enable = 9;           // Aux Solenoid Enable
int solenoid2_input_1 = 26;         // Input 1 for Aux Solenoid
int solenoid2_input_2 = 28;         // Input 2 for Aux Solenoid

void setup() {
  // put your setup code here, to run once:
  // Setup the Coils
  Serial.begin(9600);
  pinMode(SOL_L_EN, OUTPUT);
  pinMode(SOL_R_EN, OUTPUT);
  pinMode(SOL_L_1, OUTPUT);
  pinMode(SOL_L_2, OUTPUT);
  pinMode(SOL_R_1, OUTPUT);
  pinMode(SOL_R_2, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  Haptics.fullUp(&force_p, &force_q, &pwm);
  Serial.print("Up");
  delay(3000);
  Haptics.fullDown(&force_p, &force_q, &pwm);
  Serial.print("Down");
  delay(3000);
  Haptics.fullLeft(&force_p, &force_q, &pwm);
  Serial.print("Left");
  delay(3000);
  Haptics.fullRight(&force_p, &force_q, &pwm);
  Serial.print("Right");
  delay(3000);
  */
 
  Haptics.moveUp(190,4000);
  Haptics.stopMotors();
  delay(2000);
  Haptics.moveDown(190,2000);
  Haptics.stopMotors();
  delay(2000);
  Haptics.moveLeft(190,2000);
  Haptics.stopMotors();
  delay(2000);
  Haptics.moveRight(190,4500);
  Haptics.stopMotors();
  delay(2000);
 
  /*
  delay(3000);
  Haptics.moveRightMotorLeft(160,2000);
  Serial.print("R");
  Haptics.stopMotors();
  delay(3000);
  Haptics.moveLeftMotorLeft(160,1800);
  Serial.print("L");
  Haptics.stopMotors();
  delay(3000);
  Haptics.moveRightMotorRight(160,1800);
  Serial.print("R");
  Haptics.stopMotors();
  delay(3000);
  */
}

void driveMotor(int dutyCycleMain, int dutyCycleAux,
                bool Main1, bool Main2, bool Aux1, bool Aux2) {
  analogWrite(solenoid1_enable, dutyCycleMain);
  digitalWrite(solenoid1_input_1, Main1);
  digitalWrite(solenoid1_input_2, Main2);
  analogWrite(solenoid2_enable, dutyCycleAux);
  digitalWrite(solenoid2_input_1, Aux1);
  digitalWrite(solenoid2_input_2, Aux2);
}



