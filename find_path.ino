#include <math.h>
#include <Haptics.h>

JointSpace q_in_cm, q;
WorkSpace p_in_cm, p;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  p.x = 0;
  p.y = 0.0536;
  while (p.y < 0.14){
    Serial.print(p.x,4);Serial.print(",");
    Serial.print(p.y,4);Serial.print(",");
    q = Haptics.invKinPosition(p);
    q_in_cm = Haptics.getQInCmForTest(q);
    Serial.print(q_in_cm.q1,4);Serial.print(",");
    Serial.print(q_in_cm.q2,4);Serial.println("");
    p.x = 0;
    p.y = p.y  + 0.001;
    }
  p.y = 0.14;
  while(p.y > 0.1125){
    Serial.print(p.x,4);Serial.print(",");
    Serial.print(p.y,4);Serial.print(",");
    q = Haptics.invKinPosition(p);
    q_in_cm = Haptics.getQInCmForTest(q);
    Serial.print(q_in_cm.q1,4);Serial.print(",");
    Serial.print(q_in_cm.q2,4);Serial.println("");
    p.x = 0;
    p.y = p.y  - 0.001;
    }
  p.x = 0;
  p.y = 0.1125;
  while(p.x > -0.035){
    Serial.print(p.x,4);Serial.print(",");
    Serial.print(p.y,4);Serial.print(",");
    q = Haptics.invKinPosition(p);
    q_in_cm = Haptics.getQInCmForTest(q);
    Serial.print(q_in_cm.q1,4);Serial.print(",");
    Serial.print(q_in_cm.q2,4);Serial.println("");
    p.x = p.x - 0.001;
    }
   p.x = -0.035;
   p.y = 0.1125;
  while(p.x < 0.035){
    Serial.print(p.x,4);Serial.print(",");
    Serial.print(p.y,4);Serial.print(",");
    q = Haptics.invKinPosition(p);
    q_in_cm = Haptics.getQInCmForTest(q);
    Serial.print(q_in_cm.q1,4);Serial.print(",");
    Serial.print(q_in_cm.q2,4);Serial.println("");
    p.x = p.x + 0.001;
    } 
   p.x = 0.035;
   p.y = 0.1125;
   delay(2000);
}
