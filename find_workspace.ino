#include <math.h>
#include <Haptics.h>

JointSpace q_in_cm, q;
WorkSpace p_in_cm, p;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  q.q1 = -0.00;

}

void loop() {
  // put your main code here, to run repeatedly:
  while (q.q1 >= -0.14){
    q.q2 = 0.00;
    while (q.q2 <= 0.14){
      Serial.print(q.q1,4);Serial.print(",");
      Serial.print(q.q2,4);Serial.print(",");
      p = Haptics.dirKin(q);
      p_in_cm = Haptics.getPInCm(p);
      Serial.print(p_in_cm.x,4);Serial.print(",");
      Serial.print(p_in_cm.y,4);Serial.println("");
      q.q2 = q.q2 + 0.001;
      }
      Serial.print(q.q1,4);Serial.print(",");
      Serial.print(q.q2,4);Serial.print(",");
      p = Haptics.dirKin(q);
      p_in_cm = Haptics.getPInCm(p);
      Serial.print(p_in_cm.x,4);Serial.print(",");
      Serial.print(p_in_cm.y,4);Serial.println("");
      q.q1 = q.q1 - 0.001;
    }
}
