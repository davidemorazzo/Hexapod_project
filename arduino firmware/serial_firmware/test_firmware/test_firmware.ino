#include <Servo.h>
Servo s1, s2; 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(30000);
  s1.attach(11);
  s2.attach(5);
  Serial.flush();
  while(!Serial){}
  Serial.println("Arduino Connected");
}

String str="30";
void loop() {
  // put your main code here, to run repeatedly:
   Serial.println(str.toInt()+1);
   delay(1);
   str = Serial.readStringUntil("\n");
   s1.write(str.toInt());
   s2.write(str.toInt());
}
