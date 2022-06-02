#include <Servo.h>
Servo servomotors[12];
Servo head_servo;

void setup() {
  //put your setup code here, to run once:
//  servomotors[0].attach(A0); //s11
//    servomotors[1].attach(2); //s12
//  servomotors[2].attach(5); //s21
  servomotors[3].attach(4); //s22
  servomotors[4].attach(7); //s31
  servomotors[5].attach(6); //s32
  servomotors[6].attach(13); //s41
////  pinMode(13, OUTPUT);
////  digitalWrite(13, LOW);
  servomotors[7].attach(12); //s42
  servomotors[8].attach(11); //s51
  servomotors[9].attach(10); //s52
  servomotors[10].attach(9); //s61
  servomotors[11].attach(8); //s62
  Serial.begin(9600);
  while(!Serial){}
}
int pos;

void loop() {
  // put your main code here, to run repeatedly:
//  servomotors[0].write(90);
  if(Serial.available()>0){
    pos = Serial.parseInt();
    Serial.println(pos);
    if(pos!=0){
      for(int i=3; i<=11; i++){
        servomotors[i].write(pos);
      }
    }
  } 
}
