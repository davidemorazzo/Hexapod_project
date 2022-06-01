#include <Servo.h>

Servo servomotors[12];
Servo head_servo;
void setup() {
  servomotors[0].attach(A0); //s11
  servomotors[1].attach(2); //s12
  servomotors[2].attach(5); //s21
  servomotors[3].attach(4); //s22
  servomotors[4].attach(7); //s31
  servomotors[5].attach(6); //s32
  servomotors[6].attach(13); //s41
  servomotors[7].attach(12); //s42
  servomotors[8].attach(11); //s51
  servomotors[9].attach(10); //s52
  servomotors[10].attach(9); //s61
  servomotors[11].attach(8); //s62
  Serial.begin(57600);
  // wait serial to connect
  Serial.flush();
  while(!Serial){}
  Serial.println("Arduino Connected");
}

char command_id, b[10];
char bytes_read[1];
int current_angle;

void loop() {
  //Serial.println(int(b));
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    Serial.readBytesUntil('\n', b, 10);
    current_angle = toInt(b);
    Serial.println(b);
    Serial.println("One println");
  }
//  if(Serial.available() > 0){
//    command_id = Serial.read();
//    Serial.println("Received:");  
//    Serial.println(command_id, BIN);
//    if(int(command_id)==1){
//      Serial.println("Command is 1!");
//    }
//    switch(command_id){
//      case 1:
//        Serial.println("Entered case 1");
//        delay(10000);
//        Serial.flush();
//        if(Serial.available()>0){ 
//          for(uint8_t i=0; i<=2; i++){
//            current_angle = Serial.parseInt();
//            Serial.println(current_angle, DEC);
//            //current_angle = (uint8_t) bytes_read[0];
//            //servomotors[i].write((int)current_angle);
//            //if(i%2==1){
//            //  i+=2;
//            //}
//          }
//        }
//      default:
//        Serial.println("Default case");       
    //}
//  }
}
