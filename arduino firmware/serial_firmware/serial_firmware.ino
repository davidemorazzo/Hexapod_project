#include <Servo.h>

// Servomotors declaration
Servo servomotors[12];
Servo head_servo;

void setup() {
  // Attach servomotors to the correct pin
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
  pinMode(3, OUTPUT); // head
  /*
  Setting   Divisor   Frequency
  0x01      1         31372.55
  0x02      8         3921.16
  0x03      32        980.39
  0x04      64        490.20   <--DEFAULT
  0x05      128       245.10
  0x06      256       122.55
  0x07      1024      30.64
  */
  TCCR2B = TCCR2B & 0b11111000 | 0x05;
  
  // begin serial
  Serial.begin(30000);
  // wait serial to connect
  Serial.flush();
  while(!Serial){}
  Serial.println("Arduino Connected");
}

uint8_t command_id;
char bytes_read[1];
uint8_t current_angle;
float dc;
String str;


void loop() {
  /*
  Serial commands
  1 -> read and implement servomotors positions
  2 -> write ultrasonic measurement
  3 -> write IMU measurements
  4 -> head position
  */
  
  if(Serial.available() > 0){
    // something to read

    str = Serial.readStringUntil("\n");
    command_id = str.toInt();
    //Serial.write(command_id);
    //str = Serial.readStringUntil("\n"); 
    //Serial.println(str.toInt());
    // commands options
    switch(command_id){
      case 1:
        // read motor positions
        for(uint8_t i=0; i<12; i++){
          str = Serial.readStringUntil("\n");
          servomotors[i].write(str.toInt());
        }
      break;
      
      case 2:
      // write ultrasonic measurement
      break;

      case 3:
      // write IMU measurements
      break;

      case 4:
      //head position
        Serial.readBytes(bytes_read, 1);
        current_angle = (uint8_t) bytes_read[0];
        dc = map(current_angle, 0, 180, 30, 160);
        analogWrite(3, dc);

      // Default case
      default:
      break;
    }
  }
  

}
