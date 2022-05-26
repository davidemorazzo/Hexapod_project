#include <Servo.h>

// Servomotors declaration
Servo servomotors[12];
Servo head_servo;

void setup() {
  // Attach servomotors to the correct pin
  servomotors[0].attach(3); //s11
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
  head_servo.attach(A0);

  // begin serial
  Serial.begin(115200);
  // wait serial to connect
  Serial.flush();
  while(!Serial){}
  Serial.println("Arduino Connected");
  servomotors[0].write(0);
}

uint8_t command_id;
char bytes_read[1];
char angle_buffer[20] = {0};
uint8_t current_angle;


void loop() {
  /*
  Serial commands
  1 -> read and implement servomotors positions
  2 -> write ultrasonic measurement
  3 -> write IMU measurements
  */
  
  if(Serial.available() > 0){
    // something to read

    command_id = Serial.read();  
    Serial.write(command_id);
    

    // commands options
    switch(int(command_id)){
      case 1:
        
        // read motor positions
        //bytes_read = Serial.readBytesUntil('\n', angle_buffer, 20);
        //Serial.write(bytes_read);
        // motor positions are coded in int16
        
        for(int8_t i=0; i<12; i++){
          //int16_t current_angle = angle_buffer[i];
          Serial.readBytes(bytes_read, 1);
          current_angle = (uint8_t) bytes_read[0];
          servomotors[i].write((int)current_angle);
        }
      break;
      
      case 2:
      // write ultrasonic measurement
      break;

      case 3:
      // write IMU measurements
      break;
      
      default:
      break;
    }
  }
  

}
