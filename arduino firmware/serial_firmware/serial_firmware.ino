#include <Servo.h>
#define TRIG_PIN A2 
#define ECHO_PIN A3

// Servomotors declaration
Servo servomotors[12];
Servo head_servo;

void ESP8266_ATCOMMAND(){
   Serial.begin(115200);       // set up a wifi serial communication baud rate 115200
//   pinMode(controller, OUTPUT);    // sets the RelayPin to be an output
   Serial.println("AT+CWMODE=3\r\n");//set to softAP+station mode
   delay(2000);     //delay 4s
   
   Serial.println("AT+CWSAP=\"Adeept_ESP\",\"12345678\",8,2\r\n");   //TCP Protocol, server IP addr, port
    delay(2000);     //delay 4s
   Serial.println("AT+RST\r\n");     //reset wifi
   delay(2000);     //delay 4s

   Serial.println("AT+CIPMUX=1\r\n");
   delay(2000);
   Serial.println("AT+CIPSERVER=1,8080\r\n");
   delay(2000);
   Serial.println("AT+CIPSTO=7000\r\n");
   delay(2000);
}


void setup() {
  // Attach servomotors to the correct pin
  servomotors[0].attach(A0); //s11
  servomotors[1].attach(2); //s12
  servomotors[2].attach(5); //s21
  servomotors[3].attach(4); //s22
  servomotors[4].attach(7); //s31
  servomotors[5].attach(6); //s32
  //servomotors[6].attach(13); //s41
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
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

  /* Setup Ultrasonic sensor HC-SR04 */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // begin serial
  Serial.begin(115200);
  ESP8266_ATCOMMAND();
  // wait serial to connect
  Serial.flush();
  while(!Serial){}
  Serial.println("Arduino Connected");
}

uint8_t command_id;
char bytes_read[1];
uint8_t current_angle;
float dc;
long duration;
int distance;


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
    digitalWrite(13, HIGH);
    command_id = Serial.read();  
    //Serial.write(command_id);
    

    // commands options
    switch(int(command_id)){
      case 1:
        
        // read motor positions
        for(int8_t i=0; i<12; i++){
          current_angle = uint8_t(Serial.read());
          //current_angle = (uint8_t) bytes_read[0];
          servomotors[i].write((int)current_angle);
        }
      break;
      
      case 2:
        // write ultrasonic measurement
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        duration = pulseIn(ECHO_PIN, HIGH);
        distance = duration * 0.034/2;

        // Write on Serial port
        //Serial.write()
        
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
