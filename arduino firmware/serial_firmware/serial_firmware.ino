#include <Servo.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "I2C.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
const int pingPin = A3;  // pin connected to Echo Pin in the ultrasonic distance sensor
const int echoPin = A2;  // pin connected to trig Pin in the ultrasonic distance sensor


uint8_t command_id, data_buf[10];
char bytes_read[1];
byte bytes_write[4];
uint8_t current_angle;
float dc, angle_z[1];
long duration;
double distance;
int packet_idx, data_idx;
String packet_str;
Servo servomotors[12];
Servo head_servo;
int offsets[12];


void ESP8266_ATCOMMAND(){
  /* ESP-01s initialization */ 
   Serial.println("AT+RST");     //reset wifi
   delay(1000);
   Serial.println("AT+CWMODE=2");//set to softAP+station mode
   delay(1000);
   Serial.println("AT+CWSAP=Adeept_ESP,12345678,8,2");   //TCP Protocol, server IP addr, port
   delay(1000);
   Serial.println("AT+CIPMUX=1");
   delay(1000);
   Serial.println("AT+CIPSERVER=1,80");
   delay(1000);
}


void setup() {
  Serial.begin(115200);
  ESP8266_ATCOMMAND();
  Serial.flush();
  while(!Serial){} // wait serial to connect
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

   //Setting pin for ultrasonic read

//  pinMode(trigPin,OUTPUT)//trigger pin is the output
//  pinMode(echoPin,INPUT)//echo pin is the input
//  digitalWrite(trigPin,LOW)//inizialization

  // Motor offsets
  offsets[0] = -5; 
  offsets[1] = 1;
  offsets[2] = -7;
  offsets[3] = -3;
  offsets[4] = 0;
  offsets[5] = -10;
  offsets[6] = 0;
  offsets[7] = 5;
  offsets[8] = -10;
  offsets[9] = 10;
  offsets[10] = 0;
  offsets[11] = -8;

  // Kalman filter initialization
  
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  timer = micros();
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
}




void loop() {
  if(Serial.available() > 0){
    int valid_data=0;
    int data_len=0;
    command_id=0;

    packet_str = Serial.readStringUntil('\n');
    // Serial.print(packet_str);
    /* Parsing TCP string
    +IPD,0,2:Z*/
    packet_idx = packet_str.indexOf("+IPD");
    if(packet_idx!=-1){
      
      data_len=packet_str[packet_idx+7]-48;
      command_id=packet_str[packet_idx+9];
      Serial.print(command_id);
      Serial.print(data_len);
      for(int i=0; i<=data_len-1; i++){
        data_buf[i] = packet_str[10+i];
        valid_data=1;
      }        
    }

    if(valid_data){
      /* commands options 
      1 -> move group 1, 
      2 -> move group 2, 
      3 -> ultrasonic measurements, 
      4 -> IMU measurements, 
      5 -> head position 
      */
      switch(int(command_id)){
        case 1:
          /* read motor positions group 1 */
          data_idx=0;
          Serial.println("AT+CIPSEND=0,6");
          Serial.write(data_buf, 6);
          Serial.println("");
          for(uint8_t i=0; i<=9; i++){
            current_angle = (uint8_t) data_buf[data_idx];
            //Serial.print(current_angle);
            data_idx++;
            servomotors[i].write((int)current_angle+ offsets[i]);
            if(i%2==1){
              i+=2;
            }
          }
        break;
        
        case 2:
          /* read motor positions group 2 */
          data_idx=0;
          for(uint8_t i=2; i<=11; i++){
            current_angle = (uint8_t) data_buf[i];
            data_idx++;
            servomotors[i].write((int)current_angle + offsets[i]);
            if(i%2==1){
              i+=2;
            }
          }  
        break;
  
        case 3:
           //write ultrasonic measurement
          digitalWrite(trigPin, HIGH);
          delayMicroseconds(10);
          digitalWrite(trigPin, LOW);
          duration = pulseIn(echoPin, HIGH);
          distance = duration * 0.034/2;//send out only distance
          memcpy(bytes_write, &distance, 4);
          Serial.println("AT+CIPSEND=0,4");
          Serial.write(bytes_write, 4);
          Serial.print("\r\n");
        break;
  
        case 4:
          /* write IMU measurements */
          /* TCP packet for X angle */
          memcpy(bytes_write, &kalAngleX, 4);
          Serial.println("AT+CIPSEND=0,4");
          Serial.write(bytes_write, 4);
          Serial.print("\r\n");
          /* TCP packet for Y angle */
          memcpy(bytes_write, &kalAngleY, 4);
          Serial.println("AT+CIPSEND=0,4");
          Serial.write(bytes_write, 4);
          Serial.print("\r\n");
        break;
        
        case 5:
          /* head position */
          current_angle = (uint8_t) data_buf[0];
          dc = map(current_angle, 0, 180, 30, 160);
          analogWrite(3, dc);
        break;
        
        default:
          /* Default case */ 
        break;
      }
    
    }
    while (i2cRead(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      kalAngleY = pitch;
    }else{
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    }
    if (abs(kalAngleY) > 90){
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  }
}
