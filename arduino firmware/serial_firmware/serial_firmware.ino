#include <Servo.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

// #define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
const int trigPin = A3;  // pin connected to Echo Pin in the ultrasonic distance sensor
const int echoPin = A2;  // pin connected to trig Pin in the ultrasonic distance sensor




//#define TRIG_PIN A2 
//#define ECHO_PIN A3

// Servomotors declaration
Servo servomotors[12];
Servo head_servo;
int offsets[12];

//void ESP8266_ATCOMMAND(){
//   Serial.begin(115200);       // set up a wifi serial communication baud rate 115200
//   pinMode(controller, OUTPUT);    // sets the RelayPin to be an output
//   Serial.println("AT+CWMODE=3\r\n");//set to softAP+station mode
//   delay(2000);     //delay 4s
//   
//   Serial.println("AT+CWSAP=\"Adeept_ESP\",\"12345678\",8,2\r\n");   //TCP Protocol, server IP addr, port
//    delay(2000);     //delay 4s
//   Serial.println("AT+RST\r\n");     //reset wifi
//   delay(2000);     //delay 4s
//
//   Serial.println("AT+CIPMUX=1\r\n");
//   delay(2000);
//   Serial.println("AT+CIPSERVER=1,8080\r\n");
//   delay(2000);
//   Serial.println("AT+CIPSTO=7000\r\n");
//   delay(2000);
//}


void setup() {
  // Attach servomotors to the correct pin
  servomotors[0].attach(A0); //s11
  servomotors[1].attach(2); //s12
  servomotors[2].attach(5); //s21
  servomotors[3].attach(4); //s22
  servomotors[4].attach(7); //s31
  servomotors[5].attach(6); //s32
  servomotors[6].attach(13); //s41
//  pinMode(13, OUTPUT);
//  digitalWrite(13, LOW);
  servomotors[7].attach(12); //s42
  servomotors[8].attach(11); //s51
  servomotors[9].attach(10); //s52
  servomotors[10].attach(9); //s61
  servomotors[11].attach(8); //s62
  pinMode(3, OUTPUT); // head
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

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  pinMode(trigPin,OUTPUT);//trigger pin is the output
  pinMode(echoPin,INPUT);//echo pin is the input
  digitalWrite(trigPin,LOW);//inizialization

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
  
  // begin serial
  Serial.begin(57600);
  //ESP8266_ATCOMMAND();
  // wait serial to connect
  Serial.flush();
  while(!Serial){}
  Serial.println("Arduino Connected");
}

uint8_t command_id;
char bytes_read[1];
byte bytes_write[4];
uint8_t current_angle;
float dc, angle_z[1];
long duration;
double distance;

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
    //digitalWrite(13, HIGH);
    command_id = (uint8_t) Serial.read();

    // commands options 1 -> move group 1, 2 -> move group 2, 3 -> ultrasonic measurements, 4 -> IMU measurements, 5 -> head position
    switch(int(command_id)){
      case 1:
        // read motor positions group 1
        for(uint8_t i=0; i<=9; i++){
          Serial.readBytes(bytes_read, 1);
          current_angle = (uint8_t) bytes_read[0];
          servomotors[i].write((int)current_angle+ offsets[i]);
          if(i%2==1){
            i+=2;
          }
        }
      break;
      
      case 2:
        // read motor positions group 2
        for(uint8_t i=2; i<=11; i++){
          Serial.readBytes(bytes_read, 1);
          current_angle = (uint8_t) bytes_read[0];
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
          
          Serial.write(bytes_write, 4);
          
      break;

      case 4:
        memcpy(bytes_write, &kalAngleX, 4);
        Serial.write(bytes_write, 4);
        memcpy(bytes_write, &kalAngleY, 4);
        Serial.write(bytes_write, 4);
        break;
      case 5:
        //head position
        Serial.readBytes(bytes_read, 1);
        current_angle = (uint8_t) bytes_read[0];
        dc = map(current_angle, 0, 180, 30, 160);
        analogWrite(3, dc);
        break;
      // Default case
      default:
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
