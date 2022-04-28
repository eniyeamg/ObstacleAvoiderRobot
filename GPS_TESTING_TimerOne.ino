#include <NewPing.h>  //Ultrasonic Sensor 
#include<Wire.h>
#include <Servo.h>    //Library to control servo motor
#include <AFMotor.h> //library for the motor driver shield 
#include <TinyGPS++.h>
#include<TimerOne.h>

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

//GPS Varialbes and Setup

#define GPSSerial   Serial1
int GPS_Course;        //variable to gold gps course to destination
int SatelliteNumbers;  //variable to hold the amount of satellites available
TinyGPSPlus gps;
//////////////////////////////////////////////////////////////////////////////////////
// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Magnetometer and GPS Variables
long int ti;
volatile bool intFlag = false;
int heading_cood;
int16_t mx, my, mz;
int heading;
unsigned long Distance_to_Home;
int ac = 0;             //array counter
int waypointcount = 0;  //waypoint counter
double Home_LAT[50];    //variable for storing the destination latitude
double Home_LON[50];    //variable for storing the destination longitude
int increment = 0;

//Obstacle Avoidance
#define TRIGGER_PIN 22
#define ECHO_PIN 23
#define max_dist  10
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_dist);
int leftDistance;
int rightDistance;
int distance;
boolean object;

//Servo control
Servo myservo;
int pos = 0;

//Bluetooth
#define BLESerial     Serial2
String str;
char bluetoothcase = 0; //variable for storing received data

//Setting up of the DC Motors, will use the AFMotor.h library
AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);
AF_DCMotor motor4(4, MOTOR12_64KHZ);

int turn_Speed = 150;   //motor speed when turning Left and Right
int mtr_Speed = 75;    //motor speed when turning Forward and Reversing


void Stop() {
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  motor3.setSpeed(0);
  motor3.run(RELEASE);
  motor4.setSpeed(0);
  motor4.run(RELEASE);
  delay(100);
  myservo.write(90);
}

void Reverse() {
  motor1.setSpeed(mtr_Speed);
  motor1.run(BACKWARD);
  motor2.setSpeed(mtr_Speed);
  motor2.run(BACKWARD);
  motor3.setSpeed(mtr_Speed);
  motor3.run(BACKWARD);
  motor4.setSpeed(mtr_Speed);
  motor4.run(BACKWARD);
}

void Forward() {
  motor1.setSpeed(mtr_Speed);
  motor1.run(FORWARD);
  motor2.setSpeed(mtr_Speed);
  motor2.run(FORWARD);
  motor3.setSpeed(mtr_Speed);
  motor3.run(FORWARD);
  motor4.setSpeed(mtr_Speed);
  motor4.run(FORWARD);
}

void RightTurn() {
  motor1.setSpeed(mtr_Speed);
  motor1.run(FORWARD);
  motor2.setSpeed(turn_Speed);
  motor2.run(BACKWARD);
  motor3.setSpeed(turn_Speed);
  motor3.run(BACKWARD);
  motor4.setSpeed(mtr_Speed);
  motor4.run(FORWARD);
}

void LeftTurn() {
  motor1.setSpeed(turn_Speed);
  motor1.run(BACKWARD);
  motor2.setSpeed(mtr_Speed);
  motor2.run(FORWARD);
  motor3.setSpeed(mtr_Speed);
  motor3.run(FORWARD);
  motor4.setSpeed(turn_Speed);
  motor4.run(BACKWARD);
}

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  BLESerial.begin(9600);
  GPSSerial.begin(11500);
  Serial.begin(115200);

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

//  pinMode(13, OUTPUT);
//  Timer1.initialize(10000); // initialize timer1, and set a 1/2 second period
//  Timer1.attachInterrupt(callback); // attaches callback() as a timer overflow interrupt


  // Store initial time
  ti = millis();

  myservo.attach(9);                                               // attaches the servo to pin 9
  myservo.attach(10);                                              //attaches the servo to pin 10
  pinMode(TRIGGER_PIN, OUTPUT);                                        // Ping Sensor
  pinMode(ECHO_PIN, INPUT);                                         // Ping Sensor

  Stop();
  delay(2000);


}

// Counter
long int cpt = 0;

//void callback()
//{
//  intFlag = true;
//  digitalWrite(13, digitalRead(13) ^ 1);
//}

// Main loop, read and display data
void MagnetometerReading()
{
//  while (!intFlag);
//  intFlag = false;

  // _____________________
  // ::: Magnetometer :::


  // Read register Status 1 and wait for the DRDY: Data Ready

  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));

  // Read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

  // Create 16 bits values from 8 bits data

  // Magnetometer
  int16_t mx = -(Mag[3] << 8 | Mag[2]);
  int16_t my = -(Mag[1] << 8 | Mag[0]);
  int16_t mz = -(Mag[5] << 8 | Mag[4]);


  // Magnetometer
  BLESerial.print (mx + 200, DEC);
  BLESerial.print ("\t");
  BLESerial.print (my - 70, DEC);
  BLESerial.print ("\t");
  BLESerial.print (mz - 700, DEC);
  BLESerial.print ("\t");
  //Research why minus is in place

  // End of line
  BLESerial.println("");
  delay(1000);
}


float GetHeading() {
  heading_cood = atan2(mx, my) / 0.0174532925;
  if (heading < 0)
  {
    heading += 360;
  }
  return 360 - heading;
}
void bluetooth() {

  if (BLESerial.available() > 0)
  {
    bluetoothcase = BLESerial.read(); //Read the incoming data from the Bluetooth Module and Store into Data
    Serial.print("Recieved: ");
    Serial.println(bluetoothcase);
    Stop();

    switch (bluetoothcase) {
      case '1':
        Serial.println("Forward");
        Forward();
        break;
      case '2':
        Serial.println("Reverse");
        Reverse();
        break;
      case '3':
        Serial.println("Left");
        LeftTurn();
        break;
      case '4':
        Serial.println("Right");
        RightTurn();
        break;
      case '5':
        Serial.println("Stop");
        Stop();
        break;
      case '6':
        Serial.println("Autonomous");
        waypoint();
        break;
    }
  }
}
int getDistance() {
  delay(25);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

int lookLeft() {
  myservo.write(150);
  delay(100);
  leftDistance = getDistance();
  delay(100);
  myservo.write(90);
  Serial.println("Looking left");
  Serial.println(leftDistance);
  return leftDistance;
  delay(50);
}

int lookRight() {
  myservo.write(30);
  delay(100);
  rightDistance = getDistance();
  delay(100);
  myservo.write(90);
  Serial.println("Looking right");
  Serial.println(rightDistance);
  return rightDistance;
  delay(50);
}


void waypoint() {
  int scanningDirection;
  while (true) {

    bluetooth();
    if (bluetoothcase == '5') {
      Stop();
      break;
    }
    else if (bluetoothcase == '6') {
      switch (scanningDirection) {
        case 0:
          myservo.write(30); //right
          scanningDirection = 1;
          break;
        case 1:
          myservo.write(90); //forward
          scanningDirection = 2;
          break;
        case 2:
          myservo.write(150); //left
          scanningDirection = 3;
          break;
        case 3:
          myservo.write(90); //forward
          scanningDirection = 0;
          break;
      }
      delay(500);
      distance = getDistance();
      if (distance <= 20) {
        Stop();

        if (scanningDirection == 1) { //right
          Reverse();
          delay(1000);
          LeftTurn();
          delay(1000);
          Forward();
          delay(1000);
          RightTurn();
          delay(1000);
          Forward();
          delay(1000);
          LeftTurn();
        }
        else if (scanningDirection == 3) { //left
          Reverse();
          delay(1000);
          RightTurn();
          delay(1000);
          Forward();
          delay(1000);
          LeftTurn();
          delay(1000);
          Forward();
          delay(1000);
          RightTurn();
        }

        //        else(scanningDirection == 3 || ==4) {} //otherwise, was looking heading

        Serial.println("Something is in the way! Stop the car!");
        lookLeft();
        Serial.println("Scan the left side");
        lookRight();
        Serial.println("Scan the right side");
      }
      else {
        Distance_to_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LAT[ac], Home_LON[ac]); //Query Tiny GPS for Distance to Destination
        GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Home_LAT[ac], Home_LON[ac]);                            //Query Tiny GPS for Course to Destination

        if (Distance_to_Home == 0) {
          Stop();
          Serial.println("You have arrived!");
          ac++;
          break;
        }

        else {
          if (abs(GPS_Course - heading) <= 10)
          {
            Forward();
          }
          else {
            float heading = GetHeading();
            int x = (GPS_Course - 360);
            int y = (heading - (x));
            int z = (y - 360);
            Serial.println(x);
            Serial.println(y);
            Serial.println(z);

            if ((z <= 180) && (z == 0)) {
              LeftTurn();
              Serial.println("Turn left...");
            }
            else {
              RightTurn();
              Serial.println("Turn right...");
            }
          }
          //turn to face north
          //what direction is the destination?
          //convert from north to forward/backward movement
          //face direction of travel
          //then scan area
          //is obstacle in the way?
          //if not, go forward
          //else scan the area 160 degrees, whichever gives greatest distance go that direction
        }
      }

    }

  }
}


void loop() {
  waypoint();
//Forward();
}
