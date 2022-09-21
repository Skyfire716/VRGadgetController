
/*
   See documentation at https://nRF24.github.io/RF24
   See License information at root directory of this library
   Author: Brendan Doherty (2bndy5)
*/

/**
   A simple example of sending data from 1 nRF24L01 transceiver to another.

   This example was written to be used on 2 devices acting as "nodes".
   Use the Serial Monitor to change each node's behavior.
*/
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

// instantiate an object for the nRF24L01 transceiver
RF24 radio(8, 10);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0;

void setup() {

  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  // print example's introductory prompt
  Serial.println(F("RF24/examples/GettingStarted"));

  // To set the radioNumber via the Serial monitor on startup
  Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
  while (!Serial.available()) {
    // wait for user input
  }
  char input = Serial.parseInt();
  radioNumber = input == 1;
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);

  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening();  // put radio in RX mode
  }

  // For debugging info
  //printf_begin();             // needed only once for printing details
  //radio.printDetails();       // (smaller) function that prints raw register values
  //radio.printPrettyDetails(); // (larger) function that prints human readable data

}  // setup

void loop() {

  if (role) {
    // This device is a TX node

    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&payload, sizeof(float));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.println(payload);  // print payload sent
      payload += 0.01;          // increment float payload
    } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }

    // to make this example readable in the serial monitor
    delay(1000);  // slow transmissions down by 1 second

  } else {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload, bytes);             // fetch payload from FIFO
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload0
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      Serial.println(payload);  // print the payload's value
    }
  }  // role

  if (Serial.available()) {
    // change the role via the serial monitor

    char c = toupper(Serial.read());
    if (c == 'T' && !role) {
      // Become the TX node

      role = true;
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      radio.stopListening();

    } else if (c == 'R' && role) {
      // Become the RX node

      role = false;
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
      radio.startListening();
    }
  }

}  // loop


/*
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <float16.h>

#define CE_PIN 8
#define CSN_PIN 10
#define M1_PIN 1
#define M2_PIN 2 
#define M3_PIN 3
#define M4_PIN 4
#define SOLENOID_UP 3
#define SOLENOID_DOWN 4

#define DEBUG 1

RF24 radio(CE_PIN, CSN_PIN);
uint16_t BNO055_SAMPLERATE_DELAY_MS = DEBUG ? 500 : 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
imu::Quaternion quat;
Servo M1;
Servo M2;
Servo M3;
Servo M4;

#define PAYLOAD_SIZE 8
#define RECEIVE_PAYLOAD_SIZE 7
#define RTT_THRESHOLD 200
#define SOLENOID_ACTION_TIME 200      //The solenoids should not be longer active than 10 Minutes in a hour due to overheating => Don't make this time to long

//Recieving 3D Vector for motor Control => 48 Bit
//8 Bit for Solenoids and other Controls
//56Bit Receive Payload => 7

int8_t outputBuf[PAYLOAD_SIZE];
int8_t received[RECEIVE_PAYLOAD_SIZE];


unsigned long solenoid_up_uptime;
unsigned long solenoid_down_uptime;

///@test
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
 
// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
 
// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role
 
// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0;
///TESTEND


void setup() {
  Serial.begin(115200);
  pinMode(SOLENOID_UP, OUTPUT);
  pinMode(SOLENOID_DOWN, OUTPUT);
  digitalWrite(SOLENOID_UP, LOW);
  digitalWrite(SOLENOID_DOWN, LOW);
  while(!Serial);
  Serial.println("VRGadgetController Startup...");
  if(!radio.begin()){
    Serial.println("Cannot connect to RF24");
    while(1);
  }
  Serial.println("Connected to RF24!");
  // Initialise the sensor 
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  M1.attach(M1_PIN);
  M2.attach(M2_PIN);
  M3.attach(M3_PIN);
  M4.attach(M4_PIN);
  
  ///TEST
  Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
  while (!Serial.available()) {
    // wait for user input
  }
  char input = Serial.parseInt();
  radioNumber = input == 1;
  Serial.print(F("radioNumber = "));
  Serial.println((int)radioNumber);
  
  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
 
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setAutoAck(true);
  //radio.setDataRate(RF24_250KBPS);
  //radio.setChannel(122);
  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes
  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0
 
  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
 
  // additional setup specific to the node's role
  if (role) {
    radio.stopListening();  // put radio in TX mode
  } else {
    radio.startListening();  // put radio in RX mode
  }
 
  // For debugging info
   //printf_begin();             // needed only once for printing details
   //radio.printDetails();       // (smaller) function that prints raw register values
   //radio.printPrettyDetails(); // (larger) function that prints human readable data
  
  
  
}

void loop() {
  //Get the current Orientation of the gadget from the IMU
  quat = bno.getQuat();
  //The Adafruit_BNO055 Lib uses doubles to store the parameters of the Quaternion
  //On Teensy 4.1 a double uses 64Bit
  //The BNO055 uses 16Bit per parameter
  //To safe payload we shrink the 256Bit Quaternion from the Lib down to 64Bit
  //The float16 lib expects 32Bit doubles to create a float16 so we cast the 64Bit double to 32Bit Float the the float16 lib can handle
#if DEBUG
  Serial.print("Quaternion: ");
  Serial.print(quat.x());
  Serial.print(", ");
  Serial.print(quat.y());
  Serial.print(", ");
  Serial.print(quat.z());
  Serial.print(", ");
  Serial.println(quat.w());
#endif
  float16 xquat((float) quat.x());
  float16 yquat((float) quat.y());
  float16 zquat((float) quat.z());
  float16 wquat((float) quat.w());
  uint16_t x16 = xquat.getBinary();
  uint16_t y16 = yquat.getBinary();
  uint16_t z16 = zquat.getBinary();
  uint16_t w16 = wquat.getBinary();
  //Clear the old payload
  memset(outputBuf, 0, 8);
  memcpy(outputBuf, &x16, 2);
  memcpy(&outputBuf[2], &y16, 2);
  memcpy(&outputBuf[4], &z16, 2);
  memcpy(&outputBuf[6], &w16, 2);
  //Transfer the payload to the PC
  unsigned long start_timer = micros();
  bool report = radio.write(&outputBuf, PAYLOAD_SIZE);
  if(report){
#if DEBUG
    Serial.println("transmission successful");
#endif
    uint8_t pipe;
    if(radio.available(&pipe)){
      radio.read(&received, RECEIVE_PAYLOAD_SIZE);
      unsigned long end_timer = micros();
      //Only apply the control commands to the hardware if the RTT to the PC is short enough
      if(end_timer - start_timer <= RTT_THRESHOLD){
        int16_t xV, yV, zV;
        uint8_t solenoid;
        memcpy(&xV, received, 2);
        memcpy(&yV, &received[2], 2);
        memcpy(&zV, &received[4], 2);
        memcpy(&solenoid, &received[6], 1);
        //Translate the Force Vector to motor control
        //This dissertation might provide some good concepts of doing so https://repository.upenn.edu/cgi/viewcontent.cgi?article=1705&context=edissertations
        //https://www.araa.asn.au/acra/acra2015/papers/pap109.pdf
        //===============
        //Calculate and set motor speed here to match the Force Vector
        //===============
        
        //===============
        if((solenoid & 0x01) == 0x01){
          digitalWrite(SOLENOID_UP, HIGH);
          solenoid_up_uptime = millis();
        }
        if((solenoid & 0x02) == 0x02){
          digitalWrite(SOLENOID_DOWN, HIGH);
          solenoid_down_uptime = millis();
        }
      }
    }
  }
  if(millis() - solenoid_down_uptime >= SOLENOID_ACTION_TIME){
    digitalWrite(SOLENOID_DOWN, LOW);
  }
  if(millis() - solenoid_up_uptime >= SOLENOID_ACTION_TIME){
    digitalWrite(SOLENOID_UP, LOW);
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}




// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

/*
#include <Wire.h>

void setup() {
  Wire.begin();

  Serial.begin(9600);
  while (!Serial); // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}

void loop() {
  int nDevices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000); // Wait 5 seconds for next scan
}
*/


/*



// This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
//   which provides a common 'type' for sensor data and some helper functions.

//   To use this driver you will also need to download the Adafruit_Sensor
//   library and include it in your libraries folder.

//   You should also assign a unique ID to this sensor for use with
//   the Adafruit Sensor API so that you can identify this particular
//   sensor in any data logs, etc.  To assign a unique ID, simply
//   provide an appropriate value in the constructor below (12345
//   is used by default in this example).

//   Connections
//   ===========
//   Connect SCL to analog 5
//   Connect SDA to analog 4
//   Connect VDD to 3.3-5V DC
//   Connect GROUND to common ground
//
//   History
//   =======
//   2015/MAR/03  - First release (KTOWN)
//

// Set the delay between fresh samples 

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  // Initialise the sensor 
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  Serial.println("Wait Done");
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}




void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

*/
