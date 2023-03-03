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

#define M1_PIN 6
#define M2_PIN 5 
#define M3_PIN 7
#define M4_PIN 4
#define SOLENOID_UP 2
#define SOLENOID_DOWN 3

#define DEBUG 0

RF24 radio(CE_PIN, CSN_PIN);
uint16_t BNO055_SAMPLERATE_DELAY_MS = DEBUG ? 500 : 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
imu::Quaternion quat;
Servo M1;     //Top Left
Servo M2;     //Top Right
Servo M3;     //Bottom Left
Servo M4;     //Bottom Right

#define PAYLOAD_SIZE 8
#define RECEIVE_PAYLOAD_SIZE 7
#define RTT_THRESHOLD 1750
#define SOLENOID_ACTION_TIME 200      //The solenoids should not be longer active than 10 Minutes in a hour due to overheating => Don't make this time to long
#define THROTTLE_ACTION_TIME 300      //Throttle Up time before shutdown

//Recieving 3D Vector for motor Control => 48 Bit
//8 Bit for Solenoids and other Controls
//56Bit Receive Payload => 7

int8_t outputBuf[PAYLOAD_SIZE];
int8_t received[RECEIVE_PAYLOAD_SIZE];

unsigned long solenoid_up_uptime;
unsigned long solenoid_down_uptime;
unsigned long throttle_uptime;

///@test
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = true;  // true = TX role, false = RX role

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
  // while(!Serial);
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
    // while (1);
  }
  M1.attach(M1_PIN, 1000, 2000);
  M2.attach(M2_PIN, 1000, 2000);
  M3.attach(M3_PIN, 1000, 2000);
  M4.attach(M4_PIN, 1000, 2000);
  
  M3.write(90);
  M4.write(90);
  
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.enableDynamicPayloads(); 
  radio.enableAckPayload();
  
  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
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
 // printf_begin();             // needed only once for printing details
//   radio.printDetails();       // (smaller) function that prints raw register values
//   radio.printPrettyDetails(); // (larger) function that prints human readable data
  
  Serial.print("Setup Done"); 
  
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
  // Transfer the payload to the PC
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
        //Only React when 0x04 is high
        if((solenoid & 0x04) == 0x04){
          //Translate the Force Vector to motor control
          //This dissertation might provide some good concepts of doing so https://repository.upenn.edu/cgi/viewcontent.cgi?article=1705&context=edissertations
          //https://www.araa.asn.au/acra/acra2015/papers/pap109.pdf
          //===============
          //Calculate and set motor speed here to match the Force Vector
          //===============
          #if DEBUG
          Serial.print("Vector ");
          Serial.print(xV);
          Serial.print(", ");
          Serial.print(yV);
          Serial.print(", ");
          Serial.println(zV);
          #endif
          if(!(xV == 0 && yV == 0 && zV == 0)){
            imu::Vector<3> forceVector(xV, zV, yV);
            double throttle = forceVector.magnitude();
            forceVector.normalize();
            imu::Vector<3> rotationVector(0, 0, 1);
            imu::Quaternion rot;
            rot.fromAxisAngle(rotationVector, 45);
            imu::Vector<3> XVector(0, 1, 0);
            imu::Vector<3> YVector(1, 0, 0);
            imu::Vector<3> normalXVector = rot.rotateVector(XVector);
            imu::Vector<3> normalYVector = rot.rotateVector(YVector);
            double xAngle = acos(forceVector.dot(normalXVector) / (forceVector.magnitude() * normalXVector.magnitude()));
            double yAngle = acos(forceVector.dot(normalYVector) / (forceVector.magnitude() * normalYVector.magnitude()));
            double xAngleDeg = xAngle * 180 / PI;
            double yAngleDeg = yAngle * 180 / PI;
            #if DEBUG
            Serial.print("Throttle: ");
            Serial.print(throttle);
            Serial.print(" AngleX ");
            Serial.print(xAngleDeg);
            Serial.print(", AngleY ");
            Serial.println(yAngleDeg);
            #endif
            byte throttleM1, throttleM2, throttleM3, throttleM4;
            throttleM1 = throttleM2 = throttleM3 = throttleM4 = 90;
            //TODO Further Thrust Vectoring
            //Thrust Vector is a normal Vector of a plane representing the impulse of the collision
            //The local coordinate system of the drone gets rotated so x and y are the diagonals through two diagonal aligned motors
            //The angle between these diagonal x and y vectors and the thrust vector result into the partial thrust each motor has to deliver
            //Further calculations need to be done.
            double xboost = map(tan(xAngle), -45, 45, -100, 100);
            double yboost = map(tan(yAngle), -45, 45, -100, 100);
            
            
            
            if(abs(xboost) > abs(yboost)){
              throttleM1 = map(throttle, -100, 100, 0, 180);
              throttleM2 = map(throttle * (100 - (abs(xboost) - abs(yboost))) / 100.0, -100, 100, 0, 180);
              throttleM3 = map(throttle * (100 - (abs(xboost) - abs(yboost)) * 2 * abs(yboost)) / 100.0, -100, 100, 0, 180);
              throttleM4 = map(throttle * (100 - 2 * abs(xboost)) / 100.0, -100, 100, 0, 180);
            }else{
              
            }
            #if DEBUG
            Serial.print("Throttle ");
            Serial.print(map(throttleM1, 0, 180, -100, 100));
            Serial.print("%,  ");
            Serial.print(map(throttleM2, 0, 180, -100, 100));
            Serial.print("%,  ");
            Serial.print(map(throttleM3, 0, 180, -100, 100));
            Serial.print("%,  ");
            Serial.print(map(throttleM4, 0, 180, -100, 100));
            Serial.println("%");
            #endif
            M1.write(throttleM1);
            M2.write(throttleM2);
            M3.write(throttleM3);
            M4.write(throttleM4);
            throttle_uptime = millis();
          }
          //===============
          if((solenoid & 0x01) == 0x01){
            digitalWrite(SOLENOID_UP, HIGH);
            solenoid_up_uptime = millis();
            #if DEBUG
            Serial.println("Trigger Solenoid Up");
            #endif
          }
          if((solenoid & 0x02) == 0x02){
            digitalWrite(SOLENOID_DOWN, HIGH);
            solenoid_down_uptime = millis();
            #if DEBUG
            Serial.println("Trigger Solenoid Down");
            #endif
          }
        }
        memset(received, 0, 7);
      }
    }
  }else{
    #if DEBUG
    Serial.println("Transmission Failed");
    #endif
  }
  if(millis() - solenoid_down_uptime >= SOLENOID_ACTION_TIME){
    digitalWrite(SOLENOID_DOWN, LOW);
  }
  if(millis() - solenoid_up_uptime >= SOLENOID_ACTION_TIME){
    digitalWrite(SOLENOID_UP, LOW);
  }
  if(millis() - throttle_uptime >= THROTTLE_ACTION_TIME){
    M1.write(90);
    M2.write(90);
    M3.write(90);
    M4.write(90);
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
