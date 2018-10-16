/*
24-774 Advanced Control System Integration
*/

#include <Arduino.h>
#include "QUBEServo.h"
#include "ACSI_lib.h"
#include <SPI.h>
#include <math.h>

bool startup = true;  // true the first time the sketch is run after the Arduino power is cycled or the reset pushbutton is pressed

unsigned long previousMicros = 0;  // used to store the last time the SPI data was written
const long sampleTime = 1000;  // set the sample time (the interval between SPI transactions) to 1000us = 1ms

// set pin 10 as the slave select for the Quanser QUBE
// (Note that if a different pin is used for the slave select, pin 10 should be set as
// an output to prevent accidentally putting the Arduino UNO into slave mode.)
const int slaveSelectPin = 10;

// initialize the SPI data to be written
byte mode = 1;                      // normal mode = 1
byte writeMask = B00011111;         // Bxxxxxx11 to enable the motor, Bxxx111xx to enable the LEDs, Bx11xxxxx to enable writes to the encoders
byte LEDRedMSB = 0;                 // red LED command MSB
byte LEDRedLSB = 0;                 // red LED command LSB
byte LEDGreenMSB = 0;               // green LED command MSB
byte LEDGreenLSB = 0;               // green LED command LSB
byte LEDBlueMSB = 0;                // blue LED command MSB
byte LEDBlueLSB = 0;                // blue LED command LSB
byte encoder0ByteSet[3] = {0,0,0};  // encoder0 is set to this value only when writes are enabled with writeMask
byte encoder1ByteSet[3] = {0,0,0};  // encoder1 is set to this value only when writes are enabled with writeMask
byte motorMSB = 0x80;               // motor command MSB must be B1xxxxxxx to enable the amplifier
byte motorLSB = 0;                  // motor command LSB

// initialize the SPI data to be read
byte moduleIDMSB = 0;               // module ID MSB (module ID for the QUBE Servo is 777 decimal)
byte moduleIDLSB = 0;               // module ID LSB
byte encoder0Byte[3] = {0,0,0};     // arm encoder counts
byte encoder1Byte[3] = {0,0,0};     // pendulum encoder counts
byte tach0Byte[3] = {0,0,0};        // arm tachometer
byte moduleStatus = 0;              // module status (the QUBE Servo sends status = 0 when there are no errors)
byte currentSenseMSB = 0;           // motor current sense MSB 
byte currentSenseLSB = 0;           // motor current sense LSB

// global variables for LED intensity (999 is maximum intensity, 0 is off)
int LEDRed = 0;
int LEDGreen = 0;
int LEDBlue = 0;

// Setup global variables for wrap up function
float alpha = 0.0;  // pendulum angle in radians
float alpha_prev = 0.0;
float alpha_dot = 0.0;
float theta = 0.0;  // arm angle in radians
float theta_prev = 0.0;
float theta_dot = 0.0;
float ObsInternalState[4] = {0,0,0,0};
float currentSense = 0.0;
int moduleID = 0;

float motorVoltage = 1.0;

//Setup serial builder
Display displayData;

void readSensors() {
  // initialize the SPI bus using the defined speed, data order and data mode
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    // take the slave select pin low to select the device
    digitalWrite(slaveSelectPin, LOW);
    
    // send and receive the data via SPI (except for the motor command, which is sent after the pendulum control code) 
    moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
    moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
    encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
    encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
    encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
    encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
    encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
    encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
    tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
    tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
    tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
    moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
    currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
    currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
    SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0
    
    // combine the received bytes to assemble the sensor values

    /*Module ID*/
    moduleID = (moduleIDMSB << 8) | moduleIDLSB;
    
    /*Motor Encoder Counts*/
    long encoder0 = ((long)encoder0Byte[2] << 16) | (long)(encoder0Byte[1] << 8) | encoder0Byte[0];
    if (encoder0 & 0x00800000) {
      encoder0 = encoder0 | 0xFF000000;
    }
    // convert the arm encoder counts to angle theta in radians
    theta = (float)encoder0 * (-2.0 * M_PI / 2048);

    /*Pendulum Encoder Counts*/
    long encoder1 = ((long)encoder1Byte[2] << 16) | (long)(encoder1Byte[1] << 8) | encoder1Byte[0];
    if (encoder1 & 0x00800000) {
      encoder1 = encoder1 | 0xFF000000;
    }
    // wrap the pendulum encoder counts when the pendulum is rotated more than 360 degrees
    encoder1 = encoder1 % 2048;
    if (encoder1 < 0) {
      encoder1 = 2048 + encoder1;
    }    
    // convert the pendulum encoder counts to angle alpha in radians
    alpha = (float)encoder1 * (2.0 * M_PI / 2048) - M_PI;

    /*Current Sense Value*/
    currentSense = (currentSenseMSB << 8) | currentSenseLSB;
}

void driveMotor() {
    // convert the analog value to the PWM duty cycle that will produce the same average voltage
    float motorPWM = motorVoltage * (625.0 / 15.0);
    
    int motor = (int)motorPWM;  // convert float to int (2 bytes)
    motor = motor | 0x8000;  // motor command MSB must be B1xxxxxxx to enable the amplifier
    motorMSB = (byte)(motor >> 8);
    motorLSB = (byte)(motor & 0x00FF);
    
  // convert the LED intensities to MSB and LSB
    LEDRedMSB = (byte)(LEDRed >> 8);
    LEDRedLSB = (byte)(LEDRed & 0x00FF);
    LEDGreenMSB = (byte)(LEDGreen >> 8);
    LEDGreenLSB = (byte)(LEDGreen & 0x00FF);
    LEDBlueMSB = (byte)(LEDBlue >> 8);
    LEDBlueLSB = (byte)(LEDBlue & 0x00FF);
    
    // send the motor data via SPI
    SPI.transfer(motorMSB);
    SPI.transfer(motorLSB);
    
    // take the slave select pin high to de-select the device
    digitalWrite(slaveSelectPin, HIGH);
    SPI.endTransaction();
}

// This function is used to clear the stall error and reset the encoder values to 0.
// The motor and LEDs are turned off when this function is called.
void resetQUBEServo() {
  
  // enable the motor and LEDs, and enable writes to the encoders
  writeMask = B01111111;
  
  // turn off the LEDs
  LEDRedMSB = 0;
  LEDRedLSB = 0;
  LEDGreenMSB = 0;
  LEDGreenLSB = 0;
  LEDBlueMSB = 0;
  LEDBlueLSB = 0;
  
  // reset the encoder values to 0
  encoder0ByteSet[2] = 0;
  encoder0ByteSet[1] = 0;
  encoder0ByteSet[0] = 0;
  encoder1ByteSet[2] = 0;
  encoder1ByteSet[1] = 0;
  encoder1ByteSet[0] = 0;
  
  // turn off the motor, and clear the stall error by disabling the amplifier
  motorMSB = 0;  // motor command MSB is B0xxxxxxx to disable the amplifier
  motorLSB = 0;
  
  // initialize the SPI bus using the defined speed, data order and data mode
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  digitalWrite(slaveSelectPin, LOW);   // take the slave select pin low to select the device
  
  // send and receive the data via SPI
  moduleIDMSB = SPI.transfer(mode);                    // read the module ID MSB, send the mode
  moduleIDLSB = SPI.transfer(0);                       // read the module ID LSB
  encoder0Byte[2] = SPI.transfer(writeMask);           // read encoder0 byte 2, send the write mask
  encoder0Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder0 byte 1, send the red LED MSB
  encoder0Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder0 byte 0, send the red LED LSB
  encoder1Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder1 byte 2, send the green LED MSB
  encoder1Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder1 byte 1, send the green LED LSB
  encoder1Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder1 byte 0, send the blue LED MSB
  tach0Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer0 byte 2, send the blue LED LSB
  tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2
  tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
  moduleStatus = SPI.transfer(encoder0ByteSet[0]);     // read the status, send encoder0 byte 0
  currentSenseMSB = SPI.transfer(encoder1ByteSet[2]);  // read the current sense MSB, send encoder1 byte 2
  currentSenseLSB = SPI.transfer(encoder1ByteSet[1]);  // read the current sense LSB, send encoder1 byte 1
  SPI.transfer(encoder1ByteSet[0]);                    // send encoder1 byte 0
  SPI.transfer(motorMSB);                              // send the motor MSB
  SPI.transfer(motorLSB);                              // send the motor LSB
  
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  SPI.endTransaction();
  
  writeMask = B00011111;  // enable the motor and LEDs, disable writes to the encoders
  motorMSB = 0x80;  // enable the amplifier
}


void doStateEstimationByDifferenceEquation(float dt) {
  //alpha = alpha+M_PI; // Let hanging position be alpha=pi => upright position=0

  alpha_dot = (alpha-alpha_prev)/dt;
  theta_dot = (theta-theta_prev)/dt;

  alpha_prev = alpha;
  theta_prev = theta;
}

void doDifferentialObserverStep() {

    float A_diff_obs[2][2] = {
	    {0.96079,0},
	    {0,0.96079}};

    float B_diff_obs[2][2] = {
	    {0.030749,0},
	    {0,0.030749}};

    float C_diff_obs[4][2] = {
	    {0,0},
	    {0,0},
	    {-50,0},
	    {0,-50}};

    float D_diff_obs[4][2] = {
	    {1,0},
	    {0,1},
	    {39.211,0},
	    {0,39.211}};

  
    float StateX[4] = {0.0,0.0,0.0,0.0};
    for (int el = 0; el < 4; el++) {
         StateX[el] = C_diff_obs[el][0]*ObsInternalState[0] + C_diff_obs[el][1]*ObsInternalState[1] + D_diff_obs[el][0]*theta + D_diff_obs[el][1]*alpha;
    }
    /*
        Serial.println("#################################################################");
        Serial.print("Differential State = ["); 
        Serial.print(StateX[0]); Serial.print(", ");
        Serial.print(StateX[1]); Serial.print(", ");
        Serial.print(StateX[2]); Serial.print(", ");
        Serial.print(StateX[3]); Serial.println("]");
	*/

    float tempState0, tempState1;
    tempState0 = A_diff_obs[0][0]*ObsInternalState[0] + A_diff_obs[0][1]*ObsInternalState[1] + B_diff_obs[0][0]*theta + B_diff_obs[0][1]*alpha;
    tempState1 = A_diff_obs[1][0]*ObsInternalState[0] + A_diff_obs[1][1]*ObsInternalState[1] + B_diff_obs[1][0]*theta + B_diff_obs[1][1]*alpha;
    ObsInternalState[0] = tempState0;
    ObsInternalState[1] = tempState1;
    
    theta = StateX[0];
    alpha = StateX[1];
    theta_dot = StateX[2];
    alpha_dot = StateX[3];
}

void setControlInput() {
  float current_time = micros();
  float theta_ref = sin(1e-6 * current_time * 6 * M_PI);
  float alpha_ref = 0.;
  float theta_dot_ref = 0.;
  float alpha_dot_ref = 0.;

  float theta_error = theta_ref - theta;
  float alpha_error = alpha_ref - alpha ;
  float theta_dot_error = theta_dot_ref - theta_dot;
  float alpha_dot_error = alpha_dot_ref - alpha_dot;

  /*
        Serial.println("#################################################################");
        Serial.print("Differential State = ["); 
        Serial.print(theta_error); Serial.print(", ");
        Serial.print(alpha_error); Serial.print(", ");
        Serial.print(theta_dot_error); Serial.print(", ");
        Serial.print(alpha_dot_error); Serial.println("]");
	*/

  float lqr_matrix[] = {-7.0711, 48.0855, -2.3960, 4.0603};
  //float lqr_matrix[] = {-10.0000,55.1537,-3.0227,4.7446};
  //float lqr_matrix[] = {-3.1623,38.3300,-1.5197,3.0998};
  if (fabs(alpha) < 0.4) {
    motorVoltage = lqr_matrix[0]*theta_error + lqr_matrix[1]*alpha_error + lqr_matrix[2]*theta_dot_error + lqr_matrix[3]*alpha_dot_error;
  }
  else {
    motorVoltage = 0;
  }

  motorVoltage *= -1;
}
