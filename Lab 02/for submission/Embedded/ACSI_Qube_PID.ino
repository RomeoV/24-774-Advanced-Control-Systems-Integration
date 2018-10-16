/*
   Quanser QUBE Servo Startup - ACSI

   An example using the Arduino UNO board to communicate with the Quanser QUBE Servo
   through the microcontroller interface panel.  This implements a pseudo-controller
   to demonstrate sensor read and motor write functionality.

   Select 250000 baud on the Arduino Serial Tested with Arduino Software (IDE) 1.6.7.

   Original file created 2016 by Quanser Inc.
   www.quanser.com

   Modified for ACSI 2017 by Bin Xu
   www.cmu.edu
 */

// include the QUBE Servo library
#include "QUBEServo.h"

// include ACSI_lib header
#include "ACSI_lib.h"

// include the SPI library and the math library
#include <SPI.h>
#include <math.h>

float cumulate_error = .0;
float previous_error = .0;
float reference = .0;
float prev_nominal_reference = .0;
bool closed_loop = false;
bool input_shaping = true;


void readRef(){
  unsigned long currentTime = micros();
  unsigned long T = 10 * 1e6; // 10 seconds

  unsigned long phase = currentTime % T;
  //Serial.println(phase);
  float peak_reference;

  if(closed_loop == true){
      peak_reference = 45.0 * M_PI / 180.0;
  }
  else{
      peak_reference = 0.5;
  }
  
  float nominal_reference;
  unsigned long current_start;
  if(phase < T/2.0){
      current_start = 0;
      nominal_reference = peak_reference;
      prev_nominal_reference = .0;
  }
  else{
      current_start = T/2.;
      nominal_reference = 0;
      prev_nominal_reference = peak_reference;
  }

  if(!input_shaping){
      reference = nominal_reference;
  }
  else{
      // get these values from matlab to prevent errors and computation
      float A0, A1, A2;
      float t1, t2;
      if(closed_loop){
         A0 = 0.3688;
         A1 = 0.4770;
         A2 = 0.1542;
         t1 = 0.2040;
         t2 = 0.4080;  
      }
      else{
         A0 = 0.3313;
         A1 = 0.4886;
         A2 = 0.1802;
         t1 = 0.1960;
         t2 = 0.3920;      
      }

      float phase_time = (phase - current_start) * 1e-6;
      if(phase_time < t1){
        reference = prev_nominal_reference + A0 * (nominal_reference - prev_nominal_reference);
      }
      else if(t1 < phase_time && phase_time < t2){
        reference = prev_nominal_reference + (A0 + A1) * (nominal_reference - prev_nominal_reference);
      }
      else{
        reference = nominal_reference;
      }
      /* 
      Serial.print("current phase is ");
      Serial.print(phase*1e-6);
      Serial.print(", phase time is ");
      Serial.print(phase_time);
      Serial.print(", nominal reference is ");
      Serial.print(nominal_reference);
      Serial.print(", reference is ");
      Serial.println(reference);
      */
  }

}


// Don't touch the setup unless you decide to use interrupts
void setup() {
	// set the slaveSelectPin as an output
	pinMode (slaveSelectPin, OUTPUT);

	// initialize SPI
	SPI.begin();

	// initialize serial communication at 250000 baud
	// (Note that 250000 baud must be selected from the drop-down list on the Arduino
	// Serial Monitor for the data to be displayed properly.)
	Serial.begin(250000);
}

void loop() {

	// after the Arduino power is cycled or the reset pushbutton is pressed, call the resetQUBEServo function
	// Don't touch
	if (startup) {
		resetQUBEServo();
		startup = false;
	}

	// if the difference between the current time and the last time an SPI transaction
	// occurred is greater than the sample time, start a new SPI transaction
	// Alternatively, use a timer interrupt
	unsigned long currentMicros = micros();
	if (currentMicros - previousMicros >= sampleTime) {
		previousMicros = previousMicros + sampleTime;  

		// Simple function for reading from sensors.  Data are read into global variables.  For variable definitions, see ACSI_lib.h.
		readSensors();
		// This will define the data that will be displayed at the serial terminal.
		//displayData.buildString(theta, alpha, currentSense, moduleID, moduleStatus);

		// Note: theta - motor angle, alpha - pendulum, note that wrapped around and pointing down position is -3.14


		//Below demonstrates changing the LED state (you probably don't care) and changing the motor voltage (you certainly DO care)
		LEDRed = 999;
    LEDGreen = 999;
    LEDBlue = 0;

    // read the reference 
    readRef();
		//This command actually writes the data to the Qube servo
    if(closed_loop){
		  float kp = 1.0, ki = 0.0015, kd = 150.0; // matlab values
	  	float error = reference - theta;
      float derror = error - previous_error;

      if(fabs(motorVoltage) <= 10){
        // prevent saturation error
        cumulate_error += error;
      }
		  motorVoltage = kp * error + ki * cumulate_error + kd * derror; 
      previous_error = error;
    
      // the direction seems to be opposite
      motorVoltage = -motorVoltage;
    }
    else{
      motorVoltage = reference;
    }
    //motorVoltage = 0;
		driveMotor();
	}


	// print data to the Arduino Serial Monitor in between SPI transactions
	// (Note that the Serial.print() function is time consuming.  Printing the entire
	// string at once would exceed the sample time required to balance the pendulum.)
	else {  //We're in between samples
		// only print if there's a string ready to be printed, and there's enough time before the next SPI transaction
		if (currentMicros - previousMicros <= (sampleTime - 100) ) {
			// if there is room available in the serial buffer, print one character
			if(Serial.availableForWrite() > 0) {
				/*
				Serial.print("pendulum is ");
				Serial.println(alpha);
				Serial.print("motor is ");
				Serial.println(theta);
        Serial.println(micros());
				Serial.println("##############################");
				*/
				/*
				   Serial.print(displayData.dData[displayData.dDataIndex]);
				   displayData.dDataIndex = displayData.dDataIndex + 1;
				// if the entire string has been printed, clear the flag so a new string can be obtained
				if(displayData.dDataIndex == displayData.dData.length()) {
				displayData.dDataReady = false;
				}
				 */

			}
		}
	}
}
