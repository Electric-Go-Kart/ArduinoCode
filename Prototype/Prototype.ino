/*  Code For E-GoKart Senior Design Project for Colorado State University
 *  Written By: Deagan Malloy
 *  Written For: Arduino Mega
 *  This will be the code which takes the PPM signal from the remote and converts it to PWM for the Motor Driver.
 *  It will also take in distance measurements from LiDar modules and factors that distance into the drive output.
 *  The requirements are two UART Tx and Rx ports, 4 digital PWM input pins, 3 digital PWM output pins, and 1 analog output pin.
 *  
 *  PINS:  2: Throttle input
 *         3: Steer Input
 *         4: Headlights switch input
 *         5: Neutral/ Drive/ Reverse switch input
 *         9: Output for the Steering Servo
 *        10: PWM output for 1st input to H-Bridge
 *        11: PWM output for 2nd input to H-Bridge
 *        22: Output to turn on Headlights 
 */

#include <Servo.h>
Servo steeringServo;

#include <TFMini.h>
TFMini Sensor1;
TFMini Sensor2;

const int throttle = 2;           // channel 3 on reciever
const int steer = 3;              // channel 4 on reciever
const int DriveOrReverse = 5;     // channel 6 on reciever
const int headlights = 4;         // channel 5 on reciever

const int servoPIN = 9;           // Pin for servo output
const int PWMout1 = 10;           // These two will be the outputs for the motors
const int PWMout2 = 11;           // Left and right motor will run at same speed by only using 2 outputs
const int lightsPIN = 22;         // Pin used to turn the headlights on and off

int servoPosNew = 90;

void setup() {
  pinMode(throttle, INPUT);
  pinMode(steer, INPUT);
  pinMode(DriveOrReverse,INPUT);
  pinMode(headlights, INPUT);

  pinMode(PWMout1, OUTPUT);
  pinMode(PWMout2, OUTPUT);
  pinMode(lightsPIN, OUTPUT);

  steeringServo.attach(servoPIN);

  Serial.begin(9600);                                          // Serial with baud rate of 9600
  Serial1.begin(115200);
  Serial2.begin(115200);                                       // Serials for the Sensors
  
  Sensor1.begin(&Serial1);    
  Sensor2.begin(&Serial2);                                     // Initialize Sensors with Serial

  Sensor1.setSingleScanMode();
  Sensor2.setSingleScanMode();                                 // Force each sensor to only read when triggered
  
  delay(50);                                                   // Let signals stabilize
}

void loop() {
  static int throttle_pulse = 1000;                            // These 4 are the pulse length in microseconds that they are active high
  static int steer_pulse = 1500;                               // These 4 are the pulse length in microseconds that they are active high
  static int drive_state_pulse = 1000;                         // These 4 are the pulse length in microseconds that they are active high
  static int headlights_pulse = 1000;                          // These 4 are the pulse length in microseconds that they are active high
  static int drive_state = 0;                                  // 0=neutral, 1=drive, 2=reverse
  static bool safe_path = true;                               // Whether or not the path is clear
  static int counter = 0;
  
  switch(counter){
    case 0:
      throttle_pulse = pulseIn(throttle, HIGH, 30000);
      updateThrottleOut(throttle_pulse, drive_state);
      break;
    case 1:
      steer_pulse = pulseIn(steer, HIGH, 30000);
      updateSteering(steer_pulse);
      break;
    case 2:
      drive_state_pulse = pulseIn(DriveOrReverse, HIGH, 30000);
      drive_state = updateDriveState(drive_state_pulse);
      break;
    case 3:
      headlights_pulse = pulseIn(headlights, HIGH, 30000);
      updateHeadlights(headlights_pulse);
      break;    
    default:
      counter = -1;
      break;      
  }
  counter++;
}

void updateThrottleOut(int &pulse, int &drive_state, bool &safe_path){
  static int setRpm1 = 0, setRpm2 = 0;            // Initialize RPM's

  if(drive_state == 0){                           // Neutral - Ouput Low Low to start breaking and bring car to stop
    setRpm1 = 0;
    setRpm2 = 0;
    analogWrite(PWMout1, setRpm1);                // Set both PWM to 0
    analogWrite(PWMout2, setRpm2);                // Set both PWM to 0
    
  } else if(drive_state == 1 && safe_path){      // Drive - Output High, Low to drive forward
      if(pulse > 1100 && pulse < 2050)                // if not clear path, will go into else and stop car
        setRpm1 = map(pulse, 1100, 2050, 0, 255); // Map input pulse to PWM output duty cycle between 0 and 255
      else 
        setRpm1 = 0;                              // If input pulse is too slow, stay still
        
      setRpm2 = 0;                                // Second needs to be Low to drive Forwards
      analogWrite(PWMout2, setRpm2);              // Set second PWM Low
      delay(10);                                  // Small delay
      analogWrite(PWMout1, setRpm1);              // Set first PWM to PWM signal
    
  } else if(drive_state == 2){                    // Reverse - Output Low High to drive backwards (just opposite of drive)
      if(pulse > 1100 && pulse < 2050)
        setRpm2 = map(pulse, 1100, 2050, 0, 255); // map input pulse to PWM output duty cycle between 0 and 255
      else 
        setRpm2 = 0;                              // If input pulse is too slow, stay still
        
      setRpm1 = 0;                                // First needs to be Low to Reverse
      analogWrite(PWMout1, setRpm1);              // Set first PWM Low
      delay(10);                                  // Small delay
      analogWrite(PWMout2, setRpm2);              // Set second PWM to PWM signal
    
  } else {                                        // Default Case - Output Low Low to stop
      setRpm1 = 0;
      setRpm2 = 0;
      analogWrite(PWMout1, setRpm1);              // Set them both to 0
      analogWrite(PWMout2, setRpm2);              // Set them both to 0
  }
}

void updateHeadlights(int &pulse){                // Reads in if pulse is long or short, and turns on and off headlights
    if(pulse > 1500 && pulse < 2050)
      digitalWrite(lightsPIN, HIGH);              // Turn on headlights
     else 
      digitalWrite(lightsPIN, LOW);               // Turn off headlights    
}

int updateDriveState(int &pulse){
  if(pulse > 1600 && pulse < 2050){               // Reverse
    return 2;
  } else if(pulse > 1400 && pulse <= 1600){       // Drive
      return 1;
  } else if(pulse > 950 && pulse <= 1400){        // Neutral
      return 0;
  }
}

void updateSteering(int &pulse){
  if(pulse < 950 || pulse > 2050){                // If invalid pulse length, return to original position 
    steeringServo.write(90);  
    return;
  } 
  servoPosNew = map(pulse, 980, 2020, 0, 180);    // + 12. Will need to change this to only rotate 90 degrees with of offset of 12 currently in order to recenter after turning 
  if(servoPosNew > 180)
    servoPosNew = 180;
    
  steeringServo.write(servoPosNew);               // Write the new position
  delay(50);
}

boolean distanceCheck(){
  static uint16_t distance_one = 0; 
  static uint16_t distance_two = 0;               // Initialize the Variables
  Sensor1.externalTrigger();                      // Trigger the sensor
  distance_one = Sensor1.getDistance();           // Read Data
  Sensor2.externalTrigger();                      // Repeat
  distance_two = Sensor2.getDistance();
  
  if (distance_one < 100 || distance_two < 100) // Ensure that the distance is correct
    return false;                                 // Less than 100 cm is unsafe
  else
    return true;                                  // Greater than 100 cm is safe
                                                  // Measurements which are out of range will give the value 65535, which is the maximum value of a 16 bit unsigned integer.
                                                  // The range of this sensor should be 1500 cm, but has not been verified.
}
