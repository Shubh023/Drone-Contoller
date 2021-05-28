#include "NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <Servo.h> 
#include <PID_v1.h>

bool stopped = true;
float manual = 0.0;
#define STOPVAL  10

#define KP 1.3
#define KI 0.03
#define KD 15.0
#define DESIRED_VAL 0.0
#define LEFTMIN 1000.0
#define RIGHTMIN 1000.0
#define LEFTMAX 2000.0
#define RIGHTMAX 2000.0
#define USEDMIN 1050 // Reference is 1020
#define USEDMAX 1350 // Reference is 1350
#define MINANGLE -60.0
#define MAXANGLE 60.0


Servo left;
Servo right;

NineAxesMotion mySensor;         //Object that for the sensor 
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 10;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;

float accX;
float linX;
double roll = 0.0;
float rollprev = 0.0;
float leftval = 0.0;
float rightval = 0.0;
double Output = 0.0;
double Setpoint = 0.0;
double Kp=1.2, Ki=0.05, Kd=2;
PID myPID(&roll, &Output, &Setpoint, Kp, Ki, Kd, AUTOMATIC);

void setup() //This code is executed once
{
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor
  mySensor.updateAccelConfig();
  updateSensorData = true;
  
  //////////////////////// ESC CALIBRATION //////////////////////////////
  // Serial.println("Starting ESCs Calibration");
  left.attach(5);  // PWM Pin connnection for left Motor 
  right.attach(6);  // PWM Pin connnection for left Motor 
  delay(1);
  left.writeMicroseconds(LEFTMAX);
  right.writeMicroseconds(RIGHTMAX);
  delay(500);
  left.writeMicroseconds(LEFTMIN);
  right.writeMicroseconds(RIGHTMIN);
  delay(100);
  
  // Mandatory Wait for ESC to be armed Correctly 
  delay(6000);
  //Serial.println("Finished Calibration");  

  // SETUP PID
  Setpoint = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(MINANGLE, MAXANGLE);
}

boolean debounce(boolean state)
{
  boolean stateNow = analogRead(A1) < 200;
  if (state != stateNow)
  {
    delay(25);
    stateNow = analogRead(A1) < 200;
  }
  return stateNow;
}

void loop() //This code is looped forever
{
  if (debounce(stopped) == true and stopped == false)
  {
    stopped = true;
  }
  else if (debounce(stopped) == false and stopped == true)
  {
    stopped = false;
  }
  
  rollprev = roll;
  if (updateSensorData)  //Keep the updating of data as a separate task
  {
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status
    updateSensorData = false;
  }
  
  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();
    mySensor.updateEuler();        //Update the Euler data into the structure of the object
    mySensor.updateCalibStatus();  //Update the Calibration Status

    // accX = tmySensor.readAccelerometer(X_AXIS);
    //linX = mySensor.readLinearAcceleration(X_AXIS);

    /*
    Serial.print(" AccX: ");
    Serial.print(accX); //Accelerometer X-Axis data in m/s2

    Serial.print(" LinX: ");
    Serial.print(linX); //Accelerometer X-Axis data in m/s2
    */

    
    if (stopped) {
      roll = 0.0;
      manual = 0.0;
    }
    else {
      manual = analogRead(A0);
      roll = (double)mySensor.readEulerRoll();
    }

    /*
    Serial.print(" stopped: ");
    Serial.print(int(100 * stopped));

    Serial.print(" manual: ");
    Serial.print(manual);

    p_output =  (roll - DESIRED_VAL) * KP;
    i_output += (roll - DESIRED_VAL) * KI;
    d_output =  (roll - DESIRED_VAL - rollprev + DESIRED_VAL) * KD;

    pid_output = p_output + i_output + d_output;
    if (abs(pid_output) > 200)
      pid_output = 200 * (abs(pid_output) / pid_output);
    
    leftval = map(pid_output, -200, 200, USEDMIN, USEDMAX);
    rightval = map(pid_output, -200, 200, USEDMIN, USEDMAX);
    */

    myPID.Compute();
     
    //Serial.print(" manual: ");
    //Serial.print(manual);

    // MANUAL CONTROL
    //leftval = map(manual, 0, 1024, USEDMIN, USEDMAX);
    //rightval = map(manual, 0, 1024, USEDMAX, USEDMIN);

    Serial.print(" Roll: ");
    Serial.print(roll); //Roll data in deg

    Serial.print(" output: ");
    Serial.print(Output);

    Serial.print(" setPoint: ");
    Serial.print(Setpoint);

    leftval = (USEDMIN + USEDMAX) / 2 - Output;
    rightval = (USEDMIN + USEDMAX) / 2 + Output;
    if (leftval > USEDMAX) leftval = USEDMAX;
    if (rightval > USEDMAX) rightval = USEDMAX;
    if (leftval < USEDMIN) leftval = USEDMIN;
    if (rightval < USEDMIN) rightval = USEDMIN;
    if (stopped)
      leftval = LEFTMIN;
    left.writeMicroseconds(leftval);
    if (stopped)
      rightval = RIGHTMIN;
    right.writeMicroseconds(rightval);

    /*
    Serial.print(" kp: ");
    Serial.print(myPID.GetKp());
    Serial.print(" ki: ");
    Serial.print(myPID.GetKi());
    Serial.print(" kd: ");
    Serial.print(myPID.GetKi());
    */

    
    Serial.print(" leftval: ");
    Serial.print(leftval);

    Serial.print(" rightval: ");
    Serial.print(rightval);
  
   
    
    Serial.println();
    updateSensorData = true;
  }
}
