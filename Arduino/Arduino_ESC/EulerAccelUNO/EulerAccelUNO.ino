#include "NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <Servo.h> 

Servo left;
Servo right;

NineAxesMotion mySensor;         //Object that for the sensor 
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 15;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;
/*
float accx;
float accy;
float accz;
float linx;
float liny;
float linz;
float gravx;
float gravy;
float gravz;
*/
float heading;
float roll;
float pitch;
float leftval = 0;
float rightval = 0;


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
  Serial.println("Starting ESCs Calibration");
  left.attach(6, 1000, 2000);  // PWM Pin connnection for left Motor
  right.attach(5, 1000, 2000); // PWM Pin connnection for right Motor
  delay(1);
  // DO NOT CHANGE! Required to activate ESC
  left.write(15);
  right.write(15);
  // Mandatory Wait for ESC to be armed Correctly 
  delay(6000);
  Serial.println("Finished Calibration"); 
}

void loop() //This code is looped forever
{
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

    // Serial.print("t: ");
    // Serial.print(lastStreamTime); //Time elapsed since beginning in ms
    /*
    accx = mySensor.readAccelerometer(X_AXIS);
    accy = mySensor.readAccelerometer(Y_AXIS);
    accz = mySensor.readAccelerometer(Z_AXIS);
   
    linx = mySensor.readLinearAcceleration(X_AXIS);
    liny = mySensor.readLinearAcceleration(Y_AXIS);
    linz = mySensor.readLinearAcceleration(Z_AXIS);
    gravx = mySensor.readGravAcceleration(Z_AXIS);
    gravy = mySensor.readGravAcceleration(Z_AXIS);
    gravz = mySensor.readGravAcceleration(Z_AXIS);
    */
    pitch = mySensor.readEulerPitch();
    heading = mySensor.readEulerHeading();
    roll = mySensor.readEulerRoll();

    /*
    Serial.print(" accX: ");
    Serial.print(accx); //Accelerometer X-Axis data in m/s2

    Serial.print(" accY: ");
    Serial.print(accy);  //Accelerometer Y-Axis data in m/s2

    Serial.print(" accZ: ");
    Serial.print(accz);  //Accelerometer Z-Axis data in m/s2
      
   
    Serial.print(" gravX: ");
    Serial.print(gravx); //Gravity Acceleration X-Axis data in m/s2

    Serial.print(" gravY: ");
    Serial.print(gravy);  //Gravity Acceleration Y-Axis data in m/s2

    Serial.print(" grayZ: ");
    Serial.print(gravz);  //Gravity Acceleration Z-Axis data in m/s2
    

    Serial.print(" Heading: ");
    Serial.print(heading); //Heading data in deg

    Serial.print(" Pitch: ");
    Serial.print(pitch); //Pitch data in deg

    Serial.print(heading); //Heading data in deg
    Serial.print(';');
    Serial.print(pitch); //Pitch data in deg
    Serial.print(';');
    Serial.print(roll); //Roll data in deg
*/

    Serial.print(" Roll: ");
    Serial.print(roll); //Roll data in deg

    leftval = map(roll, -30 , 30, 50, 20);
    rightval = map(roll, -30 ,30, 20, 50);
    left.write(leftval);
    right.write(rightval);

    Serial.println();
    updateSensorData = true;
  }
}
