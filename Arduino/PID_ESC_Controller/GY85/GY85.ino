#include "GY_85.h"
#include <Wire.h>

GY_85 GY85;     //create the object

void setup()
{
    Wire.begin();
    delay(10);
    Serial.begin(1000000);
    delay(10);
    GY85.init();
    delay(10);
}


void loop()
{
    int* acc = GY85.readFromAccelerometer();
    int ax = GY85.accelerometer_x( acc );
    int ay = GY85.accelerometer_y( acc );
    int az = GY85.accelerometer_z( acc );

    /*
    int* comp = GY85.readFromCompass();
    int cx = GY85.compass_x( comp );
    int cy = GY85.compass_y( comp );
    int cz = GY85.compass_z( comp );
    */

    float* gyr = GY85.readGyro();
    float gx = GY85.gyro_x( gyr );
    float gy = GY85.gyro_y( gyr );
    float gz = GY85.gyro_z( gyr );
    float gt = GY85.temp  ( gyr );
    
    /*
    //Serial.print  ( "acc" );
    Serial.print  ( " ax:" );
    Serial.print  ( ax );
    Serial.print  ( " ay:" );
    Serial.print  ( ay );
    Serial.print  ( " az:" );
    Serial.print  ( az );

    Serial.print  ( "  cp" );
    Serial.print  ( " x:" );
    Serial.print  ( cx );
    Serial.print  ( " y:" );
    Serial.print  ( cy );
    Serial.print  (" z:");
    Serial.print  ( cz );
    */
    
    //Serial.print  ( "  gyr" );
    Serial.print  ( " gx:" );
    Serial.print  ( gy );
    Serial.print  ( " gy:" );
    Serial.print  ( gx );
    Serial.print  ( " gz:" );
    Serial.print  ( gz );
    Serial.print  ( " gt:" );
    Serial.println( gt );
   
    delay(5);             // only read every 0,5 seconds, 10ms for 100Hz, 20ms for 50Hz
}
