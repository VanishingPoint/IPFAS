#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);


//These determine characteristics of the warning check for vertical distance
#define CYCLES_PER_SECOND 10 //~10 observed
#define SECONDS_PER_PERIOD 30
#define PERIODS_TO_WARN 6

#define VERT_WARN_DISTANCE 3000 //in mm

//For Distance Warn
int warnStatus = 0;
int verticalWarnPeriods = 0;
int LatestVerticalPeriod = 0;


//For average level 1
#define WINDOW_SIZE 100

int INDEX = 0;
int VALUE = 0;
long int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;



//For average level 2
#define WINDOW_SIZE1 100

int INDEX1 = 0;
int VALUE1 = 0;
long int SUM1 = 0;
int READINGS1[WINDOW_SIZE1];
int AVERAGED1 = 0;



void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");

  //distanceSensor.setDistanceModeShort();
  distanceSensor.setDistanceModeLong();
  
}

void loop(void)
{

//Check  alert status from bluetooth service here

if (warnStatus != 0)
  warnactive(warnStatus);


for (int i=0; i<(CYCLES_PER_SECOND * SECONDS_PER_PERIOD); i++)  {
  
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement

  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();

 
 
  distanceSensor.stopRanging();

 byte rangeStatus = distanceSensor.getRangeStatus();

  if (rangeStatus == 0) { //Do not use poor range status values to compute average

//This computes a moving average (TE)
  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = distance;        // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result



//This computes a 2nd order rolling avg
  SUM1 = SUM1 - READINGS1[INDEX1];       // Remove the oldest entry from the sum
  VALUE1 = AVERAGED;        // Read the next sensor value
  READINGS1[INDEX1] = VALUE1;           // Add the newest reading to the window
  SUM1 = SUM1 + VALUE1;                 // Add the newest reading to the sum
  INDEX1 = (INDEX1+1) % WINDOW_SIZE1;   // Increment the index, and wrap to 0 if it exceeds the window size

  AVERAGED1 = SUM1 / WINDOW_SIZE1;      // Divide the sum of the window by the window size for the result

/*
  Serial.print(AVERAGED);
  Serial.print(", (mm) Avg");

  Serial.println();

  Serial.print("                      ");
  Serial.print(AVERAGED1);
  Serial.print(", (mm) Avg 2nd order");

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  Serial.println();
  */
  
}
}

//Code to determine warning here
if(AVERAGED1<VERT_WARN_DISTANCE) {
  LatestVerticalPeriod = 1;
  
  Serial.print("Vertical Warn Period, Recorded height ");
  Serial.print(AVERAGED1);
  Serial.println();
  
  if (verticalWarnPeriods<PERIODS_TO_WARN)
    verticalWarnPeriods++;
} else {
  LatestVerticalPeriod = 0;

  Serial.print("Vertical Height OK for period, Recorded height ");
  Serial.print(AVERAGED1);
  Serial.println();
  
  if (verticalWarnPeriods>0)
    verticalWarnPeriods--;
}
if (verticalWarnPeriods >= (PERIODS_TO_WARN - 1) && LatestVerticalPeriod == 1)
  warnStatus = 1;

}

//Area to write some functions

void warnactive(int warnType) {

  //Update the bluetooth service with the current value here
  
  if (warnType == 1)
   Serial.print("Vertical Minimums Warning");
  else if (warnType == 2)
    Serial.print("Angle Maximums Warning");
  else 
    Serial.print("Unknown Warning Status");

  while (warnStatus != -1){
    //check the warnstatus bluetooth
  }

warnStatus = 0; //Reset status to normal

//Push the new status to the bluetooth service

return;
  
}
