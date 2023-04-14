#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include <math.h>
#include <ArduinoBLE.h>

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

//These determine characteristics of the warning check for vertical distance
#define CYCLES_PER_SECOND 10 //~10 observed - shorter is better to demo
#define SECONDS_PER_PERIOD 5
#define PERIODS_TO_WARN 6  //CHANGED FOR TEST

int warnStatus = 0;

#define VERT_WARN_DISTANCE 30 //in mm

//For Distance Warn
int verticalWarnPeriods = 0;
int LatestVerticalPeriod = 0;

//For Horizontal Warn
bool anchorOn = 0; //0 will be top anchor, 1 will be bottom anchor

//These define characteristics of the horizontal warning
#define SEPERATION 300 //in mm
#define BOUNDRY_DEGREES 22

//I2C address definitions for anchors
#define TOP_ANCHOR_ADDRESS 7
#define BOTTOM_ANCHOR_ADDRESS 8

//global variables for horizontal warning
float boundryRad = (M_PI * BOUNDRY_DEGREES)/180;
float topAnchorDist;
float bottomAnchorDist;
float currentAngleRad;
int LatestAnglePeriod = 0;
int angleWarnPeriods = 0;


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

//Fall Alert ISR Config
const int ISRpin = 9; //SPECSHEET LIES!! MUST BE A SPECIFIC PIN, PIN 10 OK!!!!
bool alertflag = 0;

//Other Tag -> Anchor <-> Arduino comm pins
const int Datapin2 = 10;
const int Datapin3 = 11;

//Bluetooth parameters
  // create a service to expose our service to BLE Central Devices
  BLEService WarnService("FA01");

  BLEUnsignedCharCharacteristic WarnCharacteristic("2102", BLERead | BLEWrite | BLENotify);

  //SANTIS METHOD TEST
 int count = 0;

void setup(void)
{

  Serial.begin(9600);
  while(!Serial);

  //Config bluetooth, including advertisement

if (!BLE.begin()) 
  {
    Serial.println("starting BLE failed!");
    while (1);
  }

  String address = BLE.address();
  Serial.println("Our address is [" + address + "]");

  BLE.setDeviceName("IPFAS");      // this sets Characteristic 0x2a00 of Service 0x1800
                                               // Service 0x1800 is the Generic Access Profile
                                               // Characteristic 0x2a00 is the Device Name
                                               // Characteristic 0x2a01 is the "Appearance"
  BLE.setAppearance(384);                      // BLE_APPEARANCE_GENERIC_REMOTE_CONTROL
                                               
  BLE.setLocalName("BLE IPFAS Interface");       // this sets the local name for the advertising data
  
  // tell the world about us
  BLE.setAdvertisedService(WarnService);
  WarnService.addCharacteristic(WarnCharacteristic);
  BLE.addService(WarnService);

  WarnCharacteristic.writeValue((byte)0x00);      // start with a zero

  // advertise to the world so we can be found
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  // register new connection handler
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  
  // register disconnect handler
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

//Bluetooth config end

  Wire.begin();
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

  switchAnchorTo(anchorOn); //sets to top anchor

//Config Alert ISR
  pinMode(ISRpin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ISRpin), Alert_ISR, CHANGE);

  //Ensure wire 3 is high (off)
  pinMode(Datapin3, OUTPUT);
  digitalWrite(Datapin3, HIGH);

  //Make pin 2 an input
  pinMode(Datapin2, INPUT_PULLUP);


}

//IMPORTANT - if main loop takes too long to run, BLE will not work!
void loop(void)
{

 BLEDevice central = BLE.central();

if (central) {
  
  while (central.connected()) {
    
if (alertflag == 1)
  alert();

if (warnStatus != 0)
  warnactive(warnStatus);
  
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

//This computes a moving average
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
  
}

if (count == (CYCLES_PER_SECOND * SECONDS_PER_PERIOD)/2) {

  //read the current anchor value to the appropriate variable
  readAnchorVal(anchorOn); 
}

if (count == (CYCLES_PER_SECOND * SECONDS_PER_PERIOD)) {

//Implied triangle Code here
readAnchorVal(anchorOn); 

//check the warning status
calculateHorizontalWarn();
calculateVerticalWarn();

//note that these will trigger the warning behaviour at the beginning of the next main loop

count = 0;
  }
  
  

count++;
  }}
} //end of main loop



//Area to write some functions

void calculateVerticalWarn(void) {

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

return;
}

void calculateHorizontalWarn (void) {

  //input conditioning
  if (topAnchorDist < 0)
    topAnchorDist = topAnchorDist * -1;
  if (bottomAnchorDist < 0)
    bottomAnchorDist = bottomAnchorDist * -1;

  currentAngleRad = acos(topAnchorDist/SEPERATION); //DAID modification


  Serial.print("Top Anchor Distance: ");
  Serial.println(topAnchorDist);
  //Serial.print("Bottom Anchor Distance: ");
 // Serial.println(SEPERATION);
 Serial.println("Demo - Bottom Anchor Disconnected, Distance Inferred");

  //Calculate current angle of user, measured in radians from straight down the lifeline
 // currentAngleRad = get_angle(bottomAnchorDist, topAnchorDist, SEPERATION);

  //Condition Output
  if (currentAngleRad < 0)
    currentAngleRad = currentAngleRad * -1;

  Serial.print(",   Computed angle (rad): ");
  Serial.println(currentAngleRad);

  //Determine warn status
  if(currentAngleRad>boundryRad) {
   LatestAnglePeriod = 1;
   Serial.println("Horizontal Warn Period");
   if (angleWarnPeriods<PERIODS_TO_WARN)
     angleWarnPeriods++;
  } else {
   LatestAnglePeriod = 0;
   Serial.println("Horizontal Distance OK");
   if (angleWarnPeriods>0)
      angleWarnPeriods--;
  }
  
  if (angleWarnPeriods >= (PERIODS_TO_WARN - 1) && LatestAnglePeriod == 1)
   warnStatus = 2;

  return;
 }


void readAnchorVal(bool anchor){

//  String wholeData = "";
  char wholeData[4];
  int i = 0;
  volatile bool received = 0;

//request data from the correct anchor
  if (anchor == 0) {
     Wire.requestFrom(TOP_ANCHOR_ADDRESS, 4);
   } else if (anchor == 1) {
     Wire.requestFrom(BOTTOM_ANCHOR_ADDRESS, 4);
   }

   //recieve all data and sequentially add to string
   while (Wire.available())
       {
      char inChr = Wire.read();
//      wholeData += inChr;
        wholeData[i] = inChr;
        i++;
   }
   
   //convert to float and send to correct variable
    if (anchor == 0)
//      topAnchorDist = atof((wholeData).c_str());
        topAnchorDist = atof(wholeData);
    else if (anchor == 1)
 //     bottomAnchorDist = atof((wholeData).c_str());
        bottomAnchorDist = atof(wholeData);
  return;
}


void switchAnchorTo (bool anchor){
  if (anchor == 0){

  //Turn on top anchor
  Wire.beginTransmission(TOP_ANCHOR_ADDRESS);
  Wire.write("1");
  Wire.endTransmission();

  //Turn off bottom anchor
  Wire.beginTransmission(BOTTOM_ANCHOR_ADDRESS);
  Wire.write("0");
  Wire.endTransmission();

  //set anchorOn
  anchorOn = 0;
  
  //Print status
  Serial.print("Enabled Top Anchor, Disabled Bottom Anchor");
  Serial.println();
    
  }  else if (anchor == 1){

  //Turn on top anchor
  Wire.beginTransmission(BOTTOM_ANCHOR_ADDRESS);
  Wire.write("1");
  Wire.endTransmission();

  //Turn off bottom anchor
  Wire.beginTransmission(TOP_ANCHOR_ADDRESS);
  Wire.write("0");
  Wire.endTransmission();

  //set anchorOn
  anchorOn = 1;

  //Print status
  Serial.print("Enabled Bottom Anchor, Disabled Top Anchor");
  Serial.println();
    
  } else {
  //Turn off both anchors

  //Turn off top anchor
  Wire.beginTransmission(TOP_ANCHOR_ADDRESS);
  Wire.write("0");
  Wire.endTransmission();

  //Turn off bottom anchor
  Wire.beginTransmission(BOTTOM_ANCHOR_ADDRESS);
  Wire.write("0");
  Wire.endTransmission();

  //Print status
  Serial.print("Both Anchors Disabled - Error Case Detected");
  Serial.println();
  }

  return;
}

void warnactive(int warnType) {

  //Inform the Tag throughput Anchor of the status
  digitalWrite(Datapin3, LOW);

  //print to console and update bluetooth status
  if (warnType == 1){
   Serial.println("Vertical Minimums Warning");
   WarnCharacteristic.writeValue((byte)1);
  }
  else if (warnType == 2) {
    Serial.println("Angle Maximums Warning");
    WarnCharacteristic.writeValue((byte)2);
  }
  else 
    Serial.println("Unknown Warning Status");

  while (digitalRead(Datapin2) != LOW ){
    //check the data cables for the buttonpress
  }

  digitalWrite(Datapin3, HIGH);

warnStatus = 0; //Reset status to normal
angleWarnPeriods = 0;
verticalWarnPeriods = 0;
//Push the new status to the bluetooth service here
WarnCharacteristic.writeValue((byte)0);

return;
  
}

void Alert_ISR()
{
  Serial.println("ISR TRIGGED");
  alertflag = 1;

}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  digitalWrite(LED_BUILTIN, HIGH);    // indicate that we have a connection

}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  digitalWrite(LED_BUILTIN, LOW);     // indicate that we no longer have a connection
}

float get_angle(float ra, float rb, float rc)
{

//!! rb needs to be the topAnchorDist
//ra = bottomAnchorDist
  
  // Variables that will be sorted by size
  float a = 0,
        b = 0,
        c = 0;

  // Sort the variables from largest to smallest
  if(ra >= rb)
  {
    if(rb >= rc)
    {
      a = ra;
      b = rb;
      c = rc;
    }
    else if(ra >= rc)
    {
      a = ra;
      b = rc;
      c = rb;
    }
    else
    {
      a = rc;
      b = ra;
      c = rb;
    }
  }
  else
  {
    if(ra >= rc)
    {
      a = rb;
      b = ra;
      c = rc;
    }
    else if(rb >= rc)
    {
      a = rb;
      b = rc;
      c = ra;
    }
    else
    {
      a = rc;
      b = rb;
      c = ra;
    }
  }

  return asin((1/(2*rb*rc))*sqrt((a+(b+c))*(c-(a-b))*(c+(a-b))*(a+(b-c))));
}

void alert (void) {

if (digitalRead(ISRpin) != HIGH){
  
    //push alert status to bluetooth here
  WarnCharacteristic.writeValue((byte)3);
  Serial.println("FALL DETECTED");
while(digitalRead(ISRpin) != HIGH) {
  //loop until goes high
   }
//push clear status to bluetooth
  WarnCharacteristic.writeValue((byte)0);
  Serial.println("FALL CLEARED");
  }
alertflag = 0;
}
