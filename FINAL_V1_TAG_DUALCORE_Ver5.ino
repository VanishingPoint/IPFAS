/////////TAG CODE WITH DUAL CORE//////////
//MAC ADDRESS: 40:22:D8:06:29:28 (broadcasting to this anchor)

#include <esp_now.h>
#include <WiFi.h>

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
 
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

#define TRUE 1
#define FALSE 0

//receiver MAC Address
uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0x06, 0x29, 0x28}; 
bool gstatus = 0;
typedef struct struct_message {                                             // THIS IS WHERE FALL_STATUS and BUTTON_STATUS ARE DEFINED
  bool fall_status = false;
  bool button_status = false;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

////////////////////////////accelerometer stuff//////////////////////// 
// x, y, z input pins
const int X_PIN = 36, 
          Y_PIN = 35,
          Z_PIN = 39;

// Button input pins
const int B1_PIN = 25,
          B2_PIN = 26;

// Constant quantities:
const float ADC_PER_G = 100.0; // This one is roughly the same for all
const float ADC_AT_0G_X = 4.70,
            ADC_AT_0G_Y = 4.70,
            ADC_AT_0G_Z = 4.70;
// These quantities can be adjusted when calibrating
const int SAMPLING_RATE = 500; // In ms
const float LOWER_BOUND = 0.50; // In g's
const float UPPER_BOUND = 1.75;
const float TRIGGER_DIFF = 0.50; // Change this (in g's) to change the sensitivity of the accelerometer
const float WAIT_TIME = 5000; // In ms
const float BUTTON_TIME = 3000; // In ms

// Analog to 10-bit input values 
int x_in = 0,        
    y_in = 0,
    z_in = 0;

// 10-bit values converted to g's
float g_x = 0,
      g_y = 0,
      g_z = 0;

// Magnitude of acceleration
float a = 0;

// Status of the fall
// bool fall_status; // This variable is defined above in the structure

// Test variables
float a_before = 0;

// Button variables
bool b1 = LOW,
     b2 = LOW;

//////////////////////////////DW1000 stuff////////////////////////
// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin
 
// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

// Function declarations
void get_accel(void);
void convert_gs(void);
float calc_mag(void);
void print_data(void);
void read_buttons(void);
void send_status_procedure(void);

TaskHandle_t Task0;
TaskHandle_t Task1;

void loop0(void * parameter) {
	for (;;) {
		//Serial.print("Running on core: ");
		//Serial.println(xPortGetCoreID());

    DW1000Ranging.loop();
	}
}


void loop1(void * parameter)
{
	for (;;)
	{
    // Read buttons
    read_buttons();
    

    // Get the values in g's from the accelerometer 
    get_accel(); // Read the input
    convert_gs(); // Convert to g's
    a = calc_mag(); // Calculate the magnitude


    // This conditional checks for low accelerations, which are in theory akin to the start of a fall
    if(a < LOWER_BOUND)
    {
      // This variable is used for comparison across time
      a_before = a;

      // Wait the regular amount of time between readings
      delay(SAMPLING_RATE);

      // This loop sets a time to check for sudden increases in acceleration, which are in theory akin to the end of a fall
      // Here, the counter is inversely proportional to the sampling rate. Example: for sampling rate = 0.5s, there are 2 counts per second
      for(int i = 0; i < WAIT_TIME/SAMPLING_RATE; i++)
      {
        // Keep reading acceleration
        get_accel();
        convert_gs();
        a = calc_mag();
        read_buttons(); // Need to have the button function to read them constantly
  
        // This conditional checks for a sudden change in acceleration
        if(a - a_before > TRIGGER_DIFF)
        {
          // If the conditional evaluates to false, then the fall status is changed and the loop breaks out and keeps reading stuff
          myData.fall_status = true;
          print_data();
          break;
        }
  
        // While the conditional is not true, keep printing out the data and maintain the loop as usual
        a_before = a;
        print_data();
        send_status_procedure();
        delay(SAMPLING_RATE);
      }
      // The conditional ends; reset a_before
      a_before = 0;
    }
    
    
    // This conditional will simply detect very high accelerations and will send out an alarm
    if(a > UPPER_BOUND)
    {
        myData.fall_status = true; 
    }


    // If none of the conditionals were triggered, just keep the loop going
    print_data();
    send_status_procedure();
    delay(SAMPLING_RATE);
  }



}


void setup()
{
	Serial.begin(115200);
	Serial.println("Setup started.");

  //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    
    // start as tag, do not assign random short address
    
    DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);


   // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
    

    /*** Our code ***/
    // Initialise button pins
    pinMode(B1_PIN, INPUT);
    pinMode(B2_PIN, INPUT);
    
    // Columns of data
//    Serial.print("Magnitude in g's");
//    Serial.print("\t\tFall detection status");
//    Serial.println("\t\tButton status");
    
    // This command changes the resolution of the analog input pins to 10 bits instead of the default 12
    analogReadResolution(10);
    
    delay(50);
	xTaskCreatePinnedToCore(
			loop0, /* Function to implement the task */
			"Task0", /* Name of the task */
			10000, /* Stack size in words */
			NULL, /* Task input parameter */
			0, /* Priority of the task */
			&Task0, /* Task handle. */
			0); /* Core where the task should run */

	xTaskCreatePinnedToCore(
			loop1, /* Function to implement the task */
			"Task1", /* Name of the task */
			10000, /* Stack size in words */
			NULL, /* Task input parameter */
			0, /* Priority of the task */
			&Task1, /* Task handle. */
			1); /* Core where the task should run */
	Serial.println("Setup completed.");

}

void loop()
{
	delay(1);
}





////////////////////////////////FUNCTIONS - wifi //////////////////////////
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Loop 1: ");
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  // TEST: LET US PRINT THE CONTENTS OF ESP_NOW_SEND_SUCCESS AND status
  Serial.print("Contents of 'status': ");
  Serial.println(status);
  Serial.print("Contents of 'ESP_NOW_SEND_SUCCESS': ");
  Serial.println(ESP_NOW_SEND_SUCCESS);
  // End of test

  if (status == ESP_NOW_SEND_SUCCESS) {                                                 // THIS IS WHERE THIS SHIT IS GOING WRONG
    Serial.println("Sent with success");
    myData.fall_status = false;
    myData.button_status = false;
  }
  else {
    Serial.println("Error sending the data");
  }
}


////////////////////////////////FUNCTIONS - DW1000//////////////////////////
void newRange()
{
  Serial.print("Loop 0: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX); //should be "84, distance" 
  Serial.print(",");
  Serial.println(DW1000Ranging.getDistantDevice()->getRange());
}


void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

 
void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}


/////////////////////Function definitions - accelerometer/////////////
void get_accel(void)
{
  // Read the analog in value:
  x_in = analogRead(X_PIN);
  delay(2);
  y_in = analogRead(Y_PIN);
  delay(2);
  z_in = analogRead(Z_PIN);
  delay(2);
  return;
}


void convert_gs(void)
{
  /*  Convert 10-bit values to g's. On average, and in the Arduino, there
   *  is a change of about 100 ADC per change of 1g and real 0g occurs at
   *  about 500 ADC, so the y-intercept should be 500/100 = 5. I still
   *  need to find out what the actual change in the voltage per g in the
   *  ESP is, but we can worry about fine calibration later as everything
   *  is so far a rough estimate.
   */
  g_x = (-1 * x_in / ADC_PER_G) + ADC_AT_0G_X;
  g_y = (-1 * y_in / ADC_PER_G) + ADC_AT_0G_Y;
  g_z = (-1 * z_in / ADC_PER_G) + ADC_AT_0G_Z;
  return;
}


float calc_mag(void)
{
  return sqrt(sq(g_x) + sq(g_y) + sq(g_z));
}


void print_data(void)
{
  // Prints the acceleration out as well as the status
  Serial.print("Magnitude in g's: ");
  Serial.print(a);
  Serial.print("\t\tFall status: ");
  Serial.print(myData.fall_status);
  Serial.print("\t\tButton status: ");
  Serial.println(myData.button_status);
  return;
}


void read_buttons(void)
{
  // Read the buttons. If they are both true, set the button status to true
  // IMPORTANT: For the buttons to be read, they should be left pressed as 
  if(digitalRead(B1_PIN) == HIGH && digitalRead(B2_PIN) == HIGH)
  {
    // Set the button status to true
    myData.button_status = true;
  }
  return;
}


// New function defined by us that takes care of the stuff inside the his main loop
//reset function doesn't work
void send_status_procedure(void)
{
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  return;
}