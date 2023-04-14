
//getting data from esp 
//arduino requestFrom()
//ESP32 onRequest() 



#include <stdio.h>
#include <string.h>
#include <Wire.h>

//for receiver code
#include <esp_now.h>
#include <WiFi.h>

//receiver struct must match
typedef struct struct_message {                                             // THIS IS WHERE FALL_STATUS IS DEFINED
  bool fall_status;
  bool button_status;
} struct_message;

// Create a struct_message called myData
struct_message myData;


#define TRUE 1 
#define FALSE 0 

#define HIGH 1 
#define LOW 0



//////////////////////////////////////GPIO PINS FOR TOGGLING ARDUINO////////////////////////0//////////////////
#define WIRE1_PIN 18
#define WIRE2_PIN 19
#define WIRE3_PIN 21  //input only - read only for anchor (reading from Arduino)
bool wire2 = false; // Variable to act as WIRE2_PIN; the logic is switched, so the active HIGH is 'false'


/////////////////////////storing value from beginTransmission/////////////////////////////////////////
volatile char checkTransmission = 0; //before calling ranging loop (defined in library) 

void loop(void) {
       
  status_logic();


	}



void setup()
{
	Serial.begin(115200);
	Serial.println("Setup started.");

    //WIFI RECEIVING: 
    //set device as wifi station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);


///set up wire gpio pins to toggle Arduino
  pinMode(WIRE1_PIN, OUTPUT);
  //set to high initially because doing active low
  digitalWrite(WIRE1_PIN, HIGH);

  pinMode(WIRE2_PIN, OUTPUT); 
  digitalWrite(WIRE2_PIN, HIGH);

  //read from Arduino
  pinMode(WIRE3_PIN, INPUT);

}


//////////////////////////FUNCTIONS///////////////////////////

/////////RECEIVER CODE FOR DATA - CALL BACK FUNCTION///////
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
    //change this part 
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Fall status: ");
  Serial.println(myData.fall_status);
  Serial.print("Button status: ");
  Serial.println(myData.button_status);
  Serial.println();
  
}

void status_logic(void)
{
  if (myData.fall_status == true)
  { 
    //write wire 1 low to Arduino
    digitalWrite(WIRE1_PIN, LOW); 
    Serial.println("Fall occurred.");
    delay(50);
  }

  else if ((myData.button_status == true) && (wire2 == false) && (digitalRead(WIRE3_PIN) == HIGH ))
  {
    digitalWrite(WIRE1_PIN, LOW);
    Serial.println("Manual fall trigger.");
    delay(50);
  }

  else if ((myData.button_status == true) && (wire2 == true))
  {
    digitalWrite(WIRE1_PIN, HIGH);
    Serial.println("Manual fall clear.");
    delay(50);
  }

  else if (myData.button_status == true && digitalRead(WIRE3_PIN) == LOW)
  {
    wire2 = true;
    digitalWrite(WIRE2_PIN, LOW);
    Serial.println("Clear warning.");

    //wait for wire3 to turn high (acknowledgement from Arduino) and exit when it does turn high
    while (digitalRead(WIRE3_PIN) == LOW)
    { 
      //do nothing 
      Serial.println("Waiting for Arduino acknowledgement.");
      delay(500);
    }
    // received acknowledgement from arduino
    wire2 = false;
    digitalWrite(WIRE2_PIN, HIGH);
    Serial.println("Received Arduino acknowledgement.");
    delay(50);
  }

  else
  {
    //do nothing and just receive stuff 
    delay(50);
  }
}