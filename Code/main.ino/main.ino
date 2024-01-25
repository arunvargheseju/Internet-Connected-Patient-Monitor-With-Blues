#include <Notecard.h>
#include <STM32FreeRTOS.h>
#include <Wire.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include<HttpClient.h>
#include "DFRobot_BloodOxygen_S.h"

#define I2C_ADDRESS    0x57
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);

#define myProductID "com.gmail.arunvargheseju2017:medicaliot"
Notecard notecard;
HardwareSerial Serial2(USART2); // XIAO ESP32 C3


// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define three Tasks for DigitalRead & AnalogRead
void Pulse_oximeter( void *pvParameters );
void ECG( void *pvParameters );
void SerialRead(void *pvParameters);

int32_t spo2; //SPO2 value
int32_t heartRate; //heart rate value
float temperature;

bool flag = false; 
char receivedData = 0;


// the setup function runs once when you press reset or power the board
void setup() 
{
  Serial.begin(9600);
  Serial2.begin(9600);
  
  
  //notecard.setDebugOutputStream(Serial);
  notecard.begin();

  J *restore = notecard.newRequest("card.restore");
  JAddBoolToObject(restore, "delete", true);
  notecard.sendRequest(restore);
  delay(5000);
  
  J *req = notecard.newRequest("hub.set");
  if (myProductID)
  {
      JAddStringToObject(req, "product", myProductID);
  }

  JAddStringToObject(req, "mode", "continuous");
  notecard.sendRequestWithRetry(req, 5);

  delay(50000);
  
  //pinMode(PA0,INPUT);
  
  while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }
  
  Serial.println("Init success!");
  MAX30102.sensorStartCollect();

  /* Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
   because it is sharing a resource, such as the Serial port.
   Semaphores should only be used whilst the scheduler is running, but we can set it up here. */
  
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up three Tasks to run independently.
  xTaskCreate(
    Pulse_oximeter
    ,  (const portCHAR *)"PulseOximeter"  // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    ECG
    ,  (const portCHAR *) "E.C.G"
    ,  1024  // Stack size
    ,  NULL
    ,  3 // Priority
    ,  NULL );
    
  xTaskCreate(
  SerialRead,
  (const portCHAR *)"XIAO",
  128,
  NULL,
  1,  // Priority (higher than the other tasks)
  NULL  );


  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
  
}

void loop()
{
  
}



void Pulse_oximeter( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;) // A Task shall never return or exit.
  {  
       MAX30102.getHeartbeatSPO2();
        vTaskDelay(10);
        spo2 = MAX30102._sHeartbeatSPO2.SPO2;
        vTaskDelay(10);
        heartRate = MAX30102._sHeartbeatSPO2.Heartbeat;
        vTaskDelay(10);
        temperature = MAX30102.getTemperature_C();
        temperature = temperature*(1.8) + 32;
        vTaskDelay(10);
        
        // Sending the data as JSON
        if (heartRate<60)
        {
          doctorAlert();
        }
        if (spo2<80)
        {
          doctorAlert();
        }        
    
   // xSemaphore is used for serial debugging only uncomment if necessary 
   
   /*if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 10 ) == pdTRUE )
    {
        Serial.print("temperatureC=");
        Serial.print(temperature, 4);
        Serial.println();
        
        Serial.print("HR=");
        Serial.print(heartRate, DEC);
        Serial.println();
        
        Serial.print("SPO2=");
        Serial.print(spo2, DEC);
        Serial.println();
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    } */

    vTaskDelay(60);  // one tick delay (15ms) in between reads for stability
  }
  
}

void ECG( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  { 
    //client.loop();
    // read the input on analog pin 0:
    int ECGValue = analogRead(PA0);
    
    J *req = notecard.newRequest("note.add");
    if (req != NULL)
    {
        JAddBoolToObject(req, "sync", true);
        J *body = JAddObjectToObject(req, "body");
        if (body != NULL)
        {
            JAddNumberToObject(body, "ECG", ECGValue);
            JAddNumberToObject(body, "Temperature", temperature);
            JAddNumberToObject(body, "HeartRate", heartRate);
            JAddNumberToObject(body, "SPO2", spo2);
        }
        notecard.sendRequest(req);
    }

    /*If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free
      So uncomment if necessary*/
     
    if (xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      //Serial.println("Published on topic: " + topic2);
      Serial.println(ECGValue);
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(50);  // one tick delay (15ms) in between reads for stability
  }
}
void SerialRead(void *pvParameters)
{
  for (;;)
  {
        
    // Read the data from the serial port (assuming data is in ASCII)
    // and convert it to an integer.
    if(Serial2.available() > 0)
   {
    char receivedData = Serial2.read();
    if(receivedData == 'C')
    {   
       /*If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free
        So uncomment if necessary*/
       if (xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE)
        {
               Serial.println(receivedData);
               Serial.println("Alert");
               nurseAlert();
               xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
        
        }
    }
    else if(receivedData == 'A')
    {
          Serial.println("Anomaly");
          nurseAlert();
      
    }
  }
  vTaskDelay(50); // Adjust the delay time as needed.
}
}


void nurseAlert()
{
    J *req = notecard.newRequest("note.add");
    if (req != NULL)
    {   
        JAddStringToObject(req, "file", "nurseAlert.qo");
        JAddBoolToObject(req, "sync", true);
        J *body = JAddObjectToObject(req, "body");
        if (body != NULL)
        {
            JAddNumberToObject(body, "nurseAlert", 1);
        }
        notecard.sendRequest(req);
  }
}


void doctorAlert()
{   

    J *req = notecard.newRequest("note.add");
    if (req != NULL)
    {    
        JAddStringToObject(req, "file", "doctorAlert.qo");
        JAddBoolToObject(req, "sync", true);
        J *body = JAddObjectToObject(req, "body");
        if (body != NULL)
        {
            JAddNumberToObject(body, "doctorAlert", 1);
        }
        notecard.sendRequest(req);
   }
}
