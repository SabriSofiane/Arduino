#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>
#define SERIAL_BAUD 115200

void infos()
{
  esp_chip_info_t out_info;
  esp_chip_info(&out_info);
  Serial.print("CPU freq : "); 
  Serial.println(String(ESP.getCpuFreqMHz()) + " MHz");
  Serial.print("CPU cores : ");  
  Serial.println(String(out_info.cores));
  Serial.print("Flash size : ");
  Serial.println(String(ESP.getFlashChipSize() / 1000000) + " MB");
  Serial.print("Free RAM : ");
  Serial.println(String((long)ESP.getFreeHeap()) + " bytes");
  //Serial.print("Min. free seen : "); Serial.println(String((long)esp_get_minimum_free_heap_size()) + " bytes");
  Serial.print("tskIDLE_PRIORITY : "); 
  Serial.println(String((long)tskIDLE_PRIORITY));
  Serial.print("configMAX_PRIORITIES : ");
  Serial.println(String((long)configMAX_PRIORITIES));
  Serial.print("configTICK_RATE_HZ : "); 
  Serial.println(String(configTICK_RATE_HZ) + " Hz");
  Serial.println();
}

void vTask1( void *pvParameters )
{
  const char *pcTaskName = "Task 1 is running";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet( NULL );
  for( ;; )
  {
    Serial.printf("%s - core = %d (priorite %d)\n", pcTaskName, xPortGetCoreID(), uxPriority);
    //vTaskDelay( pdMS_TO_TICKS( 1000 ) );
     
  }
}

void vTask2( void *pvParameters )
{
  const char *pcTaskName = "Task 2 is running";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet( NULL );
  for( ;; )
  {
    Serial.printf("%s - core = %d (priorite %d)\n", pcTaskName, xPortGetCoreID(), uxPriority);
    //vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    
  }
}

void vTask3( void *pvParameters )
{
  const char *pcTaskName = "Task 3 is running";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet( NULL );
  for( ;; )
  {
    Serial.printf("%s - core = %d (priorite %d)\n", pcTaskName, xPortGetCoreID(), uxPriority);
    //vTaskDelay( pdMS_TO_TICKS( 1000 ) );
     
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Start");
  infos();
  //mutex
  SemaphoreHandle_t mutex = NULL;
  mutex = xSemaphoreCreateMutex();
  

  xTaskCreate(vTask1, "vTask1", 10000, NULL, 10, NULL);
  xTaskCreate(vTask2, "vTask2", 10000, NULL, 9, NULL);
  xTaskCreate(vTask3, "vTask3", 10000, NULL, 8, NULL);
}

void loop()
{
  const char *pcTaskName = "Main loop is running";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet( NULL );
  for( ;; )
  {
    Serial.printf("%s - core = %d (priorite %d)\n", pcTaskName, xPortGetCoreID(), uxPriority);
    //vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    
  }
}
