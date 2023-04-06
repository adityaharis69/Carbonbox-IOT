#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <math.h>

const char *mqtt_server = "test.mosquitto.org";
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

QueueHandle_t xQueue1;
QueueHandle_t xQueue2;
QueueHandle_t xQueue3;
QueueHandle_t xQueue4;
struct toppic
{
  /* data */
  const char *MQTT_PUB_CO2input = "esp32/mh-z14a/input";
  const char *MQTT_PUB_CO2output = "esp32/mh-z14a/output";
  const char *MQTT_PUB_ph = "esp32/dfRobot/ph";
  const char *MQTT_PUB_tbd = "esp32/dfRobot/tbd";
  const char *MQTT_PUB_AirTemp = "esp32/sht3x/temp";
  const char *MQTT_PUB_rh = "esp32/sht3x/rh";
  const char *MQTT_PUB_WatherTemp = "esp32/ds18b20/temp";
};

#define co2InputPin GPIO_NUM_33   // co2 input
#define co2OutputPin GPIO_NUM_32  // co2 output
#define phPin GPIO_NUM_34         // ph
#define watherTemPin GPIO_NUM_19  // pin DS18B20
#define turbidityPin GPIO_NUM_35  // pin turbidity
#define waterLevelPin GPIO_NUM_4  // pin Water level
#define relayPompaPin GPIO_NUM_12 // pinPompa

const char *ssid = "Neurabot";
const char *password = "Kempul4321!";

#define WIFI_TIMEOUT_MS 20000      // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt

#define Offset 0.00 // deviation compensate
#define samplingInterval 20
#define ArrayLenth 40 // times of collection

static TaskHandle_t taskWifi;
static TaskHandle_t taskco2In;
static TaskHandle_t taskco2Out;
static TaskHandle_t tasksht13;
static TaskStatus_t tasktbd;
static TaskHandle_t taskDals11;
static TaskHandle_t taskph;
static TaskHandle_t taskrecieve;

struct dataSensor
{
  /* data */
  int co2_Input_C;
  int co2_Output_C;
  float air_Temperature_C;
  float air_Humidity_C;
  float ph_C;
  float wather_Temperature_C;
  float tbd_C;
};

float ph_value;
float wather_Temp, air_temp, air_humidity, turbidity_ntu;

int adc = 0;
float volt;

// float readTbd()
// {
//   int adc5;
//   adc = analogRead(turbidityPin);
//   adc5 = map(adc, 0, 4095, 0, 1023);
//   volt = 0;
//   for (int i = 0; i < 1000; i++)
//   {
//     volt += ((float)adc5 / 1023.0) * 5.0;
//   }
//   volt = (volt / 1000); //-0.159
//   volt = round_to_dp(volt, 2);
//   if (volt < 2.5)
//   {
//     turbidity_ntu = 3000;
//   }
//   else
//   {
//     turbidity_ntu = -1120.4 * square(volt) + 5742.3 * volt - 4353.8;
//     //    Serial.println(turbidity_ntu);
//     if (turbidity_ntu < 0)
//     {
//       turbidity_ntu = 0.0;
//       return turbidity_ntu;
//     }
//     else
//     {
//       turbidity_ntu = turbidity_ntu;
//       return turbidity_ntu;
//     }
//   }

//   delay(1000);
// }

// void sensor_turbidity_task(void *Parameters)
// {

//   char myChar[10];
//   const char *turbidity_value;

//   while (true)
//   {
//     //    Serial.println(readTbd());
//     dtostrf(readTbd(), 6, 2, myChar);
//     turbidity_value = myChar;
//     if (!client.connected())
//     {
//       reconnect(7, turbidity_value);
//     }
//     client.publish(MQTT_PUB_tbd, turbidity_value);
//     vTaskDelay(pdMS_TO_TICKS(3000));
//   }
// }

// float round_to_dp(float in_value, int decimal_place)
// {
//   float multiplier = powf(10.0f, decimal_place);
//   in_value = roundf(in_value * multiplier) / multiplier;
//   return in_value;
// }

// float square(float f)
// {
//   (abs(f) < 256.0);
//   return f * f; // check if overflow will occur because 256*256 does not fit in an int anymore
// }

void sensor_sht3x_task(void *Parameters)
{
  Adafruit_SHT31 sht31 = Adafruit_SHT31();
  dataSensor dataSensor;
  if (!sht31.begin(0x44))
  {
    Serial.println("Check circuit. SHT31 not found!");
    while (1)
      delay(1);
  }
  while (true)
  {
    dataSensor.air_Humidity_C = sht31.readHumidity();
    dataSensor.air_Temperature_C = sht31.readTemperature();

    xQueueSend(xQueue3, &dataSensor.air_Temperature_C, portMAX_DELAY);
    xQueueSend(xQueue3, &dataSensor.air_Humidity_C, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// void callback(char *topic, byte *payload, unsigned int length)
// {
//   Serial.print("Message arrived [");
//   Serial.print(topic);
//   Serial.print("] ");
//   for (int i = 0; i < length; i++)
//   {
//     Serial.print((char)payload[i]);
//   }
//   Serial.println();
// }

// void reconnect(int number, const char *valueSensor)
// {
//   // Loop until we're reconnected
//   while (!client.connected())
//   {
//     Serial.print("Attempting MQTT connection...");
//     // Create a random client ID
//     String clientId = "ESP32Client-";
//     clientId += String(random(0xffff), HEX);
//     // Attempt to connect
//     if (client.connect(clientId.c_str()))
//     {
//       if (number == 1)
//       {
//         client.publish(MQTT_PUB_CO2input, valueSensor);
//       }
//       else if (number == 2)
//       {
//         client.publish(MQTT_PUB_CO2output, valueSensor);
//       }
//       else if (number == 3)
//       {
//         client.publish(MQTT_PUB_ph, valueSensor);
//       }
//       else if (number == 4)
//       {
//         client.publish(MQTT_PUB_WatherTemp, valueSensor);
//       }
//       else if (number == 5)
//       {
//         client.publish(MQTT_PUB_AirTemp, valueSensor);
//       }
//       else if (number == 6)
//       {
//         client.publish(MQTT_PUB_rh, valueSensor);
//       }
//       else if (number == 7)
//       {
//         client.publish(MQTT_PUB_tbd, valueSensor);
//       }
//     }
//     else
//     {
//       Serial.print("failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" try again in 5 seconds");
//       delay(5000);
//     }
//   }
// }

void sensor_watherTemp_task(void *Parameters)
{
  dataSensor dataSensor;
  OneWire oneWire(watherTemPin);
  DallasTemperature sensors(&oneWire);
  sensors.begin();

  while (true)
  {
    sensors.requestTemperatures();
    dataSensor.wather_Temperature_C = sensors.getTempCByIndex(0);
    xQueueSend(xQueue4, &dataSensor.wather_Temperature_C, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// void sensor_ph_task(void *Parameter)
// {
//   int pHArray[ArrayLenth]; // Store the average value of the sensor feedback
//   int pHArrayIndex = 0;
//   static float voltage;
//   char myChar[10];
//   const char *pHValue;
//   static unsigned long samplingTime = millis();
//   static unsigned long printTime = millis();
//   while (true)
//   {
//     if (millis() - samplingTime > samplingInterval)
//     {
//       pHArray[pHArrayIndex++] = analogRead(phPin);
//       if (pHArrayIndex == ArrayLenth)
//         pHArrayIndex = 0;
//       voltage = avergearray(pHArray, ArrayLenth) * 3.3 / 4095;
//       ph_value = 3.5 * voltage + Offset;
//       dtostrf(ph_value, 6, 2, myChar);
//       pHValue = myChar;
//       samplingTime = millis();
//     }

//     if (!client.connected())
//     {
//       reconnect(3, pHValue);
//     }
//     client.publish(MQTT_PUB_ph, pHValue);
//     vTaskDelay(pdMS_TO_TICKS(3000)); // wait for 1 second before reading again
//   }
// }

// double avergearray(int *arr, int number)
// {
//   int i;
//   int max, min;
//   double avg;
//   long amount = 0;
//   if (number <= 0)
//   {
//     Serial.println("Error number for the array to avraging!/n");
//     return 0;
//   }
//   if (number < 5)
//   { // less than 5, calculated directly statistics
//     for (i = 0; i < number; i++)
//     {
//       amount += arr[i];
//     }
//     avg = amount / number;
//     return avg;
//   }
//   else
//   {
//     if (arr[0] < arr[1])
//     {
//       min = arr[0];
//       max = arr[1];
//     }
//     else
//     {
//       min = arr[1];
//       max = arr[0];
//     }
//     for (i = 2; i < number; i++)
//     {
//       if (arr[i] < min)
//       {
//         amount += min; // arr<min
//         min = arr[i];
//       }
//       else
//       {
//         if (arr[i] > max)
//         {
//           amount += max; // arr>max
//           max = arr[i];
//         }
//         else
//         {
//           amount += arr[i]; // min<=arr<=max
//         }
//       } // if
//     }   // for
//     avg = (double)amount / (number - 2);
//   } // if
//   return avg;
// }

void mh_z14a_Input_Task(void *Parameters)
{

  pinMode(co2InputPin, INPUT_PULLDOWN);
  dataSensor dataSensor;
  // int co2_Input_C;
  while (true)
  {
    while (digitalRead(co2InputPin) == LOW)
    {
    };
    long t0 = millis();
    while (digitalRead(co2InputPin) == HIGH)
    {
    };
    long t1 = millis();
    while (digitalRead(co2InputPin) == LOW)
    {
    };
    long t2 = millis();
    long th = t1 - t0;
    long tl = t2 - t1;
    long ppm = 5000L * (th - 2) / (th + tl - 4);
    while (digitalRead(co2InputPin) == HIGH)
    {
    }
    dataSensor.co2_Input_C = int(ppm);
    xQueueSend(xQueue1, &dataSensor.co2_Input_C, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(3000)); // wait for 1 second before reading again
  }
}

void mh_z14a_Output_task(void *Parameters)
{
  pinMode(co2OutputPin, INPUT_PULLDOWN);
  dataSensor dataSensor;
  while (true)
  {
    while (digitalRead(co2OutputPin) == LOW)
    {
    };
    long t0 = millis();
    while (digitalRead(co2OutputPin) == HIGH)
    {
    };
    long t1 = millis();
    while (digitalRead(co2OutputPin) == LOW)
    {
    };
    long t2 = millis();
    long th = t1 - t0;
    long tl = t2 - t1;
    long ppm = 5000L * (th - 2) / (th + tl - 4);
    while (digitalRead(co2OutputPin) == HIGH)
    {
    }
    dataSensor.co2_Output_C = int(ppm);
    xQueueSend(xQueue2, &dataSensor.co2_Output_C, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(3000)); // wait for 1 second before reading again
  }
}

void recive(void *pvParameters)
{
  dataSensor recieveData;
  while (true)
  {
    // Receive the data from the queue
    xQueueReceive(xQueue4, &recieveData.wather_Temperature_C, portMAX_DELAY);
    Serial.print("wather tem : ");
    Serial.print(recieveData.wather_Temperature_C);
    Serial.print("\t");
    xQueueReceive(xQueue3, &recieveData.air_Humidity_C, portMAX_DELAY);
    Serial.print("air Humi : ");
    Serial.print(recieveData.air_Humidity_C);
    Serial.print("\t");
    xQueueReceive(xQueue3, &recieveData.air_Temperature_C, portMAX_DELAY);
    Serial.print("air tem : ");
    Serial.print(recieveData.air_Temperature_C);
    Serial.print("\t");
    xQueueReceive(xQueue1, &recieveData.co2_Input_C, portMAX_DELAY);
    Serial.print("co2 input : ");
    Serial.print(recieveData.co2_Input_C);
    Serial.print("\t");
    xQueueReceive(xQueue2, &recieveData.co2_Output_C, portMAX_DELAY);
    Serial.print("co2 Output : ");
    Serial.println(recieveData.co2_Output_C);

    Serial.print("Free heap (bytes): ");
    Serial.println(xPortGetFreeHeapSize());
  }
}

// void prinDataSensor(void)
// {
//   /* code */
//   Serial.print("DS18B20 : ");
//   Serial.print(wather_Temp);
//   Serial.print("\t\t");
//   Serial.print("CO2 INPUT: ");
//   Serial.print(co2_Input_C);
//   Serial.print("\t\t");
//   Serial.print("CO2 OUTPUT: ");
//   Serial.print(co2_Output_C);
//   Serial.print("\t\t");
//   Serial.print("PH : ");
//   Serial.println(ph_value);
// }

void keepWiFiAlive(void *parameter)
{
  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }

    Serial.println("[WIFI] Connecting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();

    // Keep looping while we're not connected and haven't reached the timeout
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startAttemptTime < WIFI_TIMEOUT_MS)
    {
    }

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("[WIFI] FAILED");
      vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
      continue;
    }
    Serial.print("[WIFI] Connected: ");
    Serial.println(WiFi.localIP());
  }
}

void setup()
{
  Serial.begin(112500);
  Serial.println("-----------------DATABIOTA PROJECT---------------------");

  xQueue1 = xQueueCreate(10, sizeof(int));
  xQueue2 = xQueueCreate(10, sizeof(int));
  xQueue3 = xQueueCreate(10, sizeof(int));
  xQueue4 = xQueueCreate(10, sizeof(int));

  xTaskCreatePinnedToCore(
      keepWiFiAlive,
      "keepWiFiAlive", // Task name
      3000,            // Stack size (bytes)
      NULL,            // Parameter
      1,               // Task priority
      &taskWifi,       // Task handle
      0);

  xTaskCreatePinnedToCore(
      recive,
      "recive",     // Task name
      1500,         // Stack size (bytes)
      NULL,         // Parameter
      2,            // Task priority
      &taskrecieve, // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Input_Task,
      "mh_z14a_Input_Task", // Task name
      2000,                 // Stack size (bytes)
      NULL,                 // Parameter
      1,                    // Task priority
      &taskco2In,           // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Output_task,
      "mh_z14a_Output_task", // Task name
      2000,                  // Stack size (bytes)
      NULL,                  // Parameter
      1,                     // Task priority
      &taskco2Out,           // Task handle
      0);

  // xTaskCreatePinnedToCore(
  //     sensor_ph_task,
  //     "sensor_ph_task", // Task name
  //     2048,             // Stack size (bytes)
  //     NULL,             // Parameter
  //     1,                // Task priority
  //     NULL,             // Task handl
  //     1);

  xTaskCreatePinnedToCore(
      sensor_watherTemp_task,
      "sensor_watherTemp_task", // Task name
      2000,                     // Stack size (bytes)
      NULL,                     // Parameter
      2,                        // Task priority
      &taskDals11,              // Task handle
      0);

  xTaskCreatePinnedToCore(
      sensor_sht3x_task,
      "sensor_sht3x_task", // Task name
      2500,                // Stack size (bytes)
      NULL,                // Parameter
      2,                   // Task priority
      &tasksht13,          // Task handle
      1);

  // xTaskCreatePinnedToCore(
  //     sensor_turbidity_task,
  //     "sensor_turbidity_task", // Task name
  //     3048,                    // Stack size (bytes)
  //     NULL,                    // Parameter
  //     1,                       // Task priority
  //     NULL,                    // Task handl
  //     1);

  // client.setServer(mqtt_server, 1883);
  // client.setCallback(callback);
  vTaskDelete(NULL);
}
void loop()
{
  // client.loop();
}