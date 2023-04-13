#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <math.h>
#include <ArduinoJson.h>

// wifi inisialisasi
const char *ssid = "Neurabot";
const char *password = "Kempul4321!";
// mqtt inisislisasi
const char *mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char *mqtt_client_id = "ESP32Client";

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

char payload[256];

QueueHandle_t xQueue1; // co2 input
QueueHandle_t xQueue2; // co2 output
QueueHandle_t xQueue3; // air tem
QueueHandle_t xQueue4; // air rh
QueueHandle_t xQueue5; // wather tem
QueueHandle_t xQueue6; // ph
QueueHandle_t xQueue7; // tbd
QueueHandle_t xQueue8; // payload

struct toppic
{
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

#define WIFI_TIMEOUT_MS 20000      // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt

#define Offset 0.00 // deviation compensate
#define samplingInterval 20
#define ArrayLenth 40 // times of collection

static TaskHandle_t taskWifi;
static TaskHandle_t taskco2In;
static TaskHandle_t taskco2Out;
static TaskHandle_t tasksht13;
static TaskHandle_t taskDals11;
static TaskHandle_t taskph;
static TaskHandle_t taskrecieve;
static TaskHandle_t taskTurbidity;
static TaskHandle_t taskmqttPublish;

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
void sensor_tbd_task(void *Parameters)
{
  pinMode(turbidityPin, INPUT);
  int adc;
  float volt;
  dataSensor dataSensor;

  while (true)
  {
    adc = analogRead(turbidityPin);
    // Serial.println(adc);
    adc = map(adc, 0, 4095, 0, 1023);
    volt = adc * (5.0 / 1023.0);
    volt = round_to_dp(volt, 2);
    // Serial.println(volt);
    dataSensor.tbd_C = -1120.4 * square(volt) + 5742.3 * volt - 4353.8;
    if (dataSensor.tbd_C < 0)
    {
      Serial.println("Under zero");
      dataSensor.tbd_C = -1.0;
      xQueueSend(xQueue7, &dataSensor.tbd_C, portMAX_DELAY);
    }
    else
    {
      xQueueSend(xQueue7, &dataSensor.tbd_C, portMAX_DELAY);
    }
  }
  vTaskDelay(pdMS_TO_TICKS(10000));
}

float round_to_dp(float in_value, int decimal_place)
{
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier) / multiplier;
  return in_value;
}

float square(float f)
{
  (abs(f) < 256.0);
  return f * f; // check if overflow will occur because 256*256 does not fit in an int anymore
}

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

    xQueueSend(xQueue4, &dataSensor.air_Humidity_C, portMAX_DELAY);
    xQueueSend(xQueue3, &dataSensor.air_Temperature_C, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

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
    xQueueSend(xQueue5, &dataSensor.wather_Temperature_C, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void sensor_ph_task(void *Parameter)
{
  pinMode(phPin, INPUT);
  unsigned long int avgValue; // Store the average value of the sensor feedback
  float b;
  int buf[10], temp;
  dataSensor dataSensor;

  while (true)
  {
    for (int i = 0; i < 10; i++) // Get 10 sample value from the sensor for smooth the value
    {
      buf[i] = analogRead(phPin);
      delay(10);
    }
    for (int i = 0; i < 9; i++) // sort the analog from small to large
    {
      for (int j = i + 1; j < 10; j++)
      {
        if (buf[i] > buf[j])
        {
          temp = buf[i];
          buf[i] = buf[j];
          buf[j] = temp;
        }
      }
    }
    avgValue = 0;
    for (int i = 2; i < 8; i++) // take the average value of 6 center sample
      avgValue += buf[i];
    float Value = (float)avgValue * 3.3 / 4095.0 / 6; // convert the analog into millivolt
    dataSensor.ph_C = 4.086 * Value;
    xQueueSend(xQueue6, &dataSensor.ph_C, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10000)); // wait for 1 second before reading again
  }
}

void mh_z14a_Input_Task(void *Parameters)
{

  pinMode(co2InputPin, INPUT_PULLDOWN);
  dataSensor dataSensor;
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
    vTaskDelay(pdMS_TO_TICKS(10000)); // wait for 1 second before reading again
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
    vTaskDelay(pdMS_TO_TICKS(10000)); // wait for 1 second before reading again
  }
}

void recive(void *pvParameters)
{
  dataSensor recieveData;
  toppic mqttTopic;

  while (true)
  {
    // Receive the data from the queue
    xQueueReceive(xQueue7, &recieveData.tbd_C, portMAX_DELAY);
    xQueueReceive(xQueue6, &recieveData.ph_C, portMAX_DELAY);
    xQueueReceive(xQueue5, &recieveData.wather_Temperature_C, portMAX_DELAY);
    xQueueReceive(xQueue4, &recieveData.air_Humidity_C, portMAX_DELAY);
    xQueueReceive(xQueue3, &recieveData.air_Temperature_C, portMAX_DELAY);
    xQueueReceive(xQueue2, &recieveData.co2_Output_C, portMAX_DELAY);
    xQueueReceive(xQueue1, &recieveData.co2_Input_C, portMAX_DELAY);
    Serial.println(recieveData.tbd_C);

    // with json
    DynamicJsonDocument jsonDoc(256);
    jsonDoc["co2Input"] = String(recieveData.co2_Input_C);
    jsonDoc["co2Output"] = String(recieveData.co2_Output_C);
    jsonDoc["airTem"] = String(recieveData.air_Temperature_C, 2);
    jsonDoc["airRh"] = String(recieveData.air_Humidity_C, 2);
    jsonDoc["watherTem"] = String(recieveData.wather_Temperature_C, 2);
    jsonDoc["tbd"] = String(recieveData.tbd_C, 2);
    jsonDoc["pH"] = String(recieveData.ph_C, 2);

    serializeJson(jsonDoc, payload);
    Serial.println(payload);
    xQueueSend(xQueue8, &payload, portMAX_DELAY);
    Serial.print("Free heap (bytes): ");
    Serial.println(xPortGetFreeHeapSize());
    // vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void publishMqtt(void *Parameters)
{
  while (true)
  {
    xQueueReceive(xQueue8, &payload, portMAX_DELAY);
    if (!mqtt_client.connected())
    {
      reconnect();
    }

    bool result = mqtt_client.publish("my/topic", payload);
    if (result)
    {
      Serial.println("Message published successfully");
    }
    else
    {
      Serial.println("Message failed to publish");
    }
    mqtt_client.loop();
    // vTaskDelay(pdMS_TO_TICKS(11000));
  }
}

void reconnect()
{
  while (!mqtt_client.connected())
  {
    Serial.println("Connecting to MQTT broker...");
    if (mqtt_client.connect("espjos"))
    {
      Serial.println("Connected to MQTT broker");
      mqtt_client.subscribe("my/topic");
    }
    else
    {
      Serial.println("Connection to MQTT broker failed");
      delay(5000);
    }
  }
}

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
  mqtt_client.setServer(mqtt_server, mqtt_port);
  xQueue1 = xQueueCreate(10, sizeof(int));
  xQueue2 = xQueueCreate(10, sizeof(int));
  xQueue3 = xQueueCreate(10, sizeof(int));
  xQueue4 = xQueueCreate(10, sizeof(int));
  xQueue5 = xQueueCreate(10, sizeof(int));
  xQueue6 = xQueueCreate(10, sizeof(int));
  xQueue7 = xQueueCreate(10, sizeof(int));
  xQueue8 = xQueueCreate(10, sizeof(int));

  xTaskCreatePinnedToCore(
      keepWiFiAlive,
      "keepWiFiAlive", // Task name
      5000,            // Stack size (bytes)
      NULL,            // Parameter
      1,               // Task priority
      &taskWifi,       // Task handle
      0);

  xTaskCreatePinnedToCore(
      recive,
      "recive",     // Task name
      4000,         // Stack size (bytes)
      NULL,         // Parameter
      1,            // Task priority
      &taskrecieve, // Task handle
      1);

  xTaskCreatePinnedToCore(
      publishMqtt,
      "publishMqtt",    // Task name
      5000,             // Stack size (bytes)
      NULL,             // Parameter
      2,                // Task priority
      &taskmqttPublish, // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Input_Task,
      "mh_z14a_Input_Task", // Task name
      5000,                 // Stack size (bytes)
      NULL,                 // Parameter
      2,                    // Task priority
      &taskco2In,           // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Output_task,
      "mh_z14a_Output_task", // Task name
      5000,                  // Stack size (bytes)
      NULL,                  // Parameter
      1,                     // Task priority
      &taskco2Out,           // Task handle
      1);

  xTaskCreatePinnedToCore(
      sensor_ph_task,
      "sensor_ph_task", // Task name
      2500,             // Stack size (bytes)
      NULL,             // Parameter
      2,                // Task priority
      &taskph,          // Task handl
      1);

  xTaskCreatePinnedToCore(
      sensor_watherTemp_task,
      "sensor_watherTemp_task", // Task name
      2500,                     // Stack size (bytes)
      NULL,                     // Parameter
      3,                        // Task priority
      &taskDals11,              // Task handle
      0);

  xTaskCreatePinnedToCore(
      sensor_sht3x_task,
      "sensor_sht3x_task", // Task name
      2500,                // Stack size (bytes)
      NULL,                // Parameter
      3,                   // Task priority
      &tasksht13,          // Task handle
      1);

  xTaskCreatePinnedToCore(
      sensor_tbd_task,
      " sensor_tbd_task",
      2000,
      NULL,
      3,
      &taskTurbidity,
      1);

  // Connect to MQTT broker

  vTaskDelete(NULL);
}

void loop()
{
  // mqtt_client.loop();
}