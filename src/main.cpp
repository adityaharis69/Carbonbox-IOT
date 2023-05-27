#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <math.h>
#include <ArduinoJson.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <Point.h>
#include <WiFiMulti.h>

WiFiMulti multi;

const char *ssid = "Neurabot";
const char *password = "Kempul4321!";

#define DEVICE "CARBONBOX-V1"
#define WIFI_SSID "NEURABOT"
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "1UCnp58kS25EHtJrBDpdMqyKZ4rs2y7j-NlqAv9tA4mKpNd6w42X83xaAxzMmSb9-7jGJS14JyWyA021FB4-aw=="
#define INFLUXDB_ORG "b74c259f49b45988"
#define INFLUXDB_BUCKET "CARBONBOX"
// Time zone info
#define TZ_INFO "UTC7"

Point sensor("wifi_status");

// durasi tunggu task
uint32_t waktuTunggu = 30000;

// InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient clientInflux(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// InfluxDBClient clientInflux(my_client, INFLUXDB_URL, INFLUXDB_TOKEN);
// InfluxDBClient influxDbClient(wifiClient, influxDbUrl, influxDbToken);
// wifi inisialisasi
// Data point
// Point sensor("wifi_status");

// mqtt inisislisasi
// const char *mqtt_server = "test.mosquitto.org";
// const int mqtt_port = 1883;
// const char *mqtt_client_id = "ESP32Client";

// PubSubClient mqtt_client(wifi_client);

const int d_sensor = 2;
int arrayData[d_sensor] = {0, 0};

char payload[256];

QueueHandle_t xQueue1; // co2 input
QueueHandle_t xQueue2; // co2 output
QueueHandle_t xQueue3; // air tem
QueueHandle_t xQueue4; // air rh
QueueHandle_t xQueue5; // wather tem
QueueHandle_t xQueue6; // ph
QueueHandle_t xQueue7; // tbd
// QueueHandle_t xQueue8; // payload

// struct toppic
// {
//   const char *MQTT_PUB_CO2input = "esp32/mh-z14a/input";
//   const char *MQTT_PUB_CO2output = "esp32/mh-z14a/output";
//   const char *MQTT_PUB_ph = "esp32/dfRobot/ph";
//   const char *MQTT_PUB_tbd = "esp32/dfRobot/tbd";
//   const char *MQTT_PUB_AirTemp = "esp32/sht3x/temp";
//   const char *MQTT_PUB_rh = "esp32/sht3x/rh";
//   const char *MQTT_PUB_WatherTemp = "esp32/ds18b20/temp";
// };

#define co2InputPin GPIO_NUM_32   // co2 input
#define co2OutputPin GPIO_NUM_33  // co2 output
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
static TaskHandle_t taskWatherOverLine;

struct dataSensor
{
  int co2_Input_C;
  int co2_Output_C;
  float air_Temperature_C;
  float air_Humidity_C;
  float ph_C;
  float wather_Temperature_C;
  float tbd_C;
};

void watherOverLine(void *Parameters)
{
  pinMode(relayPompaPin, OUTPUT);
  pinMode(waterLevelPin, INPUT_PULLUP);
  digitalWrite(relayPompaPin, HIGH);

  while (true)
  {
    // Serial.println(digitalRead(waterLevelPin));
    if (digitalRead(waterLevelPin) == 0)
    {
      digitalWrite(relayPompaPin, HIGH);
      // Serial.println("break PIMP");
    }
    else
    {
      digitalWrite(relayPompaPin, LOW);
      // Serial.println("runnung PUMP");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

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
  vTaskDelay(pdMS_TO_TICKS(waktuTunggu));
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

void mh_z14a_Output_Task(void *Parameters)
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
    dataSensor.co2_Input_C = int(ppm);
    xQueueSend(xQueue1, &dataSensor.co2_Output_C, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10000)); // wait for 1 second before reading again
  }
}

void mh_z14a_Input_task(void *Parameters)
{
  dataSensor dataSensor;
  while (true)
  {
    bool check = checkSerial("u");
    if (check)
    {
      serialRead();
      dataSensor.co2_Input_C = int(arrayData[1]);
      xQueueSend(xQueue2, &dataSensor.co2_Input_C, portMAX_DELAY);
    }

    Serial2.print("s:");
    vTaskDelay(pdMS_TO_TICKS(10000)); // wait for 1 second before reading again
  }
}
boolean serialRead()
{
  if (Serial2.available() > 0) // program untuk parsing data
  {
    String input = Serial2.readStringUntil('\n');
    input.trim();

    int tempIndex = 0;
    for (int i = 0; i < d_sensor; i++)
    {
      int index = input.indexOf(":", tempIndex);
      arrayData[i] = input.substring(tempIndex, index).toInt();
      tempIndex = index + 1;
    }

    input = "";

    return true;
  }
  return false;
}

boolean checkSerial(String value)
{
  if (Serial2.available() > 0)
  {
    String input = Serial2.readStringUntil(':');
    input.trim();
    if (input == value)
    {
      return true;
    }
    return false;
  }
  return false;
}

void recive(void *pvParameters)
{
  dataSensor recieveData;
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // add tag
  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());

  // Check server connection
  // Connect to InfluxDB

  String filed[7] = {"CO2_IN", "CO2_OUT", "TEMP_UDARA", "RH_UDARA", "TEMP_AIR", "PH", "TBD"};

  while (true)
  {
    if (!clientInflux.validateConnection())
    {
      Serial.println("InfluxDB connection failed!");
      while (1)
      {
        delay(500);
      }
    }
    Serial.println("Connected to InfluxDB");

    // Receive the data from the queue
    xQueueReceive(xQueue7, &recieveData.tbd_C, portMAX_DELAY);
    xQueueReceive(xQueue6, &recieveData.ph_C, portMAX_DELAY);
    xQueueReceive(xQueue5, &recieveData.wather_Temperature_C, portMAX_DELAY);
    xQueueReceive(xQueue4, &recieveData.air_Humidity_C, portMAX_DELAY);
    xQueueReceive(xQueue3, &recieveData.air_Temperature_C, portMAX_DELAY);
    xQueueReceive(xQueue2, &recieveData.co2_Output_C, portMAX_DELAY);
    xQueueReceive(xQueue1, &recieveData.co2_Input_C, portMAX_DELAY);

    Serial.print("Data Sensor");
    Serial.print(" ");
    Serial.print(recieveData.co2_Input_C);
    Serial.print(" ");
    Serial.print(recieveData.co2_Output_C);
    Serial.print(" ");
    Serial.print(recieveData.air_Temperature_C);
    Serial.print(" ");
    Serial.print(recieveData.air_Humidity_C);
    Serial.print(" ");
    Serial.print(recieveData.wather_Temperature_C);
    Serial.print(" ");
    Serial.print(recieveData.ph_C);
    Serial.print(" ");
    Serial.println(recieveData.tbd_C);

    sensor.clearFields();
    sensor.addField(filed[0], recieveData.co2_Input_C);
    sensor.addField(filed[1], recieveData.co2_Output_C);
    sensor.addField(filed[2], recieveData.air_Temperature_C);
    sensor.addField(filed[3], recieveData.air_Humidity_C);
    sensor.addField(filed[4], recieveData.wather_Temperature_C);
    sensor.addField(filed[5], recieveData.ph_C);
    sensor.addField(filed[6], recieveData.tbd_C);

    // Check WiFi connection and reconnect if needed
    // If no Wifi signal, try to reconnect it
    if (multi.run() != WL_CONNECTED)
    {
      Serial.println("Wifi connection lost");
    }

    // Write point
    if (!clientInflux.writePoint(sensor))
    {
      Serial.print("InfluxDB write failed: ");
      Serial.println(clientInflux.getLastErrorMessage());
    }

    // for (int x = 0; x <= 7; x++)
    // {
    //   nilai[0]=

    // }
    // with json
    // DynamicJsonDocument jsonDoc(256);
    // jsonDoc["co2Input"] = String(recieveData.co2_Input_C);
    // jsonDoc["co2Output"] = String(recieveData.co2_Output_C);
    // jsonDoc["airTem"] = String(recieveData.air_Temperature_C, 2);
    // jsonDoc["airRh"] = String(recieveData.air_Humidity_C, 2);
    // jsonDoc["watherTem"] = String(recieveData.wather_Temperature_C, 2);
    // jsonDoc["tbd"] = String(recieveData.tbd_C, 2);
    // jsonDoc["pH"] = String(recieveData.ph_C, 2);
    // serializeJson(jsonDoc, payload);

    // // Parse JSON string
    // DeserializationError error = deserializeJson(jsonDoc, payload);
    // JsonArray jsonArray = jsonDoc.as<JsonArray>();

    // int dataCount = jsonArray.size();

    // Serial.print("data count  : ");
    // Serial.println(dataCount);

    // for (int a = 0; a <= 6; a++)
    // {
    //   // Create a data point
    //   Point dataPoint(filed[a]);
    //   dataPoint.addField("value", 120);

    //   // Write data point to InfluxDB
    //   if (clientInflux.writePoint(dataPoint))
    //   {
    //     Serial.println("Data sent to InfluxDB");
    //   }
    //   else
    //   {
    //     Serial.println("Failed to send data to InfluxDB");
    //   }
    // }

    // Serial.println(payload);
    // xQueueSend(xQueue8, &payload, portMAX_DELAY);
    // Serial.print("Free heap (bytes): ");
    Serial.print("HEAP MEMORI  :: ");
    Serial.println(xPortGetFreeHeapSize());
    // vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// void publishMqtt(void *Parameters)
// {
//   while (true)
//   {
//     xQueueReceive(xQueue8, &payload, portMAX_DELAY);
//     if (!mqtt_client.connected())
//     {
//       reconnect();
//     }

//     bool result = mqtt_client.publish("my/topic", payload);
//     if (result)
//     {
//       Serial.println("Message published successfully");
//     }
//     else
//     {
//       Serial.println("Message failed to publish");
//     }
//     mqtt_client.loop();
//     // vTaskDelay(pdMS_TO_TICKS(11000));
//   }
// }

// void reconnect()
// {
//   while (!mqtt_client.connected())
//   {
//     Serial.println("Connecting to MQTT broker...");
//     if (mqtt_client.connect("espjos"))
//     {
//       Serial.println("Connected to MQTT broker");
//       mqtt_client.subscribe("my/topic");
//     }
//     else
//     {
//       Serial.println("Connection to MQTT broker failed");
//       delay(5000);
//     }
//   }
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
    multi.addAP(ssid, password);
    // WiFi.begin(, );
    unsigned long startAttemptTime = millis();

    // Keep looping while we're not connected and haven't reached the timeout
    while (multi.run() != WL_CONNECTED &&
           millis() - startAttemptTime < WIFI_TIMEOUT_MS)
    {
    }

    if (multi.run() != WL_CONNECTED)
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
  Serial2.begin(9600);

  Serial.println("-----------------DATABIOTA PROJECT---------------------");
  // mqtt_client.setServer(mqtt_server, mqtt_port);
  // Accurate time is necessary for certificate validation
  // For the fastest time sync find NTP servers in your area: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial

  // timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Data point
  Point sensor("wifi_status");
  sensor.addTag("device", "CarbonBox");
  sensor.addTag("sensor", "CO2");
  sensor.addTag("sensor", "pH");
  sensor.addTag("sensor", "temparature air");
  sensor.addTag("sensor", "temparature udara");
  sensor.addTag("sensor", "RH udara");

  // influxDbClient.setConnectionParams(influxDbOrg, influxDbBucket);

  // Wait for InfluxDB connection
  // while (!influxDbClient.validateConnection())
  // {
  //   delay(1000);
  //   Serial.println("Connecting to InfluxDB...");
  // }
  // Serial.println("Connected to InfluxDB");

  xQueue1 = xQueueCreate(10, sizeof(int));
  xQueue2 = xQueueCreate(10, sizeof(int));
  xQueue3 = xQueueCreate(10, sizeof(int));
  xQueue4 = xQueueCreate(10, sizeof(int));
  xQueue5 = xQueueCreate(10, sizeof(int));
  xQueue6 = xQueueCreate(10, sizeof(int));
  xQueue7 = xQueueCreate(10, sizeof(int));
  // xQueue8 = xQueueCreate(10, sizeof(int));

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
      10000,        // Stack size (bytes)
      NULL,         // Parameter
      1,            // Task priority
      &taskrecieve, // Task handle
      1);

  // xTaskCreatePinnedToCore(
  //     publishMqtt,
  //     "publishMqtt",    // Task name
  //     5000,             // Stack size (bytes)
  //     NULL,             // Parameter
  //     2,                // Task priority
  //     &taskmqttPublish, // Task handle
  //     1);

  xTaskCreatePinnedToCore(
      mh_z14a_Input_task,
      "mh_z14a_Input_Task", // Task name
      5000,                 // Stack size (bytes)
      NULL,                 // Parameter
      2,                    // Task priority
      &taskco2In,           // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Output_Task,
      "mh_z14a_Output_task", // Task name
      7000,                  // Stack size (bytes)
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

  xTaskCreatePinnedToCore(
      watherOverLine,
      " watherOverLine",
      2000,
      NULL,
      3,
      &taskWatherOverLine,
      1);
  // Connect to MQTT broker

  vTaskDelete(NULL);
}

void loop()
{
  // mqtt_client.loop();
}