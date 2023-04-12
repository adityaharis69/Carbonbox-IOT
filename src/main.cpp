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
const char *mqtt_port = "1883";
const char *mqtt_client_id = "ESP32Client";

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

QueueHandle_t xQueue1; // co2 input
QueueHandle_t xQueue2; // co2 output
QueueHandle_t xQueue3; // air tem
QueueHandle_t xQueue4; // air rh
QueueHandle_t xQueue5; // wather tem
QueueHandle_t xQueue6; // ph
QueueHandle_t xQueue7; // tbd

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

struct convertMqtt
{
  char floatChar[10];
  const char *floatSensorVAl;
  String my_str;
  const char *my_const_char_value;
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
  vTaskDelay(pdMS_TO_TICKS(3000));
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

    vTaskDelay(pdMS_TO_TICKS(3000));
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
    vTaskDelay(pdMS_TO_TICKS(3000));
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
    vTaskDelay(pdMS_TO_TICKS(3000)); // wait for 1 second before reading again
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
  toppic mqttTopic;
  convertMqtt conv_co2Input;
  convertMqtt conv_co2Output;
  convertMqtt conv_dalsTem;
  convertMqtt conv_airRh;
  convertMqtt conv_airTem;

  // mqtt_client.setServer(mqtt_server, 1883);
  // mqtt_client.connect(mqtt_client_id);
  // mqtt_client.setClient(wifi_client);
  // while (!mqtt_client.connected())
  // {
  //   Serial.println("Connecting to MQTT broker...");
  //   if (mqtt_client.connect(mqtt_client_id))
  //   {
  //     Serial.println("Connected to MQTT broker");
  //   }
  //   else
  //   {
  //     Serial.print("Failed to connect to MQTT broker, rc=");
  //     Serial.print(mqtt_client.state());
  //     Serial.println(" retrying...");
  //     // delay(1000);
  //   }
  // }

  while (true)
  {
    // Receive the data from the queue
    xQueueReceive(xQueue6, &recieveData.ph_C, portMAX_DELAY);
    // Serial.print("pH : ");
    // Serial.print(recieveData.ph_C);
    // Serial.print("\t");

    xQueueReceive(xQueue7, &recieveData.tbd_C, portMAX_DELAY);
    // Serial.print("Turbidity : ");
    // Serial.print(recieveData.tbd_C);
    // Serial.print("\t");

    xQueueReceive(xQueue5, &recieveData.wather_Temperature_C, portMAX_DELAY);
    // Serial.print("wather tem : ");
    // Serial.print(recieveData.wather_Temperature_C);
    // Serial.print("\t");
    xQueueReceive(xQueue4, &recieveData.air_Humidity_C, portMAX_DELAY);
    // Serial.print("air Humi : ");
    // Serial.print(recieveData.air_Humidity_C);
    // Serial.print("\t");
    xQueueReceive(xQueue3, &recieveData.air_Temperature_C, portMAX_DELAY);
    // Serial.print("air tem : ");
    // Serial.print(recieveData.air_Temperature_C);
    // Serial.print("\t");
    xQueueReceive(xQueue1, &recieveData.co2_Input_C, portMAX_DELAY);
    // Serial.print("co2 input : ");
    // Serial.print(recieveData.co2_Input_C);
    // Serial.print("\t");
    xQueueReceive(xQueue2, &recieveData.co2_Output_C, portMAX_DELAY);
    // Serial.print("co2 Output : ");
    // Serial.println(recieveData.co2_Output_C);

    // with json
    DynamicJsonDocument jsonDoc(128);
    jsonDoc["co2Input"] = recieveData.co2_Input_C;
    jsonDoc["co2Output"] = recieveData.co2_Output_C;
    jsonDoc["airTem"] = recieveData.air_Temperature_C;
    jsonDoc["airRh"] = recieveData.air_Humidity_C;
    jsonDoc["watherTem"] = recieveData.wather_Temperature_C;
    jsonDoc["pH"] = recieveData.ph_C;
    jsonDoc["tbd"] = recieveData.tbd_C;

    char payload[128];
    serializeJson(jsonDoc, payload);
    Serial.println(payload);
    if (!mqtt_client.connected())
    {
      reconnect();
    }
    mqtt_client.loop();
    mqtt_client.publish(mqttTopic.MQTT_PUB_CO2input, "aditya");
    // if (mqtt_client.connected())
    // {
    //   if (mqtt_client.publish(mqttTopic.MQTT_PUB_CO2input, payload))
    //   {
    //     Serial.println("Message published successfully!");
    //   }
    //   else
    //   {
    //     Serial.println("Failed to publish message!");
    //   }
    // }
    // else
    // {
    //   while (!mqtt_client.connected())
    //   {
    //     Serial.println("reconnect to MQTT broker...");
    //     if (mqtt_client.connect(mqtt_client_id))
    //     {
    //       if (mqtt_client.publish(mqttTopic.MQTT_PUB_CO2input, payload))
    //       {
    //         Serial.println("Message published successfully!");
    //       }
    //       else
    //       {
    //         Serial.println("Failed to publish message!");
    //       }
    //     }
    //     else
    //     {
    //       Serial.print("Failed to connect to MQTT broker, rc=");
    //       Serial.print(mqtt_client.state());
    //       Serial.println(" retrying...");
    //       // delay(1000);
    //     }
    //   }
    // }

    // mqtt_client.disconnect();
    // mqtt_client.setServer(mqtt_server, 1883);
    // mqtt_client.connect(mqtt_client_id);
    // mqtt_client.setClient(wifi_client);
    // while (!mqtt_client.connected())
    // {
    //   Serial.println("Connecting to MQTT broker...");
    //   if (mqtt_client.connect("ESP32Client"))
    //   {
    //     Serial.println("Connected to MQTT broker");
    //   }
    //   else
    //   {
    //     Serial.print("Failed to connect to MQTT broker, rc=");
    //     Serial.print(mqtt_client.state());
    //     Serial.println(" retrying...");
    //     delay(1000);
    //   }
    // }
    // // delay(20);
    // // xQueueReceive(xQueue1, &recieveData.co2_Input_C, portMAX_DELAY);
    // if (recieveData.co2_Input_C > 350)
    // {
    //   //   // while (!mqtt_client.connected())
    //   //   // {

    //   //   //   Serial.println("connecting..................co2Input");
    //   //   //   mqtt_client.connect(mqtt_client_id);
    //   //   // }
    //   //   // conv_co2Input.my_str = recieveData.co2_Input_C;
    //   //   // conv_co2Input.my_const_char_value = conv_co2Input.my_str.c_str();
    //          mqtt_client.publish(mqttTopic.MQTT_PUB_CO2input, conv_co2Input.my_const_char_value);
    //   //   // Serial.print(conv_co2Input.my_const_char_value);
    //   //   // Serial.print("\t");
    // }
    // // delay(20);
    // // xQueueReceive(xQueue2, &recieveData.co2_Output_C, portMAX_DELAY);
    // if (recieveData.co2_Output_C > 350)
    // {
    //   // while (!mqtt_client.connected())
    //   // {

    //   //   Serial.println("connecting.................. co2Output");
    //   //   mqtt_client.connect(mqtt_client_id);
    //   // }
    //   // conv_co2Output.my_str = recieveData.co2_Output_C;
    //   // conv_co2Output.my_const_char_value = conv_co2Output.my_str.c_str();
    //   mqtt_client.publish(mqttTopic.MQTT_PUB_CO2output, conv_co2Output.my_const_char_value);
    //   // Serial.print(conv_co2Output.my_const_char_value);
    //   // Serial.print("\t");
    // }
    // // delay(20);
    // // xQueueReceive(xQueue3, &recieveData.air_Temperature_C, portMAX_DELAY);
    // if (recieveData.air_Temperature_C > 0)
    // {
    //   // while (!mqtt_client.connected())
    //   // {

    //   //   Serial.println("connecting..................airTem");
    //   //   mqtt_client.connect(mqtt_client_id);
    //   // }
    //   // dtostrf(recieveData.air_Temperature_C, 6, 2, conv_airTem.floatChar);
    //   // conv_airTem.floatSensorVAl = conv_airTem.floatChar;
    //   mqtt_client.publish(mqttTopic.MQTT_PUB_AirTemp, conv_airTem.floatSensorVAl);
    //   // Serial.print(conv_airTem.floatSensorVAl);
    //   // Serial.print("\t");
    // }

    // // delay(20);
    // // xQueueReceive(xQueue4, &recieveData.air_Humidity_C, portMAX_DELAY);
    // if (recieveData.air_Humidity_C > 0)
    // {
    //   // while (!mqtt_client.connected())
    //   // {

    //   //   Serial.println("connecting.................. rh");
    //   //   mqtt_client.connect(mqtt_client_id);
    //   // }
    //   // dtostrf(recieveData.air_Humidity_C, 6, 2, conv_airRh.floatChar);
    //   // conv_airRh.floatSensorVAl = conv_airRh.floatChar;
    //   mqtt_client.publish(mqttTopic.MQTT_PUB_rh, conv_airRh.floatSensorVAl);
    //   // Serial.print(conv_airRh.floatSensorVAl);
    //   // Serial.print("\t");
    // }

    // // delay(20);
    // // xQueueReceive(xQueue5, &recieveData.wather_Temperature_C, portMAX_DELAY);
    // if (recieveData.wather_Temperature_C > 0)
    // {
    //   // while (!mqtt_client.connected())
    //   // {

    //   //   Serial.println("connecting..................waterTem");
    //   //   mqtt_client.connect(mqtt_client_id);
    //   // }
    //   // dtostrf(recieveData.wather_Temperature_C, 6, 2, conv_dalsTem.floatChar);
    //   // conv_dalsTem.floatSensorVAl = conv_dalsTem.floatChar;
    //   mqtt_client.publish(mqttTopic.MQTT_PUB_WatherTemp, conv_dalsTem.floatSensorVAl);
    //   // Serial.print(conv_dalsTem.floatSensorVAl);
    //   // Serial.println("\t");
    // }
    mqtt_client.disconnect();
    Serial.print("Free heap (bytes): ");
    Serial.println(xPortGetFreeHeapSize());
    // vTaskDelay(pdMS_TO_TICKS(3000));
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

void reconnect()
{
  // Loop until we're reconnected
  toppic toppic;
  while (!mqtt_client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect("ESP32Client"))
    {
      Serial.println("connected");
      mqtt_client.publish(toppic.MQTT_PUB_CO2input, "aditya");
      // Subscribe
      // client.subscribe("esp32/output");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
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
  xQueue5 = xQueueCreate(10, sizeof(int));
  xQueue6 = xQueueCreate(10, sizeof(int));
  xQueue7 = xQueueCreate(10, sizeof(int));

  mqtt_client.setServer(mqtt_server, 1883);

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
      5000,         // Stack size (bytes)
      NULL,         // Parameter
      1,            // Task priority
      &taskrecieve, // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Input_Task,
      "mh_z14a_Input_Task", // Task name
      3000,                 // Stack size (bytes)
      NULL,                 // Parameter
      2,                    // Task priority
      &taskco2In,           // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Output_task,
      "mh_z14a_Output_task", // Task name
      3000,                  // Stack size (bytes)
      NULL,                  // Parameter
      2,                     // Task priority
      &taskco2Out,           // Task handle
      0);

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

  xTaskCreatePinnedToCore(
      sensor_tbd_task,
      " sensor_tbd_task",
      2000,
      NULL,
      2,
      &taskTurbidity,
      1);

  vTaskDelete(NULL);
}

void loop()
{
  // mqtt_client.loop();
}