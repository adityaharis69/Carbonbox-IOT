#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Wire.h>

#define MQTT_PORT 1883
const char *mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

const char *MQTT_PUB_CO2input = "esp32/mh-z14a/inpit";
const char *MQTT_PUB_CO2output = "esp32/mh-z14a/output";
const char *MQTT_PUB_ph = "esp32/dfRobot/ph";
const char *MQTT_PUB_tbd = "esp32/dfRobot/tbd";
const char *MQTT_PUB_AirTemp = "esp32/sht3x/temp";
const char *MQTT_PUB_rh = "esp32/sht3x/rh";
const char *MQTT_PUB_WatherTemp = "esp32/ds18b20/temp";

#define pwmPin GPIO_NUM_33       // co2 New
#define pwmPin2 GPIO_NUM_32      // co2 Old
#define phPin GPIO_NUM_34        // ph
#define watherTemPin GPIO_NUM_19 // pin DS18B20

OneWire oneWire(watherTemPin);
DallasTemperature sensors(&oneWire);

const char *ssid = "Neurabot";
const char *password = "Kempul4321!";

#define WIFI_TIMEOUT_MS 20000      // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt

#define Offset 0.00 // deviation compensate
#define samplingInterval 20
#define ArrayLenth 40 // times of collection

int co2_Input_Contraction;
int co2_Ouput_Contraction;
static float pHValue;
float wather_Temp;

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("codersid/nodemcu/v1", "hello world");
      // ... and resubscribe
      client.subscribe("codersid/nodemcu/v1");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sensor_watherTemp_task(void *Parameters)
{
  sensors.begin();

  while (true)
  {
    sensors.requestTemperatures();
    wather_Temp = sensors.getTempCByIndex(0);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void sensor_ph_task(void *Parameter)
{
  int pHArray[ArrayLenth]; // Store the average value of the sensor feedback
  int pHArrayIndex = 0;
  static float voltage;
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();

  while (true)
  {
    if (millis() - samplingTime > samplingInterval)
    {
      pHArray[pHArrayIndex++] = analogRead(phPin);
      if (pHArrayIndex == ArrayLenth)
        pHArrayIndex = 0;
      voltage = avergearray(pHArray, ArrayLenth) * 3.3 / 4095;
      pHValue = 3.5 * voltage + Offset;
      samplingTime = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(3000)); // wait for 1 second before reading again
  }
}

double avergearray(int *arr, int number)
{
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0)
  {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5)
  { // less than 5, calculated directly statistics
    for (i = 0; i < number; i++)
    {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  }
  else
  {
    if (arr[0] < arr[1])
    {
      min = arr[0];
      max = arr[1];
    }
    else
    {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++)
    {
      if (arr[i] < min)
      {
        amount += min; // arr<min
        min = arr[i];
      }
      else
      {
        if (arr[i] > max)
        {
          amount += max; // arr>max
          max = arr[i];
        }
        else
        {
          amount += arr[i]; // min<=arr<=max
        }
      } // if
    }   // for
    avg = (double)amount / (number - 2);
  } // if
  return avg;
}

void mh_z14a_New_Task(void *Parameters)
{
  pinMode(pwmPin, INPUT_PULLDOWN);
  while (true)
  {
    while (digitalRead(pwmPin) == LOW)
    {
    };
    long t0 = millis();
    while (digitalRead(pwmPin) == HIGH)
    {
    };
    long t1 = millis();
    while (digitalRead(pwmPin) == LOW)
    {
    };
    long t2 = millis();
    long th = t1 - t0;
    long tl = t2 - t1;
    long ppm = 5000L * (th - 2) / (th + tl - 4);
    while (digitalRead(pwmPin) == HIGH)
    {
    }
    co2_Input_Contraction = int(ppm);
    vTaskDelay(pdMS_TO_TICKS(3000)); // wait for 1 second before reading again
  }
  // vTaskDelete(NULL);
}

void mh_z14a_Old_task(void *Parameters)
{

  pinMode(pwmPin2, INPUT_PULLDOWN);
  while (true)
  {
    while (digitalRead(pwmPin2) == LOW)
    {
    };
    long t0 = millis();
    while (digitalRead(pwmPin2) == HIGH)
    {
    };
    long t1 = millis();
    while (digitalRead(pwmPin2) == LOW)
    {
    };
    long t2 = millis();
    long th = t1 - t0;
    long tl = t2 - t1;
    long ppm = 5000L * (th - 2) / (th + tl - 4);
    while (digitalRead(pwmPin2) == HIGH)
    {
    }
    co2_Ouput_Contraction = int(ppm);
    vTaskDelay(pdMS_TO_TICKS(3000)); // wait for 1 second before reading again
  }
  // vTaskDelete(NULL);
}

void prinDataSensor(void)
{
  /* code */
  Serial.print("DS18B20 : ");
  Serial.print(wather_Temp);
  Serial.print("\t\t");
  Serial.print("CO2 INPUT: ");
  Serial.print(co2_Input_Contraction);
  Serial.print("\t\t");
  Serial.print("CO2 OUTPUT: ");
  Serial.print(co2_Ouput_Contraction);
  Serial.print("\t\t");
  Serial.print("PH : ");
  Serial.println(pHValue);
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

// void mqttPublish(void *Parameters)
// {

//   while (true)
//   {
//     if (!client.connected())
//     {
//       reconnect();
//       Serial.println("reconnecting");
//     }
//     client.loop();

//     snprintf(msg, MSG_BUFFER_SIZE, "co2 New #%ld", (const char *)co2_Input_Contraction);
//     Serial.print("Publish message: ");
//     Serial.println(msg);
//     client.publish("codersid/nodemcu/v1", msg);
//   }
//   vTaskDelay(pdMS_TO_TICKS(3000));
// }

void setup()
{
  Serial.begin(112500);
  Serial.println("-----------------DATABIOTA PROJECT---------------------");
  // vTaskDelay(180000);

  Serial.println(WiFi.localIP());
  xTaskCreatePinnedToCore(
      keepWiFiAlive,
      "keepWiFiAlive", // Task name
      5000,            // Stack size (bytes)
      NULL,            // Parameter
      1,               // Task priority
      NULL,            // Task handle
      0);

  xTaskCreatePinnedToCore(
      mh_z14a_New_Task,
      "mh_z14a_New_Task", // Task name
      5122,               // Stack size (bytes)
      NULL,               // Parameter
      1,                  // Task priority
      NULL,               // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Old_task,
      "mh_z14a_Old_task", // Task name
      5122,               // Stack size (bytes)
      NULL,               // Parameter
      1,                  // Task priority
      NULL,               // Task handle
      1);

  xTaskCreatePinnedToCore(
      sensor_ph_task,
      "sensor_ph_task", // Task name
      2048,             // Stack size (bytes)
      NULL,             // Parameter
      1,                // Task priority
      NULL,             // Task handle
      1);

  xTaskCreatePinnedToCore(
      sensor_watherTemp_task,
      "sensor_watherTemp_task", // Task name
      2048,                     // Stack size (bytes)
      NULL,                     // Parameter
      1,                        // Task priority
      NULL,                     // Task handle
      1);

  // xTaskCreatePinnedToCore(
  //     mqttPublish,
  //     "mqttPublish", // Task name
  //     5000,          // Stack size (bytes)
  //     NULL,          // Parameter
  //     1,             // Task priority
  //     NULL,          // Task handle
  //     0);

  // Connect to the MQTT broker
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000)
  {
    lastMsg = now;
    snprintf(msg, MSG_BUFFER_SIZE, "co2 New #%ld", (const char *)co2_Input_Contraction);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("codersid/nodemcu/v1", msg);
  }
}