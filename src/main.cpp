#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <math.h>

Adafruit_SHT31 sht31 = Adafruit_SHT31();

const char *mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

const char *MQTT_PUB_CO2input = "esp32/mh-z14a/input";
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
#define turbidityPin GPIO_NUM_27 // pin turbidity

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
float ph_value;
float wather_Temp, air_temp, air_humidity, turbidity_ntu;

int adc = 0;
float volt;

float readTbd()
{
  int adc5;
  adc = analogRead(turbidityPin);
  adc5 = map(adc, 0, 4095, 0, 1023);
  volt = 0;
  for (int i = 0; i < 1000; i++)
  {
    volt += ((float)adc5 / 1023.0) * 5.0;
  }
  volt = (volt / 1000); //-0.159
  volt = round_to_dp(volt, 2);
  if (volt < 2.5)
  {
    turbidity_ntu = 3000;
  }
  else
  {
    turbidity_ntu = -1120.4 * square(volt) + 5742.3 * volt - 4353.8;
    //    Serial.println(turbidity_ntu);
    if (turbidity_ntu < 0)
    {
      turbidity_ntu = 0.0;
      return turbidity_ntu;
    }
    else
    {
      turbidity_ntu = turbidity_ntu;
      return turbidity_ntu;
    }
  }

  delay(1000);
}

void sensor_turbidity_task(void *Parameters)
{

  char myChar[10];
  const char *turbidity_value;

  while (true)
  {
    //    Serial.println(readTbd());
    dtostrf(readTbd(), 6, 2, myChar);
    turbidity_value = myChar;
    if (!client.connected())
    {
      reconnect(7, turbidity_value);
    }
    client.publish(MQTT_PUB_tbd, turbidity_value);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
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

  char char_air_temp[10];
  const char *air_temp_Value;
  char char_air_humidity[10];
  const char *air_humidity_Value;
  while (true)
  {
    air_temp = sht31.readTemperature();
    dtostrf(air_temp, 6, 2, char_air_temp);
    air_temp_Value = char_air_temp;

    air_humidity = sht31.readHumidity();
    dtostrf(air_humidity, 6, 2, char_air_humidity);
    air_humidity_Value = char_air_humidity;

    if (!client.connected())
    {
      reconnect(5, air_temp_Value);
      reconnect(6, air_humidity_Value);
    }
    client.publish(MQTT_PUB_AirTemp, air_temp_Value);
    client.publish(MQTT_PUB_rh, air_humidity_Value);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

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

void reconnect(int number, const char *valueSensor)
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      if (number == 1)
      {
        client.publish(MQTT_PUB_CO2input, valueSensor);
      }
      else if (number == 2)
      {
        client.publish(MQTT_PUB_CO2output, valueSensor);
      }
      else if (number == 3)
      {
        client.publish(MQTT_PUB_ph, valueSensor);
      }
      else if (number == 4)
      {
        client.publish(MQTT_PUB_WatherTemp, valueSensor);
      }
      else if (number == 5)
      {
        client.publish(MQTT_PUB_AirTemp, valueSensor);
      }
      else if (number == 6)
      {
        client.publish(MQTT_PUB_rh, valueSensor);
      }
      else if (number == 7)
      {
        client.publish(MQTT_PUB_tbd, valueSensor);
      }
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void sensor_watherTemp_task(void *Parameters)
{
  sensors.begin();
  char myChar[10];
  const char *water_tem_value;

  while (true)
  {
    sensors.requestTemperatures();
    wather_Temp = sensors.getTempCByIndex(0);
    dtostrf(wather_Temp, 6, 2, myChar);
    water_tem_value = myChar;
    if (!client.connected())
    {
      reconnect(4, water_tem_value);
    }
    client.publish(MQTT_PUB_WatherTemp, water_tem_value);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void sensor_ph_task(void *Parameter)
{
  int pHArray[ArrayLenth]; // Store the average value of the sensor feedback
  int pHArrayIndex = 0;
  static float voltage;
  char myChar[10];
  const char *pHValue;
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
      ph_value = 3.5 * voltage + Offset;
      dtostrf(ph_value, 6, 2, myChar);
      pHValue = myChar;
      samplingTime = millis();
    }

    if (!client.connected())
    {
      reconnect(3, pHValue);
    }
    client.publish(MQTT_PUB_ph, pHValue);
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
    String my_str = String(co2_Input_Contraction);
    const char *my_const_char_value = my_str.c_str();

    if (!client.connected())
    {
      reconnect(1, my_const_char_value);
    }
    client.publish(MQTT_PUB_CO2input, my_const_char_value);

    vTaskDelay(pdMS_TO_TICKS(3000)); // wait for 1 second before reading again
  }
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
  Serial.println(ph_value);
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
  // vTaskDelay(180000);

  if (!sht31.begin(0x44))
  {
    Serial.println("Couldn't find SHT31");
    while (1)
      delay(1);
  }

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
      6000,               // Stack size (bytes)
      NULL,               // Parameter
      1,                  // Task priority
      NULL,               // Task handle
      1);

  xTaskCreatePinnedToCore(
      mh_z14a_Old_task,
      "mh_z14a_Old_task", // Task name
      6000,               // Stack size (bytes)
      NULL,               // Parameter
      1,                  // Task priority
      NULL,               // Task handle
      0);

  xTaskCreatePinnedToCore(
      sensor_ph_task,
      "sensor_ph_task", // Task name
      2048,             // Stack size (bytes)
      NULL,             // Parameter
      1,                // Task priority
      NULL,             // Task handl
      1);

  // xTaskCreatePinnedToCore(
  //     sensor_watherTemp_task,
  //     "sensor_watherTemp_task", // Task name
  //     3048,                     // Stack size (bytes)
  //     NULL,                     // Parameter
  //     1,                        // Task priority
  //     NULL,                     // Task handle
      0);
      //
      xTaskCreatePinnedToCore(
          sensor_sht3x_task,
          "sensor_sht3x_task", // Task name
          4000,                // Stack size (bytes)
          NULL,                // Parameter
          1,                   // Task priority
          NULL,                // Task handle
          1);

  xTaskCreatePinnedToCore(
      sensor_turbidity_task,
      "sensor_turbidity_task", // Task name
      3048,                    // Stack size (bytes)
      NULL,                    // Parameter
      1,                       // Task priority
      NULL,                    // Task handl
  //     1);

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
void loop()
{
  client.loop();
}