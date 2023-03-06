#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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

void setup()
{
  // initialize LEDC and other setup code
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
}
void loop()
{
  // other loop code
  prinDataSensor();
  delay(9000);
}
