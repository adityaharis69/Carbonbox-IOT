#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define pwmPin GPIO_NUM_33
#define pwmPin2 GPIO_NUM_32

const char *ssid = "Neurabot";
const char *password = "Kempul4321!";

#define WIFI_TIMEOUT_MS 20000      // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt

static const BaseType_t app_cpu = 0;
static const BaseType_t pro_cpu = 1;
int *co2_Input_Contraction;
int *co2_Ouput_Contraction;

void mh_z14a_task(void *Parameters)
{
  int co2;
  co2_Input_Contraction = &co2;
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
    co2 = int(ppm);
    co2_Input_Contraction = &co2;
    vTaskDelay(3000 / portTICK_PERIOD_MS); // wait for 1 second before reading again
  }
  // vTaskDelete(NULL);
}

void mh_z14a_task2(void *Parameters)
{
  int co2;
  co2_Ouput_Contraction = &co2;
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
    co2 = int(ppm);
    co2_Ouput_Contraction = &co2;
    vTaskDelay(3000 / portTICK_PERIOD_MS); // wait for 1 second before reading again
  }
  // vTaskDelete(NULL);
}

void prinDataSensor(void *Parameters)
{

  while (true)
  {
    /* code */
    // Serial.print("CO2 INPUT: ");
    Serial.print(*co2_Input_Contraction);
    Serial.print("\t\t");
    // Serial.print("CO2 OUTPUT: ");
    Serial.println(*co2_Ouput_Contraction);
    vTaskDelay(9000 / portTICK_PERIOD_MS);
  }
  // vTaskDelete(NULL);
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

    // When we couldn't make a WiFi connection (or the timeout expired)
    // sleep for a while and then retry.
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
  vTaskDelay(3000);

  Serial.println(WiFi.localIP());
  xTaskCreatePinnedToCore(
      keepWiFiAlive,
      "keepWiFiAlive", // Task name
      5000,            // Stack size (bytes)
      NULL,            // Parameter
      1,               // Task priority
      NULL,            // Task handle
      ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
      mh_z14a_task,
      "mh_z14a_task", // Task name
      4098,           // Stack size (bytes)
      NULL,           // Parameter
      1,              // Task priority
      NULL,           // Task handle
      pro_cpu);

  xTaskCreatePinnedToCore(
      prinDataSensor,
      "prinDataSensor", // Task name
      1024,             // Stack size (bytes)
      NULL,             // Parameter
      1,                // Task priority
      NULL,             // Task handle
      pro_cpu);

  xTaskCreatePinnedToCore(
      mh_z14a_task2,
      "mh_z14a_task2", // Task name
      4098,            // Stack size (bytes)
      NULL,            // Parameter
      1,               // Task priority
      NULL,            // Task handle
      pro_cpu);

  // vTaskStartScheduler();
}
void loop()
{
  // other loop code
}
