#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char *ssid = "Neurabot";
const char *password = "KempulKentul!";

#define pwmPin GPIO_NUM_33
#define pwmPin2 GPIO_NUM_32

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
}

void setup()
{
  // initialize LEDC and other setup code
  Serial.begin(112500);

  xTaskCreatePinnedToCore(
      mh_z14a_task,
      "mh_z14a_task", // Task name
      4096,           // Stack size (bytes)
      NULL,           // Parameter
      1,              // Task priority
      NULL,           // Task handle
      app_cpu);

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
      4096,            // Stack size (bytes)
      NULL,            // Parameter
      1,               // Task priority
      NULL,            // Task handle
      pro_cpu);
}
void loop()
{
  // other loop code
}
