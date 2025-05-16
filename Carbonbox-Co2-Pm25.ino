/*
Carbonbox Read Co2 and Pm2.5
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi Configuration
const char* ssid = "NeuPilah";
const char* password = "GladeKalengGede";

// MQTT ThingsBoard Configuration
const char* mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;
const char* mqtt_device_id = "ul4qx5cdeqxon3pqgdm9";
const char* mqtt_username = "nyrk91jwqpo18vxxpi9v";
const char* mqtt_password = "4k1blpa5jwq1qsrndq8i";
const char* mqtt_topic = "v1/devices/me/telemetry";

// Serial Communication
#define MHZ14_SERIAL Serial1
#define RX_PIN 16
#define TX_PIN 17

byte cmdReadCO2[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte cmdCalibrateZero[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
byte response[9];

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long previousSensorMillis = 0;
const long sensorInterval = 5000;
unsigned long previousMqttMillis = 0;
const long mqttInterval = 6000;

int totalCO2 = 0;
int readingCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==========================================");
  Serial.println("Sensor CO2 MH-Z14 dengan MQTT ThingsBoard");
  Serial.println("ESP32-S2 Mini");
  Serial.println("==========================================");

  MHZ14_SERIAL.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  setupWifi();
  client.setServer(mqtt_server, mqtt_port);

  Serial.println("Menunggu sensor melakukan pemanasan...");
  for (int i = 180; i > 0; i--) {
    Serial.print(".");
    if (i % 10 == 0) {
      Serial.println();
      Serial.print(" tersisa: ");
      Serial.print(i);
      Serial.println(" detik");
    }
    delay(1000);
  }
  Serial.println("\nSensor siap digunakan!");
}

void loop() {
  if (!client.connected()) {
    reconnectMqtt();
  }
  client.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;
    int co2Value = readCO2();

    if (co2Value >= 0) {
      Serial.print("Kadar CO2: ");
      Serial.print(co2Value);
      Serial.println(" ppm");

      interpretCO2Level(co2Value);
      totalCO2 += co2Value;
      readingCount++;
    } else {
      Serial.println("Gagal membaca data dari sensor!");
    }

    checkSerialCommand();
  }

  if (currentMillis - previousMqttMillis >= mqttInterval && readingCount > 0) {
    previousMqttMillis = currentMillis;
    int avgCO2 = totalCO2 / readingCount;
    sendDataToThingsBoard(avgCO2);
    totalCO2 = 0;
    readingCount = 0;
  }
}

void setupWifi() {
  Serial.print("Menghubungkan ke WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi terhubung");
    Serial.print("Alamat IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Gagal terhubung ke WiFi.");
  }
}

void reconnectMqtt() {
  while (!client.connected()) {
    Serial.print("Mencoba terhubung ke MQTT broker...");
    if (client.connect(mqtt_device_id, mqtt_username, mqtt_password)) {
      Serial.println("terhubung!");
    } else {
      Serial.print("gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi dalam 5 detik");
      if (WiFi.status() != WL_CONNECTED) {
        setupWifi();
      }
      delay(5000);
    }
  }
}

void sendDataToThingsBoard(int co2Value) {
  DynamicJsonDocument doc(128);
  doc["co2"] = co2Value;
  doc["quality"] = getCO2QualityText(co2Value);
  doc["sensor"] = "MH-Z14";

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.print("Mengirim data ke ThingsBoard: ");
  Serial.println(jsonString);

  if (client.publish(mqtt_topic, jsonString.c_str())) {
    Serial.println("Data berhasil dikirim!");
  } else {
    Serial.println("Gagal mengirim data!");
  }
}

int readCO2() {
  while (MHZ14_SERIAL.available() > 0) {
    MHZ14_SERIAL.read();
  }

  MHZ14_SERIAL.write(cmdReadCO2, 9);
  delay(30);

  if (MHZ14_SERIAL.available() > 0) {
    int index = 0;
    unsigned long startTime = millis();

    while (index < 9 && (millis() - startTime) < 100) {
      if (MHZ14_SERIAL.available() > 0) {
        response[index++] = MHZ14_SERIAL.read();
      }
    }

    if (index == 9) {
      if (response[0] == 0xFF && response[1] == 0x86) {
        byte checksum = 0;
        for (int i = 1; i < 8; i++) {
          checksum += response[i];
        }
        checksum = 0xFF - checksum + 0x01;

        if (checksum == response[8]) {
          int co2Concentration = (response[2] * 256) + response[3];
          return co2Concentration;
        } else {
          Serial.println("Checksum tidak valid");
        }
      } else {
        Serial.println("Header tidak valid");
      }
    } else {
      Serial.println("Data tidak lengkap");
    }
  }

  return -1;
}

void interpretCO2Level(int ppm) {
  Serial.print("Kualitas udara: ");
  Serial.println(getCO2QualityText(ppm));

  if (ppm > 1000) {
    Serial.println("Rekomendasi: Buka jendela atau tingkatkan ventilasi.");
  }
}

String getCO2QualityText(int ppm) {
  if (ppm < 400) {
    return "Tidak normal (terlalu rendah)";
  } else if (ppm < 700) {
    return "Sangat baik";
  } else if (ppm < 1000) {
    return "Baik";
  } else if (ppm < 1500) {
    return "Cukup (ventilasi direkomendasikan)";
  } else if (ppm < 2500) {
    return "Buruk (ventilasi diperlukan)";
  } else {
    return "Sangat buruk (berbahaya dalam jangka lama)";
  }
}

void calibrateZeroPoint() {
  Serial.println("Kalibrasi Zero Point dimulai...");
  Serial.println("Pastikan sensor berada di udara segar selama minimal 20 menit.");
  MHZ14_SERIAL.write(cmdCalibrateZero, 9);
  delay(100);
  Serial.println("Kalibrasi selesai.");
}

void checkSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("calibrate") || command.equalsIgnoreCase("cal")) {
      calibrateZeroPoint();
    } else if (command.equalsIgnoreCase("help")) {
      printHelp();
    } else if (command.equalsIgnoreCase("info")) {
      printSensorInfo();
    }
  }
}

void printHelp() {
  Serial.println("\nPerintah yang tersedia:");
  Serial.println(" - help       : Menampilkan bantuan");
  Serial.println(" - info       : Menampilkan info sensor");
  Serial.println(" - calibrate  : Kalibrasi zero point (pastikan sensor di udara bersih)");
}

void printSensorInfo() {
  Serial.println("\nInformasi Sensor:");
  Serial.println(" - Tipe Sensor  : MH-Z14");
  Serial.println(" - Rentang CO2  : 0 - 5000 ppm");
  Serial.println(" - Akurasi      : ±50 ppm atau ±5% dari pembacaan");
  Serial.println(" - Komunikasi   : UART (9600 bps)");
}
