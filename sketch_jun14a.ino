#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 1
#define CS_PIN 5
#define OLED_SDA 21 // Replace with your SDA pin number
#define OLED_SCL 22 // Replace with your SCL pin number
#define OLED_ADDR 0x3C // Replace with the correct I2C address of your OLED display
Adafruit_SSD1306 display(128, 32, &Wire, OLED_ADDR);

MD_Parola P = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

#define FIREBASE_HOST "https://esp32-dedmo-default-rtdb.firebaseio.com/" 
#define FIREBASE_AUTH "API" 

#define WIFI_SSID "Galaxy S20 FE 5G3C7F"
#define WIFI_PASSWORD "dimf2980"

#define TRIGGER_PIN 26 // HC-SR04 trigger pin
#define ECHO_PIN 25    // HC-SR04 echo pin
#define LDR_PIN 34     // LDR analog pin
FirebaseData fbdo;

unsigned long updateInterval = 5000; // Default delay interval in milliseconds

void setup() {
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SUKA EMA SHELCHA:");
  display.display();

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  P.begin();

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

bool displayNeedsClearing = true;
bool dataFetched = false;
int distanceFromDB = 0;

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastDelayFetch = 0;
  unsigned long currentMillis = millis();

  // Fetch delay from Firebase every 5 seconds
  if (currentMillis - lastDelayFetch > 5000) {
    if (Firebase.getInt(fbdo, "/test/delay")) { // Fetch the delay from Firebase
      updateInterval = fbdo.intData() * 1000; // Convert to milliseconds if delay is in seconds
      Serial.print("Fetched delay: ");
      Serial.println(updateInterval);
      lastDelayFetch = currentMillis;
    } else {
      Serial.print("Failed to get delay from Firebase. Error: ");
      Serial.println(fbdo.errorReason());
    }
  }

  if (currentMillis - lastUpdate > updateInterval) { // Update every x seconds
    long duration, distance;
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration / 2) / 29.1; // calculate the distance
    Firebase.setInt(fbdo, "/test/distance", distance); // Send the distance to Firebase
    lastUpdate = currentMillis;

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Distance:");
    display.setCursor(0, 10);
    display.println(distance);
    display.display();
  }

  if (!dataFetched) {
    if (Firebase.getInt(fbdo, "/test/distance")) { // Fetch the distance from Firebase
      distanceFromDB = fbdo.intData();
      Serial.print("Data from Firebase: ");
      Serial.println(distanceFromDB);

      dataFetched = true;
    } else {
      Serial.print("Failed to get data from Firebase. Error: ");
      Serial.println(fbdo.errorReason());
    }
  }
}