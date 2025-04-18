#include <DHT.h>
#include<SAMDTimerInterrupt.h>

// === Pin Assignments ===
#define BUTTON_PIN 2
#define LED1_PIN 7     // Button-controlled LED
#define LED2_PIN 6     // Ultrasonic-controlled LED
#define LED3_PIN 5     // Timer-controlled LED
#define TRIG_PIN 8     // Ultrasonic TRIG
#define ECHO_PIN 3     // Ultrasonic ECHO
#define DHT_PIN 4      // DHT11 sensor
#define DHTTYPE DHT11

DHT dht(DHT_PIN, DHTTYPE);

// === Variables ===
bool led1State = LOW;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

unsigned long previousMillis = 0;
const unsigned long timerInterval = 1000;

unsigned long lastDHTRead = 0;
const unsigned long dhtInterval = 2000;

bool objectDetected = false;

void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  dht.begin();

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);

  Serial.println("System Ready!");
}

void checkButton() {
  int reading = digitalRead(BUTTON_PIN);

  // Check for state change
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); // reset debounce timer
  }

  // Only toggle if enough time has passed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If state changed and button is now pressed (LOW)
    if (reading == LOW && currentButtonState == HIGH) {
      led1State = !led1State; // toggle LED state
      digitalWrite(LED1_PIN, led1State);
      Serial.println("Button Pressed → LED1 Toggled");
    }
    currentButtonState = reading;
  }

  lastButtonState = reading;
}

void handleUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  int distance = duration * 0.034 / 2;

  if (distance > 0 && distance < 15) {
    if (!objectDetected) {
      digitalWrite(LED2_PIN, !digitalRead(LED2_PIN));
      Serial.print("Ultrasonic: Object at ");
      Serial.print(distance);
      Serial.println(" cm → LED2 toggled");
      objectDetected = true;
    }
  } else {
    objectDetected = false;
  }
}

void readDHT() {
  float temp = dht.readTemperature();
  float humid = dht.readHumidity();

  if (!isnan(temp) && !isnan(humid)) {
    Serial.print("DHT Sensor: ");
    Serial.print(temp);
    Serial.print(" °C, ");
    Serial.print(humid);
    Serial.println(" %");
  } else {
    Serial.println("DHT Sensor: Failed to read");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  checkButton();

  if (currentMillis - previousMillis >= timerInterval) {
    previousMillis = currentMillis;
    digitalWrite(LED3_PIN, !digitalRead(LED3_PIN));
    Serial.println("Timer tick → LED3 toggled");

    handleUltrasonic();
  }

  if (currentMillis - lastDHTRead >= dhtInterval) {
    lastDHTRead = currentMillis;
    readDHT();
  }
} 