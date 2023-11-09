#include <Arduino.h>
#include <DHT.h>

#define DHT_PIN 14      // D5
#define FAN_SPEED_PIN 2 // D4
#define IN1 4           // D2
#define IN2 0           // D3

// constants for PID parameters
const float kp = 30.0;  // Proportional gain
const float ki = 0.0001; // Integral gain
const float kd = 1.0;   // Derivative gain

const float setpointTemperature = 23.0; // 23 degrees celsius
float temperature, error, last_error, integral, derivative, fanspeed;

// Define variables for time-based control
unsigned long previousMillis = 0;
const long interval = 1000; // Time interval for control loop (1 second)

// initialize the DHT sensor
DHT dht(DHT_PIN, DHT11);

void setup()
{
  // initialize the baud rate
  Serial.begin(115200);

  // initialize the DHT sensor
  dht.begin();

  // set fan pins as fanspeed
  pinMode(FAN_SPEED_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // initialize the PID variables
  last_error = 0;
  integral = 0;
}

void loop()
{
  // Read temperature and humidity from DHT sensor
  float humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Calculate the error
  error = temperature - setpointTemperature;

  // Calculate the integral and derivative terms
  integral += error;
  derivative = error - last_error;

  // Calculate the control fanspeed using PID
  // fanspeed = abs(kp * error + ki * integral + kd * derivative);
  fanspeed = kp * error + ki * integral + kd * derivative;

  // Limit the control fanspeed to avoid full-speed fan
  if (fanspeed > 255)
  {
    fanspeed = 255;
  }
  else if (fanspeed < 0)
  {
    fanspeed = 0;
  }

  // Update the fan speed
  // set fan direction
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(FAN_SPEED_PIN, fanspeed);

  // Save the current error for the next iteration
  last_error = error;

  // Print temperature and control fanspeed
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Setpoint: ");
  Serial.print(setpointTemperature);
  Serial.print(" Proportional: ");
  Serial.print(error);
  Serial.print(" Integral: ");
  Serial.print(integral);
  Serial.print(" Derivative: ");
  Serial.print(derivative);
  Serial.print(" °C, Fan Speed: ");
  Serial.println(fanspeed);
  // delay(500);

  // Add a delay to control the loop rate
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
  }
}