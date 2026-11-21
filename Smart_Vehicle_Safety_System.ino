#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <VL53L1X.h>

// =================================
// COMPONENT INITIALIZATION
// =================================

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
VL53L1X roadSensor;

HardwareSerial GPS(2);
HardwareSerial GSM(1);

// =================================
// ESP32 PIN CONFIGURATION
// =================================

#define EYE_SENSOR 34
#define BUZZER 25
#define CANCEL_BUTTON 32

#define GPS_RX 16
#define GPS_TX 17

#define GSM_RX 26
#define GSM_TX 27

// =================================
// SYSTEM VARIABLES
// =================================

// Accident detection
float ACCIDENT_THRESHOLD = 3.5;
bool accidentDetected = false;

unsigned long accidentTime = 0;
unsigned long lastAccident = 0;

const unsigned long CANCEL_TIME = 20000;

// Drowsiness detection
unsigned long eyeClosedTime = 0;

bool drowsy = false;

const unsigned long WARNING_TIME = 1500;
const unsigned long DANGER_TIME = 2500;

// Road detection
float normalRoadDistance = 0;

float potholeThreshold = 80;
float raisedRoadThreshold = 60;

bool roadHazard = false;

unsigned long lastRoadAlert = 0;

// =================================
// BUZZER CONTROL
// =================================

void beep(int duration)
{
    digitalWrite(BUZZER, HIGH);
    delay(duration);

    digitalWrite(BUZZER, LOW);
}

// =================================
// GPS LOCATION
// =================================

String getLocation()
{
    unsigned long start = millis();

    while (millis() - start < 15000)
    {
        while (GPS.available())
        {
            gps.encode(GPS.read());
        }

        if (gps.location.isValid())
        {
            String latitude =
                String(gps.location.lat(), 6);

            String longitude =
                String(gps.location.lng(), 6);

            return "https://maps.google.com/?q="
                   + latitude + "," + longitude;
        }
    }

    return "GPS location unavailable";
}

// =================================
// GSM SMS FUNCTION
// =================================

void sendSMS(String message)
{
    GSM.println("AT");
    delay(500);

    GSM.println("AT+CMGF=1");
    delay(500);

    GSM.println("AT+CMGS=\"+91XXXXXXXXXX\"");
    delay(1000);

    GSM.print(message);

    GSM.write(26);

    delay(5000);

    Serial.println("Emergency SMS command sent");
}

// =================================
// ACCIDENT DETECTION
// =================================

void accidentDetection()
{
    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    float acceleration = sqrt(
        a.acceleration.x * a.acceleration.x +
        a.acceleration.y * a.acceleration.y +
        a.acceleration.z * a.acceleration.z
    );

    float accelerationG = acceleration / 9.80665;

    Serial.print("Acceleration: ");
    Serial.println(accelerationG);

    if (accelerationG > ACCIDENT_THRESHOLD &&
        millis() - lastAccident > 30000)
    {
        accidentDetected = true;

        accidentTime = millis();

        lastAccident = millis();

        Serial.println("POSSIBLE ACCIDENT DETECTED!");

        beep(500);
    }

    if (accidentDetected)
    {
        Serial.println("Accident alert countdown");

        if (digitalRead(CANCEL_BUTTON) == LOW)
        {
            accidentDetected = false;

            Serial.println("Accident alert cancelled");

            beep(200);

            return;
        }

        if (millis() - accidentTime >= CANCEL_TIME)
        {
            String location = getLocation();

            String message =
                "EMERGENCY! Possible vehicle accident detected. "
                "Vehicle location: " + location;

            sendSMS(message);

            accidentDetected = false;
        }
    }
}

// =================================
// DROWSINESS DETECTION
// =================================

void drowsinessDetection()
{
    // Assumes LOW means eyes closed.
    // Verify the logic of your sensor.

    int eyeState = digitalRead(EYE_SENSOR);

    if (eyeState == LOW)
    {
        if (eyeClosedTime == 0)
        {
            eyeClosedTime = millis();
        }

        unsigned long closedDuration =
            millis() - eyeClosedTime;

        if (closedDuration >= WARNING_TIME)
        {
            drowsy = true;

            Serial.println("WARNING: DRIVER DROWSY!");

            if (closedDuration >= DANGER_TIME)
            {
                digitalWrite(BUZZER, HIGH);
            }
            else
            {
                digitalWrite(BUZZER, LOW);
            }
        }
    }

    else
    {
        eyeClosedTime = 0;

        drowsy = false;
    }
}

// =================================
// ROAD HAZARD DETECTION
// =================================

void roadHazardDetection()
{
    uint16_t distance = roadSensor.read();

    if (roadSensor.timeoutOccurred())
    {
        return;
    }

    Serial.print("Road Distance: ");

    Serial.println(distance);

    float difference =
        distance - normalRoadDistance;

    if (difference > potholeThreshold)
    {
        roadHazard = true;

        Serial.println("POTHOLE DETECTED!");

        digitalWrite(BUZZER, HIGH);
    }

    else if (difference < -raisedRoadThreshold)
    {
        roadHazard = true;

        Serial.println("RAISED ROAD DETECTED!");

        digitalWrite(BUZZER, HIGH);
    }

    else
    {
        roadHazard = false;
    }

    if (roadHazard)
    {
        lastRoadAlert = millis();
    }

    if (!roadHazard &&
        millis() - lastRoadAlert > 500)
    {
        digitalWrite(BUZZER, LOW);
    }
}

// =================================
// MAIN SETUP
// =================================

void setup()
{
    Serial.begin(115200);

    Wire.begin(21, 22);

    pinMode(EYE_SENSOR, INPUT);

    pinMode(BUZZER, OUTPUT);

    pinMode(CANCEL_BUTTON, INPUT_PULLUP);

    digitalWrite(BUZZER, LOW);

    GPS.begin(9600, SERIAL_8N1,
              GPS_RX, GPS_TX);

    GSM.begin(115200, SERIAL_8N1,
              GSM_RX, GSM_TX);

    // MPU6050 initialization

    if (!mpu.begin())
    {
        Serial.println("MPU6050 NOT FOUND!");

        while (1);
    }

    Serial.println("MPU6050 Connected");

    // VL53L1X initialization

    roadSensor.setTimeout(100);

    if (!roadSensor.init())
    {
        Serial.println("VL53L1X NOT FOUND!");

        while (1);
    }

    roadSensor.setDistanceMode(VL53L1X::Long);

    roadSensor.setMeasurementTimingBudget(50000);

    roadSensor.startContinuous(50);

    Serial.println("ROAD SENSOR CONNECTED");

    // Road sensor calibration

    Serial.println("Calibrating road sensor...");

    delay(2000);

    long totalDistance = 0;

    int samples = 0;

    for (int i = 0; i < 20; i++)
    {
        uint16_t d = roadSensor.read();

        if (!roadSensor.timeoutOccurred() && d > 0)
        {
            totalDistance += d;

            samples++;
        }

        delay(60);
    }

    if (samples > 0)
    {
        normalRoadDistance =
            (float)totalDistance / samples;
    }

    Serial.print("Normal Road Distance: ");

    Serial.println(normalRoadDistance);

    Serial.println("SMART VEHICLE SAFETY SYSTEM READY");
}

// =================================
// MAIN LOOP
// =================================

void loop()
{
    // Continuously process GPS data

    while (GPS.available())
    {
        gps.encode(GPS.read());
    }

    // Three independent safety systems

    accidentDetection();

    drowsinessDetection();

    roadHazardDetection();

    delay(50);
}