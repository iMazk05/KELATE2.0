#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//==============================
// WiFi
//==============================
const char* ssid = "ENTER_WIFI_SSID";
const char* password = "ENTER_WIFI_PASSWORD";

// MQTT Broker (Laptop IP)
const char* mqtt_server = "ENTER_LAPTOP_IP_ADDRESS";

WiFiClient espClient;
PubSubClient client(espClient);

#define M1A 18
#define M1B 19

#define M2A 21
#define M2B 22

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3

#define PWM_FREQ 1000
#define PWM_RES 8

void setMotor(int pwmA, int pwmB, float speed)
{
    speed = constrain(speed, -1.0, 1.0);

    int pwm = abs(speed) * 255;

    if(speed >= 0)
    {
        ledcWrite(pwmA, pwm);
        ledcWrite(pwmB, 0);
    }
    else
    {
        ledcWrite(pwmA, 0);
        ledcWrite(pwmB, pwm);
    }
}

void drive(float left, float right)
{
    setMotor(CH1, CH2, left);
    setMotor(CH3, CH4, right);
}

void callback(char* topic, byte* payload, unsigned int length)
{
    StaticJsonDocument<128> doc;

    DeserializationError err =
        deserializeJson(doc, payload, length);

    if(err)
        return;

    float throttle = doc["throttle"];
    float steering = doc["steering"];

    float left = throttle + steering;
    float right = throttle - steering;

    left = constrain(left,-1,1);
    right = constrain(right,-1,1);

    drive(left,right);

    Serial.print("Throttle: ");
    Serial.print(throttle);

    Serial.print(" Steering: ");
    Serial.print(steering);

    Serial.print(" Left: ");
    Serial.print(left);

    Serial.print(" Right: ");
    Serial.println(right);
}

void reconnect()
{
    while(!client.connected())
    {
        Serial.println("Connecting MQTT...");

        if(client.connect("ESP32Robot"))
        {
            Serial.println("Connected");
            client.subscribe("robot/cmd");
        }
        else
        {
            delay(1000);
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // PWM
    ledcSetup(CH1, PWM_FREQ, PWM_RES);
    ledcSetup(CH2, PWM_FREQ, PWM_RES);
    ledcSetup(CH3, PWM_FREQ, PWM_RES);
    ledcSetup(CH4, PWM_FREQ, PWM_RES);

    ledcAttachPin(M1A, CH1);
    ledcAttachPin(M1B, CH2);

    ledcAttachPin(M2A, CH3);
    ledcAttachPin(M2B, CH4);

    WiFi.begin(ssid,password);

    while(WiFi.status()!=WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi Connected");

    client.setServer(mqtt_server,1883);
    client.setCallback(callback);
}

void loop()
{
    if(!client.connected())
        reconnect();

    client.loop();
}