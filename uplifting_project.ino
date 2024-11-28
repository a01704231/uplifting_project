#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <PubSubClient.h>

Adafruit_ADS1115 ads;
const char* ssid = "eti-lab_2.4GHz";
const char* password = "unwisdom-salue-topotype";
const char* mqtt_server = "192.168.1.128";
WiFiClient espClient;
PubSubClient client(espClient);
const int pwmPin = 18;
const int valvePin = 19;
const int presion_obj1 = 132;
const int presionLimite = 135;
float pressure_kPa;
float error, u, z = 0, u2;
int Upwm;
const float Kp = 15.0;
const float Ki = 12.0;
int cerrar1 = 0;
int16_t adc0;
float voltage;
unsigned long start;
unsigned long end;
bool begin;

void setup_wifi(){
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length){
  String messageTemp;
  for (int i = 0; i < length; i++){
    messageTemp += (char)message[i];
  }
  if (String(topic) == "gripper"){
    if (messageTemp == "close"){
      if (cerrar1 != 1){
        begin = true;
      }
      cerrar1 = 1;
    }
    else if (messageTemp == "open"){
      if (cerrar1 != 0){
        begin = true;
      }
      cerrar1 = 0;
    }
  }
}

void reconnect(){
  while (!client.connected()){
    if (client.connect("ESPClient_bomba")){
      client.subscribe("gripper");
    }
    else{
      delay(5000);
    }
  }
}

void setup(){
  pinMode(pwmPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  Serial.begin(115200);
  Serial.println("Control de Presión con Sensor XGZP6847A");
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  if (!ads.begin()){
    Serial.println("Error al inicializar el ADS1115.");
    while (1);
  }
  Serial.println("ADS1115 iniciado con éxito.");
}

void loop(){
  if (!client.connected()){
    reconnect();
  }
  client.loop();
  if (cerrar1 == 1){
    adc0 = ads.readADC_SingleEnded(0);
    voltage = adc0 * 0.1875 / 1000;
    pressure_kPa = (voltage - 0.5) * 100 / 4.0;
    pressure_kPa = pressure_kPa ;
    error = presion_obj1 - pressure_kPa;
    u = Kp * error;
    Upwm = -0.0073 * (u * u) + 2.9488 * u - 185.29;
    if (begin){
      z = 0.006 * error;
    }
    else{
      z = 0.006 * error + z;
    }
    u2 = Upwm + Kp * error + Ki * z;
    analogWrite(pwmPin, u2);
    digitalWrite(valvePin, HIGH);
    end = millis();
    if (begin){
      Serial.println("cerrar");
      Serial.print("adc0 ");
      Serial.print(adc0);
      Serial.print(" voltage "); 
      Serial.print(voltage); 
      Serial.print(" pressute_kPa "); 
      Serial.print(pressure_kPa); 
      Serial.print(" error "); 
      Serial.print(error); 
      Serial.print(" u "); 
      Serial.print(u); 
      Serial.print(" Upwm "); 
      Serial.print(Upwm); 
      Serial.print(" z "); 
      Serial.print(z); 
      Serial.print(" u2 ");
      Serial.print(u2);
      begin = false;
    }
    if (end - start >= 1000){
      Serial.println();
      Serial.println(u2);
      Serial.print("ADC: ");
      Serial.println(adc0);
      Serial.print("voltageSensor: ");
      Serial.println(voltage);
      Serial.print("pressure_kPa: ");
      Serial.println(pressure_kPa);
      client.publish("pressure", String(pressure_kPa).c_str());
      start = millis();
    }
    if (pressure_kPa > presionLimite){
      cerrar1 = 0;
      error = 0;
      analogWrite(pwmPin, 0);
      digitalWrite(valvePin, LOW);
      Serial.println("Presión excede 160 kPa. Bomba apagada.");
      Serial.println(pressure_kPa);
    }
  }
  else if (cerrar1 == 0){
    if (begin){
      adc0 = ads.readADC_SingleEnded(0);
      voltage = adc0 * 0.1875 / 1000;
      pressure_kPa = (voltage - 0.5) * 100 / 4.0;
      pressure_kPa = pressure_kPa ;
      Serial.println("abrir");
      analogWrite(pwmPin, 0);
      digitalWrite(valvePin, LOW);
      Serial.println("Bomba apagada. Válvula cerrada.");
      Serial.print("Presión ajustada (kPa): ");
      Serial.println(pressure_kPa);
      client.publish("presion", String(pressure_kPa).c_str());
      begin = false;
    }
  }
}
