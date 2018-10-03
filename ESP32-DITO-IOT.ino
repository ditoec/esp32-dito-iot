#include <WiFi.h>
#include <WiFiMulti.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Adafruit_BME280.h"
#include <HTTPClient.h>

#define I2C_SDA 2
#define I2C_SCL 15
#define BME280_ADDRESS 0x76  //If the sensor does not work, try the 0x77 address as well
#define ALTITUDE 11.0 // Altitude in Bristol

#define RELAYPIN 18

/************************* WiFi Access Point *********************************/
#define WLAN_SSID       "Britannia Cahya"
#define WLAN_PASS       "Britannia1912"

/************************* MQTT Setup *********************************/
#define MQTT_SERVER      "mqtt://things.ubidots.com"
#define MQTT_PORT        1883                   
#define MQTT_USERNAME    "A1E-JfqlAFGccP1gsFCmvrcSmpUPJqonQC"
#define MQTT_KEY         ""

WiFiMulti wifiMulti;
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'temp' and 'humidity' for publishing.
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, "/v1.6/devices/ESP32/Temperature");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, "/v1.6/devices/ESP32/Humidity");
Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, "/v1.6/devices/ESP32/Pressure");

Adafruit_MQTT_Subscribe dehumidifier = Adafruit_MQTT_Subscribe(&mqtt, "/v1.6/devices/esp32/dehumidifier/lv");
Adafruit_MQTT_Subscribe humidityControl = Adafruit_MQTT_Subscribe(&mqtt, "/v1.6/devices/esp32/humiditycontrol/lv");

Adafruit_BME280 bme(I2C_SDA, I2C_SCL);

float temp_val = 0;
float humidity_val = 0;
float pressure_val = 0;
float temp_acc = 0;
float humidity_acc = 0;
float pressure_acc = 0;
float temp_av = 0;
float humidity_av = 0;
float pressure_av = 0;
int humidity_setpoint = 60;
int dehumidifier_status = 1;

void initSensor()
{
  bool status = bme.begin(BME280_ADDRESS);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void dehumidifier_callback(uint32_t value)
{
  Serial.print(F("Got Dehumidifier: "));
  Serial.println((char *)dehumidifier.lastread);
  dehumidifier_status = atoi((char *)dehumidifier.lastread);
  if(humidity_av>humidity_setpoint && dehumidifier_status == 1)digitalWrite(RELAYPIN, HIGH);// set relay high
  else digitalWrite(RELAYPIN, LOW);
}

void humidityControl_callback(uint32_t value)
{
  Serial.print(F("Got HumidityControl: "));
  Serial.println((char *)humidityControl.lastread);
  humidity_setpoint = atoi((char *)humidityControl.lastread);
  if(humidity_av>humidity_setpoint && dehumidifier_status == 1)digitalWrite(RELAYPIN, HIGH);// set relay high
  else digitalWrite(RELAYPIN, LOW);}

void readSensor(){
  float humidity_new = bme.readHumidity();
  if(!isnan(humidity_new))humidity_val = humidity_new;

  float temp_new = bme.readTemperature();
  if(!isnan(temp_new))temp_val = temp_new;

  float pressure_new = bme.readPressure();
  if(!isnan(pressure_new)){
    pressure_val = pressure_new;
    pressure_val = bme.seaLevelForAltitude(ALTITUDE,pressure_val);
    pressure_val = pressure_val/100.0F;
  }

  Serial.print("Reading BME280 OK: ");
  Serial.print(temp_val); Serial.print(" *C, "); 
  Serial.print(humidity_val); Serial.print(" H ");
  Serial.print(pressure_val); Serial.println(" hPa");

  temp_acc+=temp_val;
  pressure_acc+=pressure_val;
  humidity_acc+=humidity_val;
}

void setup()
{
    Serial.begin(115200);
    delay(10);

    initSensor();
    
    delay(1000);

    pinMode(RELAYPIN, OUTPUT);      // set relay pin to output
    digitalWrite(RELAYPIN, LOW);    // set relay low

    wifiMulti.addAP(WLAN_SSID,WLAN_PASS);

    Serial.print("Connecting Wifi...");
    while(wifiMulti.run() != WL_CONNECTED) {
      Serial.print(".");
      delay(4000);
    }
    if(wifiMulti.run() == WL_CONNECTED){
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }

    // Setup MQTT subscription callback
    humidityControl.setCallback(humidityControl_callback);
    dehumidifier.setCallback(dehumidifier_callback);
  
    // Setup MQTT subscription
    mqtt.subscribe(&humidityControl);
    mqtt.subscribe(&dehumidifier);

    readSensor();

    temp_av=temp_val;
    pressure_av=pressure_val;
    humidity_av=humidity_val;

    if(humidity_av>humidity_setpoint && dehumidifier_status == 1)digitalWrite(RELAYPIN, HIGH);// set relay high
    else digitalWrite(RELAYPIN, LOW);
    
    temp_acc=0;
    pressure_acc=0;
    humidity_acc=0;
}

void loop()
{
  wifiMulti.run();
  
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).
  MQTT_connect();

  readSensor();
  mqtt.processPackets(5000);
  readSensor();
  mqtt.processPackets(5000);
  readSensor();
  mqtt.processPackets(5000);
  readSensor();
  mqtt.processPackets(5000);

  temp_av = temp_acc/4;
  humidity_av = humidity_acc/4;
  pressure_av = pressure_acc/4;

  temp_acc=0;
  pressure_acc=0;
  humidity_acc=0;

  if(humidity_av>humidity_setpoint && dehumidifier_status == 1)digitalWrite(RELAYPIN, HIGH);    // set relay high
  else digitalWrite(RELAYPIN, LOW);
  
  boolean temp_ok = temp.publish(temp_av);
  boolean humidity_ok = humidity.publish(humidity_av);
  boolean pressure_ok = pressure.publish(pressure_av);
  
  if (temp_ok && humidity_ok && pressure_ok){
    Serial.println("Success publishing temp & humidity data!");
  }
  else{
    Serial.println("Failed publishing temp & humidity data!");
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         //while (1);
         break;
       }
  }
  Serial.println("MQTT Connected!");
}
