//*------------------------------------------------------------------------------------------------------------------------
//* All-In-One Garage Controller on ESP8266 (WeMos D1 Mini)
//*
//* See https://automatedhome.party/2017/01/06/wifi-garage-door-controller-using-a-wemos-d1-mini/
//* https://github.com/tzapu/WiFiManager/blob/master/examples/AutoConnectWithFSParameters/AutoConnectWithFSParameters.ino
//* Home Assistant MQTT Discovery, see https://home-assistant.io/docs/mqtt/discovery/
//*------------------------------------------------------------------------------------------------------------------------

#include <FS.h>
#include <ESP8266WiFi.h>             //* ESP8266 Core WiFi Library - https://github.com/esp8266/Arduino
#include <DNSServer.h>               //* Local DNS Server to redirect requests to configuration portal
#include <ESP8266WebServer.h>        //* Local WebServer used to serve the configuration portal
#include <WiFiManager.h>             //* WiFi Configuration Magic - //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>             //* JSON - https://github.com/bblanchon/ArduinoJson

#define MQTT_MAX_PACKET_SIZE 512
#include <PubSubClient.h>            //* MQTT Client - https://github.com/knolleary/pubsubclient/releases/tag/v2.6

#include <ESP8266mDNS.h>             //* Multicast DNS Responder 
#include <ESP8266HTTPUpdateServer.h> //* Update Over-The-Air (OTA)

#include "pitches.h"
#include "D1MiniPins.h"

//************START CUSTOM PARAMETERS******************//

#include "secrets.h"

#if !defined OTA_USER
#define OTA_USER "admin"
#endif
#if !defined OTA_PASSWORD
#define OTA_PASSWORD "RedactedPassword"
#endif
#if !defined MQTT_USER
#define MQTT_USER "pi"
#endif
#if !defined MQTT_PASSWORD
#define MQTT_PASSWORD "RedactedPassword"
#endif

//*-----------------------------------
//* OTA Firmware Update parameters:
//*-----------------------------------

const char* our_host_name =     "GarageESP2";
String unique_client_name =  String(our_host_name) + "-" + String(ESP.getChipId()) + "-" + String(ESP.getFlashChipId());
const char* update_path =       "/WebFirmwareUpgrade";
//* these defaults can be updated on the config page:
char update_username[40] =      OTA_USER;
char update_password[40] =      OTA_PASSWORD;

//*---------------------------------------------------------------------------------
//* Define the magnetic door switch and relay pins for left (or only) garage door:
//*---------------------------------------------------------------------------------

// #define LEFT_RELAY_PIN Dx
#define LEFT_DOOR_PIN  D1

#define LEFT_DOOR_NAME          "Garage Door"
#define LEFT_DOOR_TOPIC         "garage/left/door"
#define DOOR_STATEOPEN              "Open"
#define DOOR_STATECLOSED            "Closed"
#define DOOR_STATEUNKNOWN           "Unknown"
#define DOOR_STATEALERT             "Alert"

String left_door_state = DOOR_STATEUNKNOWN;

//*---------------------------------------------------
//* Define the pins for right garage door (optional):
//*---------------------------------------------------

// #define RIGHT_RELAY_PIN Dx
// #define RIGHT_DOOR_PIN  Dx

#define RIGHT_DOOR_NAME         "Right Garage Door"
#define RIGHT_DOOR_TOPIC        "garage/right/door"

String right_door_state = DOOR_STATEUNKNOWN;

//*-----------------------------------
//* Passive Speaker Buzzer (optional)
//*-----------------------------------

// #define SPEAKER_PIN Dx
void sound_alert();
void signal_success();
void signal_error();

//*----------------------------------------------------
//* Digital Humidity and Temperature Sensor (optional)
//*----------------------------------------------------

// #define DHT_PIN  Dx             //* DHT-22 Pin connection
#define DHT_TYPE DHT22          //* DHT Type is DHT22 (AM2302)

#define HUMIDITY_NAME           "Garage Humidity"
#define HUMIDITY_TOPIC          "garage/humidity"
#define TEMPERATURE_NAME        "Garage Temperature"
#define TEMPERATURE_TOPIC       "garage/temperature"

//*----------------------------------------------------
//* Left Car HC-SR04 Distance Sensor (optional)
//*----------------------------------------------------

// #define ECHO_TRIGGER_PIN  Dx
// #define ECHO_DETECT_PIN   Dx

#define LEFT_CAR_NAME           "Garage Left Lane"
#define LEFT_CAR_TOPIC          "garage/left/car"

#ifdef ECHO_TRIGGER_PIN
#define CAR_LANE_OCCUPIED           "Detected"
#define CAR_LANE_EMPTY              "Clear"
#define CAR_LANE_UNKNOWN            "Unknown"

#endif

float min_distance_float;
float max_distance_float;
unsigned int max_distance_uint;
//* these defaults can be updated on the config page:
char min_distance[4] = "10";    //* minimum valid distance for detection
char max_distance[4] = "220";   //* maximum valid distance for detection

//*-----------------------------------------------------------------
//* Right Car HC-SR04 Distance Sensor (optional; define Left first)
//*-----------------------------------------------------------------

// #define ECHO_TRIGGER_PIN_RIGHT  Dx
// #define ECHO_DETECT_PIN_RIGHT   Dx

#define RIGHT_CAR_NAME          "Garage Right Lane"
#define RIGHT_CAR_TOPIC         "garage/right/car"

//*-----------------------------------
//* MQTT Parameters
//*-----------------------------------

#define COMMAND_TOPIC           "garage/command"
//* these defaults can be updated on the config page:
char mqtt_server[40] =          "10.9.8.7";
char mqtt_port[6] =             "1883";
char mqtt_user[40] =            MQTT_USER;
char mqtt_pass[40] =            MQTT_PASSWORD;
char mqttDiscoveryPrefix[40] =  "homeassistant";

//*-----------------------------------
//* PIR Motion Detector (optional)
//*-----------------------------------

#define MOTION_PIN D6

#define MOTION_NAME             "Garage Motion West"
#define MOTION_TOPIC            "garage/motion-west"
#define MOTION_DETECTED             "Detected"
#define MOTION_NOT_DETECTED         "Clear"

//************END CUSTOM PARAMS******************//

#ifdef DHT_PIN
//void get_humidity_and_temperature();
#include "DHT.h"                //* DHT Libraries from Adafruit; Dependant upon Adafruit_Sensors Library
DHT dht(DHT_PIN, DHT_TYPE);     //* Initialize DHT sensor for normal 16mhz Arduino 
#endif

#ifdef ECHO_TRIGGER_PIN
//* Include NewPing Library
#include "NewPingESP8266.h"
float soundcm =  0.0343;        //* Stores calculated speed of sound in cm/ms
int iterations = 3;             //* Number of pings to average together
unsigned long last_occupancy_check_millis = 0;
#endif

#ifdef ECHO_TRIGGER_PIN
//    NewPingESP8266 sonar1(ECHO_TRIGGER_PIN, ECHO_DETECT_PIN, max_distance_uint);
NewPingESP8266 sonar1(ECHO_TRIGGER_PIN, ECHO_DETECT_PIN, 500);

#ifdef ECHO_TRIGGER_PIN_RIGHT
//        NewPingESP8266 sonar2(ECHO_TRIGGER_PIN_RIGHT, ECHO_DETECT_PIN_RIGHT, max_distance_uint);
NewPingESP8266 sonar2(ECHO_TRIGGER_PIN_RIGHT, ECHO_DETECT_PIN_RIGHT, 500);
#endif
#endif

//* This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

//* Web Server Globals:
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

//* MQTT Globals:
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void mqtt_callback(char* topic, byte* payload, unsigned int length);
void connect_to_mqtt_broker();

//*-----------------------------------
//* WiFiManager Globals:
//*-----------------------------------

//* callback notifying us that config mode is being entered
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entering config mode...");
  //    Serial.println(WiFi.softAPIP());
  //    Serial.println(myWiFiManager->getConfigPortalSSID());
}

//* flag for saving data
bool shouldSaveConfig = false;

//* callback notifying us of successful connection and the need to save config
void saveConfigCallback () {
  Serial.println("saveConfigCallback");
  shouldSaveConfig = true;
}

//*------------------------------------------------------------------------
//* The setup function runs once when you press reset or power the board:
//*------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  delay(1000);

  Serial.println(__FILE__);
  Serial.print("Compile date: ");
  Serial.println(compile_date);

  Serial.println(unique_client_name);

  //* Set Relay(output) and Door(input) pins
#ifdef LEFT_RELAY_PIN
pinMode(LEFT_RELAY_PIN, LOW);
pinMode(LEFT_RELAY_PIN, OUTPUT);
pinMode(LEFT_RELAY_PIN, LOW);
#endif
#ifdef RIGHT_RELAY_PIN
pinMode(RIGHT_RELAY_PIN, LOW);
pinMode(RIGHT_RELAY_PIN, OUTPUT);
pinMode(RIGHT_RELAY_PIN, LOW);
#endif
#ifdef LEFT_DOOR_PIN
pinMode(LEFT_DOOR_PIN, INPUT_PULLUP);
#endif
#ifdef RIGHT_DOOR_PIN
pinMode(RIGHT_DOOR_PIN, INPUT_PULLUP);
#endif

#ifdef SPEAKER_PIN
pinMode(SPEAKER_PIN, OUTPUT);
pinMode(SPEAKER_PIN, LOW);
#endif

#ifdef DHT_PIN
dht.begin();
#endif

  //* clean FS, for testing
  // SPIFFS.format();

  //* read configuration from FS json
  Serial.println("Mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("FS mounted.");
    if (SPIFFS.exists("/config.json")) {
      //* file exists, reading and loading
      Serial.println("Reading config file...");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Opened config file.");
        size_t size = configFile.size();
        //* Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nParsed json.");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(mqttDiscoveryPrefix, json["mqttDiscoveryPrefix"]);
          strcpy(update_username, json["update_username"]);
          strcpy(update_password, json["update_password"]);
//#ifdef ECHO_TRIGGER_PIN
strcpy(min_distance, json["min_distance"]);
strcpy(max_distance, json["max_distance"]);
//#endif
} else {
Serial.println("Failed to load json config.");
}
configFile.close();
}
}
} else {
Serial.println("Failed to mount FS.");
}
//* end read

          //* The extra parameters to be configured (can be either global or just in the setup)
          //*                                             id/name     placeholder/prompt    default         length
          WiFiManagerParameter custom_mqtt_server(        "server",     "MQTT Server",        mqtt_server,    40);
          WiFiManagerParameter custom_mqtt_port(          "port",       "MQTT Port",          mqtt_port,      6);
          WiFiManagerParameter custom_mqtt_user(          "user",       "MQTT User",          mqtt_user,      40);
          WiFiManagerParameter custom_mqtt_pass(          "pass",       "MQTT Password",      mqtt_pass,      40);
          WiFiManagerParameter custom_mqttDiscoveryPrefix("discovery",  "Discovery Prefix",   mqttDiscoveryPrefix,      40);
          WiFiManagerParameter custom_update_username(    "username",   "OTA Update User",    update_username, 40);
          WiFiManagerParameter custom_update_password(    "password",   "OTA Update Password", update_password, 40);
//#ifdef ECHO_TRIGGER_PIN
WiFiManagerParameter custom_min_distance(   "mindistance", "Min Valid Distance", min_distance,   4);
WiFiManagerParameter custom_max_distance(   "maxdistance", "Max Valid Distance", max_distance,   4);
//#endif

          //* WiFiManager
          //* Local intialization. Once its business is done, there is no need to keep it around.
          WiFiManager wifiManager;

          //* set config save notify callback
          wifiManager.setSaveConfigCallback(saveConfigCallback);

          //* set static ip
          //    wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

          //* add all your parameters here
          wifiManager.addParameter(&custom_mqtt_server);
          wifiManager.addParameter(&custom_mqtt_port);
          wifiManager.addParameter(&custom_mqtt_user);
          wifiManager.addParameter(&custom_mqtt_pass);
          wifiManager.addParameter(&custom_mqttDiscoveryPrefix);
          wifiManager.addParameter(&custom_update_username);
          wifiManager.addParameter(&custom_update_password);
//#ifdef ECHO_TRIGGER_PIN
wifiManager.addParameter(&custom_min_distance);
wifiManager.addParameter(&custom_max_distance);
//#endif
//* ----------------------------
//* reset settings - for testing
//* ----------------------------

// wifiManager.resetSettings();

//* ----------------------------

          //* set minimum quality of signal so it ignores AP's under that quality
          //* defaults to 8%
          //wifiManager.setMinimumSignalQuality();

          //* set timeout until configuration portal stops waiting
          wifiManager.setConfigPortalTimeout(180);

          wifiManager.setAPCallback(configModeCallback);

          //* fetches saved WiFi ssid and password and tries to connect
          //* if it does not connect it starts an access point with unique_client_name
          //* and goes into a blocking loop awaiting configuration
          Serial.println("Attempting WiFi connection...");
          if (!wifiManager.autoConnect(unique_client_name.c_str(), update_password)) {
            Serial.println("Failed to connect.");
            signal_error();
            delay(3000);
            //reset and try again, or maybe put it to deep sleep
            ESP.reset();
            delay(5000);
          }

          //*if you get here you have connected to the WiFi:

          Serial.println("WiFi connected.");
          Serial.print("IP address: ");
          Serial.println(WiFi.localIP());

          //* read updated parameters:

          strcpy(mqtt_server, custom_mqtt_server.getValue());
          strcpy(mqtt_port, custom_mqtt_port.getValue());
          strcpy(mqtt_user, custom_mqtt_user.getValue());
          strcpy(mqtt_pass, custom_mqtt_pass.getValue());
          strcpy(mqttDiscoveryPrefix, custom_mqttDiscoveryPrefix.getValue());
          Serial.print("mqttDiscoveryPrefix: ");
          Serial.println(mqttDiscoveryPrefix);
          strcpy(update_username, custom_update_username.getValue());
          strcpy(update_password, custom_update_password.getValue());
//#ifdef ECHO_TRIGGER_PIN
strcpy(min_distance, custom_min_distance.getValue());
min_distance_float = atof(min_distance);
Serial.print("min_distance: ");
Serial.println(min_distance_float);

          strcpy(max_distance, custom_max_distance.getValue());
          max_distance_float = atof(max_distance);
          max_distance_uint = atoi(max_distance);
          Serial.print("max_distance: ");
          Serial.println(max_distance_float);
//#endif

          //* save the custom parameters to FS:

          if (shouldSaveConfig) {
            Serial.println("Saving config...");
            DynamicJsonBuffer jsonBuffer;
            JsonObject& json = jsonBuffer.createObject();
            json["mqtt_server"] = mqtt_server;
            json["mqtt_port"] = mqtt_port;
            json["mqtt_user"] = mqtt_user;
            json["mqtt_pass"] = mqtt_pass;
            json["mqttDiscoveryPrefix"] = mqttDiscoveryPrefix;
            json["update_username"] = update_username;
            json["update_password"] = update_password;
//#ifdef ECHO_TRIGGER_PIN
json["min_distance"] = min_distance;
json["max_distance"] = max_distance;
//#endif
File configFile = SPIFFS.open("/config.json", "w");
if (!configFile) {
Serial.println("Failed to open config file for writing.");
signal_error();
}

            json.printTo(Serial);
            json.printTo(configFile);
            configFile.close();
            //* end save
          }

          //* Set up the mqtt client:

          unsigned int mqtt_port_int = 1883;
          mqtt_port_int = atoi(mqtt_port);

          mqttClient.setServer(mqtt_server, mqtt_port_int);
          mqttClient.setCallback(mqtt_callback); //* mqtt_callback is the function that gets called for a topic sub
          connect_to_mqtt_broker();

          //* Publish configuration for Home Assistant:

          if (mqttDiscoveryPrefix != "") {
            String mqttStateTopic;
            String mqttStateTopicDashed;
            String mqttDiscoName;
            String mqttDiscoConfigTopic;
            String mqttDiscoConfigPayload;

            // The strings below will spill over the PubSubClient_MAX_PACKET_SIZE 128 and fail to get to the broker.
            // You'll need to manually set MQTT_MAX_PACKET_SIZE in PubSubClient.h to 512.

#ifdef LEFT_DOOR_PIN
mqttStateTopic = LEFT_DOOR_TOPIC;
mqttStateTopicDashed = mqttStateTopic; mqttStateTopicDashed.replace("/", "_");
mqttDiscoName = String(our_host_name) + "_" + mqttStateTopicDashed;
mqttDiscoConfigTopic = String(mqttDiscoveryPrefix) + "/sensor/" + mqttDiscoName + "/config";
mqttDiscoConfigPayload = "{\"name\": \"" + String(LEFT_DOOR_NAME) + "\", \"state_topic\": \"" + mqttStateTopic + "\", \"icon\": \"mdi:garage\", \"value_template\": \"{{ value }}\"}";

            Serial.println("MQTT discovery connectivity config: [" + mqttDiscoConfigTopic + "] : [" + mqttDiscoConfigPayload + "]");
            mqttClient.publish(mqttDiscoConfigTopic.c_str(), mqttDiscoConfigPayload.c_str(), true);
#endif
#ifdef RIGHT_DOOR_PIN
mqttStateTopic = RIGHT_DOOR_TOPIC;
mqttStateTopicDashed = mqttStateTopic; mqttStateTopicDashed.replace("/", "_");
mqttDiscoName = String(our_host_name) + "_" + mqttStateTopicDashed;
mqttDiscoConfigTopic = String(mqttDiscoveryPrefix) + "/sensor/" + mqttDiscoName + "/config";
mqttDiscoConfigPayload = "{\"name\": \"" + String(RIGHT_DOOR_NAME) + "\", \"state_topic\": \"" + mqttStateTopic + "\", \"icon\": \"mdi:garage\", \"value_template\": \"{{ value }}\"}";

            Serial.println("MQTT discovery connectivity config: [" + mqttDiscoConfigTopic + "] : [" + mqttDiscoConfigPayload + "]");
            mqttClient.publish(mqttDiscoConfigTopic.c_str(), mqttDiscoConfigPayload.c_str(), true);
#endif
#ifdef DHT_PIN
mqttStateTopic = HUMIDITY_TOPIC;
mqttStateTopicDashed = mqttStateTopic; mqttStateTopicDashed.replace("/", "_");
mqttDiscoName = String(our_host_name) + "_" + mqttStateTopicDashed;
mqttDiscoConfigTopic = String(mqttDiscoveryPrefix) + "/sensor/" + mqttDiscoName + "/config";
mqttDiscoConfigPayload = "{\"name\": \"" + String(HUMIDITY_NAME) + "\", \"state_topic\": \"" + mqttStateTopic + "\", \"device_class\": \"humidity\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value }}\"}";

            Serial.println("MQTT discovery connectivity config: [" + mqttDiscoConfigTopic + "] : [" + mqttDiscoConfigPayload + "]");
            mqttClient.publish(mqttDiscoConfigTopic.c_str(), mqttDiscoConfigPayload.c_str(), true);

            mqttStateTopic = TEMPERATURE_TOPIC;
            mqttStateTopicDashed = mqttStateTopic; mqttStateTopicDashed.replace("/", "_");
            mqttDiscoName = String(our_host_name) + "_" + mqttStateTopicDashed;
            mqttDiscoConfigTopic = String(mqttDiscoveryPrefix) + "/sensor/" + mqttDiscoName + "/config";
            mqttDiscoConfigPayload = "{\"name\": \"" + String(TEMPERATURE_NAME) + "\", \"state_topic\": \"" + mqttStateTopic + "\", \"device_class\": \"temperature\", \"unit_of_measurement\": \"F\", \"value_template\": \"{{ value }}\"}";

            Serial.println("MQTT discovery connectivity config: [" + mqttDiscoConfigTopic + "] : [" + mqttDiscoConfigPayload + "]");
            mqttClient.publish(mqttDiscoConfigTopic.c_str(), mqttDiscoConfigPayload.c_str(), true);
#endif
#ifdef ECHO_TRIGGER_PIN
mqttStateTopic = LEFT_CAR_TOPIC;
mqttStateTopicDashed = mqttStateTopic; mqttStateTopicDashed.replace("/", "_");
mqttDiscoName = String(our_host_name) + "_" + mqttStateTopicDashed;
mqttDiscoConfigTopic = String(mqttDiscoveryPrefix) + "/sensor/" + mqttDiscoName + "/config";
mqttDiscoConfigPayload = "{\"name\": \"" + String(LEFT_CAR_NAME) + "\", \"state_topic\": \"" + mqttStateTopic + "\", \"icon\": \"mdi:car\", \"value_template\": \"{{ value }}\"}";

            Serial.println("MQTT discovery connectivity config: [" + mqttDiscoConfigTopic + "] : [" + mqttDiscoConfigPayload + "]");
            mqttClient.publish(mqttDiscoConfigTopic.c_str(), mqttDiscoConfigPayload.c_str(), true);
#endif
#ifdef ECHO_TRIGGER_PIN_RIGHT
mqttStateTopic = RIGHT_CAR_TOPIC;
mqttStateTopicDashed = mqttStateTopic; mqttStateTopicDashed.replace("/", "_");
mqttDiscoName = String(our_host_name) + "_" + mqttStateTopicDashed;
mqttDiscoConfigTopic = String(mqttDiscoveryPrefix) + "/sensor/" + mqttDiscoName + "/config";
mqttDiscoConfigPayload = "{\"name\": \"" + String(RIGHT_CAR_NAME) + "\", \"state_topic\": \"" + mqttStateTopic + "\", \"icon\": \"mdi:car\", \"value_template\": \"{{ value }}\"}";

            Serial.println("MQTT discovery connectivity config: [" + mqttDiscoConfigTopic + "] : [" + mqttDiscoConfigPayload + "]");
            mqttClient.publish(mqttDiscoConfigTopic.c_str(), mqttDiscoConfigPayload.c_str(), true);
#endif
#ifdef MOTION_PIN
pinMode(MOTION_PIN, INPUT);

            mqttStateTopic = MOTION_TOPIC;
            mqttStateTopicDashed = mqttStateTopic; mqttStateTopicDashed.replace("/", "_");
            mqttDiscoName = String(our_host_name) + "_" + mqttStateTopicDashed;
            mqttDiscoConfigTopic = String(mqttDiscoveryPrefix) + "/sensor/" + mqttDiscoName + "/config";
            mqttDiscoConfigPayload = "{\"name\": \"" + String(MOTION_NAME) + "\", \"state_topic\": \"" + mqttStateTopic + "\", \"icon\": \"mdi:run-fast\", \"value_template\": \"{{ value }}\"}";

            Serial.println("MQTT discovery connectivity config: [" + mqttDiscoConfigTopic + "] : [" + mqttDiscoConfigPayload + "]");
            mqttClient.publish(mqttDiscoConfigTopic.c_str(), mqttDiscoConfigPayload.c_str(), true);
#endif
}

            //* Start mDNS with our host name:

            if (MDNS.begin(unique_client_name.c_str())) {
              Serial.println("mDNS started");
            }

            //* Set up firmware OTA update page:

            httpUpdater.setup(&httpServer, update_path, update_username, update_password);
            httpServer.begin();
            MDNS.addService("http", "tcp", 80);
            Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and your password\n", unique_client_name.c_str(), update_path, update_username);

            signal_success();
          }

          //*------------------------
          //* Main processing loop:
          //*------------------------

          unsigned long current_millis;

          void loop() {
            current_millis = millis();

            //* If WiFi is not connected, restart the system:
            if (WiFi.status() != WL_CONNECTED) {
              Serial.println("WiFi is not connected, restarting the system....");
              signal_error();
              ESP.restart();
            }
            //* If MQTT client isn't connected to the MQTT broker, reconnect:
            if (!mqttClient.connected()) {
              connect_to_mqtt_broker();
            }

#ifdef DHT_PIN
get_humidity_and_temperature();        //* (improves accuracy of distance sensor)
#endif

#ifdef ECHO_TRIGGER_PIN
if (last_occupancy_check_millis == 0) {
check_occupancy();                 //* check sonar for cars at least once
} else if (left_door_state != DOOR_STATECLOSED && left_door_state != DOOR_STATEUNKNOWN) {          //* (occupancy will probably not change if door is closed)
check_occupancy();                 //* check sonar for cars at least once
} else if (right_door_state != DOOR_STATECLOSED && right_door_state != DOOR_STATEUNKNOWN) {
check_occupancy();                 //* check sonar for cars
}
#endif

#ifdef MOTION_PIN
check_for_motion();                    //* check PIR for motion
#endif

            check_door_state();                    //* check if the door is open or closed

            mqttClient.loop();                         //* the mqtt function that processes MQTT messages

            httpServer.handleClient();             //* handles requests for the firmware update page
          }

          //*----------------------
          //* PIR Motion Detector
          //*----------------------

#ifdef MOTION_PIN
int motion_detected = 0;
unsigned long last_motion_report_millis = 0;
unsigned long first_motion_check_millis = current_millis + 60000; 		//* PIR needs to stabilize for a minute.

    void check_for_motion() {
		if (current_millis > first_motion_check_millis) {  
            int previous_motion_detected = motion_detected;
            motion_detected = digitalRead(MOTION_PIN);
            //    Serial.println(motion_detected);
            if ((last_motion_report_millis == 0) || (motion_detected != previous_motion_detected)  || (current_millis - last_motion_report_millis > 3600000)) { //* report at least once per hour to resync
              last_motion_report_millis = current_millis;
              String motion_report;
              if (motion_detected == 0) {
                motion_report = MOTION_NOT_DETECTED;
              } else {
                motion_report = MOTION_DETECTED;
              }
              mqttClient.publish(MOTION_TOPIC, motion_report.c_str());
              Serial.print("Motion: ");
              Serial.print(motion_detected);
              Serial.print(" ");
              Serial.println(motion_report);

              if (motion_detected != previous_motion_detected) {
                if (motion_report == MOTION_DETECTED) {
                  signal_beep(NOTE_G4);
                }
              }
            }
        }
	}
#endif

          //*------------------------------------------
          //* Digital Humidity and Temperature Sensor
          //*------------------------------------------

          float current_humidity = 50;    //* Stores humidity value in percent
          float current_temperature = 60; //* Stores temperature value in degrees fahrenheit
#ifdef DHT_PIN
unsigned long lastDHT_millis = 0;
unsigned long last_humidity_report_millis = 0;
unsigned long last_temperature_report_millis = 0;

          void get_humidity_and_temperature() {
            char outstr[6];
            outstr[5] = '\0';
            //* Smallest DHT-22 sensor sampling rate = 0.5 Hz (once every two seconds)
            if ((lastDHT_millis == 0) || (current_millis - lastDHT_millis > 60000)) {  //* sample every 60 seconds
              lastDHT_millis = current_millis;

              float previous_humidity = current_humidity;
              current_humidity = dht.readHumidity();  //* Get Humidity value
              if (! isnan(current_humidity)) {
                if ((last_humidity_report_millis == 0) || (current_humidity != previous_humidity) || (current_millis - last_humidity_report_millis > 3600000)) { //* report at least once per hour
                  last_humidity_report_millis = current_millis;
                  // no matching function for call to 'PubSubClient::publish(const char [16], float&)'   -->   client.publish(HUMIDITY_TOPIC, current_humidity);
                  dtostrf(current_humidity, 5, 0, outstr);
                  mqttClient.publish(HUMIDITY_TOPIC, outstr);
                  Serial.print("Humidity: ");
                  Serial.print(current_humidity);
                  Serial.println(" %");
                }
              }
              float previous_temperature = current_temperature;
              current_temperature = dht.readTemperature(true);  //* Get Temperature value (isFahrenheit = true)
              if (! isnan(current_temperature)) {
                if ((last_temperature_report_millis == 0) || ( current_temperature != previous_temperature) || (current_millis - last_temperature_report_millis > 3600000)) { //* report at least once per hour
                  last_temperature_report_millis = current_millis;
                  // no matching function for call to 'PubSubClient::publish(const char [19], float&)'   -->   mqttClient.publish(TEMPERATURE_TOPIC, previous_temperature);
                  dtostrf(current_temperature, 5, 1, outstr);
                  mqttClient.publish(TEMPERATURE_TOPIC, outstr);
                  Serial.print("Temperature: ");
                  Serial.print(current_temperature);
                  Serial.println(" F");
                }
              }
            }
          }
#endif

          //*--------------------------
          //* HC-SR04 Distance Sensor
          //*--------------------------

#ifdef ECHO_TRIGGER_PIN

          float duration1; //* Stores First HC-SR04 pulse duration value
          float distance1; //* Stores calculated distance in cm for First Sensor
          char* left_car_lane = CAR_LANE_UNKNOWN;

#ifdef ECHO_TRIGGER_PIN_RIGHT
float duration2; //* Stores Second HC-SR04 pulse duration value
float distance2; //* Stores calculated distance in cm for Second Sensor
char* right_car_lane = CAR_LANE_UNKNOWN;
#endif

          void check_occupancy() {
            if (last_occupancy_check_millis == 0 || (current_millis - last_occupancy_check_millis) > 300000) { //* check every 5 minutes
              last_occupancy_check_millis = current_millis;

              //* Calculate the Speed of Sound in cm/ms
              soundcm = (331.4 + (0.606 * current_temperature) + (0.0124 * current_humidity)) / 10000;

#ifdef ECHO_TRIGGER_PIN
//* Measure duration for first sensor
duration1 = sonar1.ping_median(iterations);

              //* Calculate the distance
              distance1 = (duration1 / 2) * soundcm;
              if (! isnan(distance1)) {

                Serial.print("Left distance: ");
                Serial.print(distance1);
                Serial.println(" cm");

                char* last_left_car_lane = left_car_lane; //* save previous state
                if ((distance1 > min_distance_float) && (distance1 < max_distance_float)) {
                  left_car_lane = CAR_LANE_OCCUPIED;
                } else {
                  left_car_lane = CAR_LANE_EMPTY;
                }

                if (last_left_car_lane != left_car_lane) {
                  mqttClient.publish(LEFT_CAR_TOPIC, left_car_lane, true);
                  Serial.print("Left car lane: ");
                  Serial.println(left_car_lane);
                }
              }
#endif

#ifdef ECHO_TRIGGER_PIN_RIGHT
//* Add a delay between sensor readings to ignore reflections
delay(1000);

              //* Measure duration for second sensor
              duration2 = sonar2.ping_median(iterations);

              //* Calculate the distance
              distance2 = (duration2 / 2) * soundcm;
              if (! isnan(distance2)) {

                Serial.print("Right distance: ");
                Serial.print(distance2);
                Serial.println(" cm");

                char* last_right_car_lane = right_car_lane; //* save previous state
                if ((distance2 > min_distance_float) && (distance2 < max_distance_float)) {
                  right_car_lane = CAR_LANE_OCCUPIED;
                } else {
                  right_car_lane = CAR_LANE_EMPTY;
                }

                if (last_right_car_lane != right_car_lane) {
                  mqttClient.publish(RIGHT_CAR_TOPIC, right_car_lane, true);
                  Serial.print("Right car lane: ");
                  Serial.println(right_car_lane);
                }
              }
#endif

            }
          }
#endif

          //*-------------------------
          //* Magnetic door switches
          //*-------------------------

          unsigned long last_door_report_millis = 0;

          void check_door_state() {
#ifdef LEFT_DOOR_PIN
//* Checks if the left door state has changed, and MQTT pub the change
String last_left_door_state = left_door_state; //* save previous state of door
if (digitalRead(LEFT_DOOR_PIN) == 0) {     //* get new state of door
left_door_state = DOOR_STATEOPEN;
} else {
left_door_state = DOOR_STATECLOSED;
}

#ifdef ECHO_TRIGGER_PIN
if (last_left_door_state != left_door_state) {
last_occupancy_check_millis = 0;    //* check the occupancy when the door closes 
check_occupancy();
}
#endif
if ((last_left_door_state != left_door_state) || ((current_millis - last_door_report_millis) > 60000)) {
  last_door_report_millis = current_millis;
  mqttClient.publish(LEFT_DOOR_TOPIC, left_door_state.c_str(), true);
  Serial.print("Left door state: ");
  Serial.println(left_door_state);
  
  if (last_left_door_state != left_door_state) {
    signal_beep(NOTE_F3);
  }
}
#endif

#ifdef RIGHT_DOOR_PIN
//* Checks if the right door state has changed, and MQTT pub the change
String last_right_door_state = right_door_state; //* save previous state of door
if (digitalRead(RIGHT_DOOR_PIN) == 0) {     //* get new state of door
right_door_state = DOOR_STATEOPEN;
} else {
right_door_state = DOOR_STATECLOSED;
}

#ifdef ECHO_TRIGGER_PIN
if (last_right_door_state != right_door_state) {
last_occupancy_check_millis = 0;    //* check the occupancy when the door closes 
check_occupancy();
}
#endif

            if ((last_right_door_state != right_door_state) || ((current_millis - last_door_report_millis) > 60000)) {
              last_door_report_millis = current_millis;
              mqttClient.publish(RIGHT_DOOR_TOPIC, right_door_state.c_str(), true);
              Serial.print("Right door state: ");
              Serial.println(right_door_state);
  
              if (last_right_door_state != right_door_state) {
                signal_beep(NOTE_F3);
              }
            }
#endif
}

            //*----------------------------------
            //* Connect to MQTT broker.
            //*----------------------------------

            void connect_to_mqtt_broker() {
              // Loop until we're reconnected
              while (!mqttClient.connected()) {
                Serial.println("Attempting MQTT connection...");
                if (mqttClient.connect(unique_client_name.c_str(), mqtt_user, mqtt_pass)) {
                  Serial.println("Connected");
                  mqttClient.subscribe(COMMAND_TOPIC);
#ifndef DHT_PIN
mqttClient.subscribe(HUMIDITY_TOPIC);
mqttClient.subscribe(TEMPERATURE_TOPIC);
#endif
} else {
Serial.print("Failed, rc=");
Serial.print(mqttClient.state());
Serial.println(", try again in 5 seconds");
//* Wait 5 seconds before retrying
delay(5000);
}
}
}

                  //*------------------------
                  //* MQTT message received
                  //*------------------------

                  void mqtt_callback(char* topic, byte* payload, unsigned int length) {

                    String strTopic;
                    String strPayload;

                    payload[length] = '\0';
                    strTopic = String((char*)topic);
                    strPayload = String((char*)payload);
//                    Serial.print("Topic: ");
//                    Serial.print(strTopic);
//                    Serial.print(" Payload: ");
//                    Serial.println(strPayload);
                    if (strTopic == COMMAND_TOPIC) {
#ifdef LEFT_RELAY_PIN
if ((strPayload == "OPEN") || (strPayload == "OPEN LEFT")) {
if (left_door_state == DOOR_STATEOPEN) {
Serial.println("Already open");
} else {
last_door_report_millis = current_millis;
mqttClient.publish(LEFT_DOOR_TOPIC, DOOR_STATEALERT, true);
sound_alert();
click_left_remote();
}
}
if ((strPayload == "CLOSE") || (strPayload == "CLOSE LEFT")) {
if (left_door_state == DOOR_STATECLOSED) {
Serial.println("Already closed");
} else {
last_door_report_millis = current_millis;
mqttClient.publish(LEFT_DOOR_TOPIC, DOOR_STATEALERT, true);
sound_alert();
click_left_remote();
}
}
#endif
#ifdef RIGHT_RELAY_PIN
if (strPayload == "OPEN RIGHT") {
if (left_door_state == DOOR_STATEOPEN) {
Serial.println("Already open");
} else {
last_door_report_millis = current_millis;
mqttClient.publish(RIGHT_DOOR_TOPIC, DOOR_STATEALERT, true);
sound_alert();
click_right_remote();
}
}
if (strPayload == "CLOSE RIGHT") {
if (left_door_state == DOOR_STATECLOSED) {
Serial.println("Already closed");
} else {
last_door_report_millis = current_millis;
mqttClient.publish(RIGHT_DOOR_TOPIC, DOOR_STATEALERT, true);
sound_alert();
click_right_remote();
}
}
#endif
if (strPayload == "ALERT") {
sound_alert();
}
if (strPayload == "ERROR") {
signal_error();
}
#ifndef DHT_PIN
} else if (strTopic == TEMPERATURE_TOPIC) {
current_temperature = strPayload.toFloat();
Serial.print("Temperature: ");
Serial.print(current_temperature);
Serial.println(" F");
} else if (strTopic == HUMIDITY_TOPIC) {
current_humidity = strPayload.toFloat();
Serial.print("Humidity: ");
Serial.print(current_humidity);
Serial.println(" %");
#endif
#ifndef LEFT_DOOR_PIN
} else if (strTopic == LEFT_DOOR_TOPIC) {
left_door_state = strPayload;
Serial.print("Left door state: ");
Serial.println(left_door_state);
#endif
#ifndef RIGHT_DOOR_PIN
} else if (strTopic == RIGHT_DOOR_TOPIC) {
right_door_state = strPayload;
Serial.print("Right door state: ");
Serial.println(right_door_state);
#endif
}
}

#ifdef LEFT_RELAY_PIN
void click_left_remote() {
Serial.println("Activating closer...");
//'click' the relay
pinMode(LEFT_RELAY_PIN, HIGH);
delay(600);
pinMode(LEFT_RELAY_PIN, LOW);
delay(1000);
}
#endif

#ifdef RIGHT_RELAY_PIN
void click_right_remote() {
Serial.println("Activating closer...");
//'click' the relay
pinMode(RIGHT_RELAY_PIN, HIGH);
delay(600);
pinMode(RIGHT_RELAY_PIN, LOW);
delay(1000);
}
#endif

                      //*-------------------------
                      //* Passive Speaker Buzzer
                      //*-------------------------

void sound_alert() {
  Serial.println("Sounding alert...");
#ifdef SPEAKER_PIN
  for ( int a = 0; a < 4; a = a + 1 ) {
    tone(SPEAKER_PIN, NOTE_A2);
    delay(300);
    noTone(SPEAKER_PIN);
    tone(SPEAKER_PIN, NOTE_A3);
    delay(700);
    noTone(SPEAKER_PIN);
    pinMode(SPEAKER_PIN, LOW);
    delay(500);
  }

  pinMode(SPEAKER_PIN, LOW);
#endif
}
void signal_success() {
#ifdef SPEAKER_PIN
  //* https://www.youtube.com/watch?v=kpsEqINeMS4
  tone(SPEAKER_PIN, NOTE_G4);
  delay(300);
  noTone(SPEAKER_PIN);

  tone(SPEAKER_PIN, NOTE_A4);
  delay(450);
  noTone(SPEAKER_PIN);

  tone(SPEAKER_PIN, NOTE_F4);
  delay(650);
  noTone(SPEAKER_PIN);

  tone(SPEAKER_PIN, NOTE_F3);
  delay(400);
  noTone(SPEAKER_PIN);
  
  tone(SPEAKER_PIN, NOTE_C4);
  delay(1150);
  noTone(SPEAKER_PIN);

  pinMode(SPEAKER_PIN, LOW);
  delay(400);
#endif
}

void signal_error() {
  Serial.println("Signaling error...");
#ifdef SPEAKER_PIN
  for (int i = 1000; i < 1244; i = i * 1.01) {
    tone(SPEAKER_PIN, i);
    delay(20);
  }
  noTone(SPEAKER_PIN);
  pinMode(SPEAKER_PIN, LOW);
  delay(200);

  for (int i = 1244; i > 1108; i = i * .99) {
    tone(SPEAKER_PIN, i);
    delay(30);
  }
  noTone(SPEAKER_PIN);
  pinMode(SPEAKER_PIN, LOW);
#endif
}

void signal_beep(unsigned int note) {
#ifdef SPEAKER_PIN
  tone(SPEAKER_PIN, note);
  delay(100);
  noTone(SPEAKER_PIN);
  pinMode(SPEAKER_PIN, LOW);
  delay(100);
#endif
}
