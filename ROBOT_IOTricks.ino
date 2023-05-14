#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <Arduino.h>
#include <Wire.h>

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
//needed for library

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>

//flag for saving data
bool shouldSaveConfig = false;

#define mqtt_server       "maqiatto.com"
#define mqtt_port         "1883"
#define mqtt_user         "your@email.com"
#define mqtt_pass         "your mqyt_password"
#define mqtt_topic        "your mqtt topic"

WiFiClient espClient;
PubSubClient client(espClient);

#define RightMotorSpeed D1
#define RightMotorPin D3
#define LeftMotorSpeed D2
#define LeftMotorPin D4

volatile int enc_r = 0;
volatile int enc_l = 0;

// Constant for steps in disk
const float stepcount = 40.00;  // 20 Slots in disk x 2, change if different
 
// Constant for wheel diameter
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different

int sensorL = 13;
int sensorR = 12;

const int motor_offset = 40;       // Diff. when driving straight
const int motor_power = 650;      // 0-1024

unsigned long num_ticks_l;
unsigned long num_ticks_r;

// Set initial motor power
int power_l = motor_power;
int power_r = motor_power;

// Used to determine which way to turn to adjust
unsigned long diff_l;
unsigned long diff_r;

// Remember previous encoder counts
unsigned long enc_l_prev = enc_l;
unsigned long enc_r_prev = enc_r;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

//******* ------------------ *******//

ICACHE_RAM_ATTR void countpulseL(){
        enc_l++;         
}

ICACHE_RAM_ATTR void countpulseR(){
        enc_r++;
}

void connect() {
  int mqtt_port_num = strtoul(mqtt_port, NULL, 10); // convert the port from string
  client.setServer(mqtt_server, mqtt_port_num);
  client.setCallback(callback);
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client", mqtt_user, mqtt_pass)) {
      String mqtt_topic_string = String(mqtt_user) + "/" + String(mqtt_topic); // concatenating the topic with the user
      const char *mqtt_topic_char = mqtt_topic_string.c_str(); // converting to char
      Serial.println("connected");
      client.subscribe(mqtt_topic_char); // subscribing to the topic
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.print(topic);
  payload[length] = '\0';
  String s = String((char *)payload);
  Serial.println(s);
  
  if (s.indexOf("forward") != -1){
    String data=s.substring(s.indexOf('forward')+1);
    int number = data.toInt();
    forward(CMtoSteps(number));
    
  }

  else if (s.indexOf("back") != -1){
    String data=s.substring(s.indexOf('back')+1);
    int number = data.toInt();
    reverse(CMtoSteps(number));
    
  }

  else if (s.indexOf("left") != -1){
    String data=s.substring(s.indexOf('left')+1);
    int number = data.toInt();
    left(ANGLEtoSteps(number));
    
  }
  
   else if (s.indexOf("right") != -1){
    String data=s.substring(s.indexOf('right')+1);
    int number = data.toInt();
    right(ANGLEtoSteps(number));
    
  }

  else if (s.indexOf("stop") != -1){
    String data=s.substring(s.indexOf('stop')+1);
    int number = data.toInt();
    stop(number);
  }

}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(sensorL, INPUT_PULLUP);
  pinMode(sensorR, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(sensorL), countpulseL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensorR), countpulseR, CHANGE);
  
  pinMode(RightMotorSpeed, OUTPUT);
  pinMode(LeftMotorSpeed, OUTPUT);
  pinMode(RightMotorPin, OUTPUT);
  pinMode(LeftMotorPin, OUTPUT);

  digitalWrite(RightMotorSpeed, 0);
  digitalWrite(LeftMotorSpeed, 0);
  digitalWrite(RightMotorPin, 1);
  digitalWrite(LeftMotorPin, 1);

  //clean FS for testing 
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(mqtt_topic, json["mqtt_topic"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt password", mqtt_pass, 20);
  WiFiManagerParameter custom_mqtt_topic("topic", "mqtt topic", mqtt_topic, 20);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  //Configuration parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_topic);
  
  //reset Wifi settings - for testing
  //wifiManager.resetSettings();

  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("Robot Configuration", "")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["mqtt_topic"] = mqtt_topic;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
}



//********** Function to convert from centimeters to steps *************
int CMtoSteps(float cm) {
 
  int result;  // Final calculation result
  float circumference = (wheeldiameter * 3.141) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result
 
}

//********** Function to convert angles to steps *************
int ANGLEtoSteps(float angle) {
 
  int result;  // Final calculation result
  float angle_step = 90 / stepcount;  // 90º in one complete rotation crear que hay que poner 180
  float f_result = angle / angle_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result
 
}

void regulate_motor(){

      // Sample number of encoder ticks
    num_ticks_l = enc_l;
    num_ticks_r = enc_r;
   /* 
       // Print out current number of ticks (DEBUGGING)
    Serial.print(num_ticks_l);
    Serial.print("--");
    Serial.print(power_l);
    Serial.print("\t");
    Serial.print(num_ticks_r);
    Serial.print("--");
    Serial.println(power_r);
   */
    
    // Number of ticks counted since last time
    diff_l = num_ticks_l - enc_l_prev;
    diff_r = num_ticks_r - enc_r_prev;

    // Store current tick counter for next time
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;

    // If left is faster, slow it down and speed up right
    if ( diff_l > diff_r ) {
      //Serial.println("left is faster");
      power_l -= motor_offset;
      power_r += motor_offset;
    }

    // If right is faster, slow it down and speed up left
    if ( diff_l < diff_r ) {
      //Serial.println("right is faster");
      power_l += motor_offset;
      power_r -= motor_offset;
    }

    // Drive motors
    
    analogWrite(RightMotorSpeed, power_r);
    analogWrite(LeftMotorSpeed, power_l);

    // Brief pause to let motors respond
    delay(20);
}

void reverse(int steps)
{
    // Reset encoder counts
  enc_l = 0;
  enc_r = 0;

  digitalWrite(RightMotorPin, HIGH); // set direction of the motors
  digitalWrite(LeftMotorPin, HIGH); // set direction of the motors

  // Go forward until step value is reached
  while ( (enc_l < steps) && (enc_r < steps) ) {
    
    regulate_motor();
 }
   // Stop when done
   digitalWrite(RightMotorSpeed, LOW);
   digitalWrite(LeftMotorSpeed, LOW);
   //enc_l = 0;  //  reset counter A to zero
   //enc_r = 0;  //  reset counter B to zero 
   //delay(10);
}

void forward(int steps)
{
    // Reset encoder counts
  enc_l = 0;
  enc_r = 0;

  digitalWrite(RightMotorPin, LOW); // set direction of the motors
  digitalWrite(LeftMotorPin, LOW); // set direction of the motors

  // Go forward until step value is reached
  while ( (enc_l < steps) && (enc_r < steps) ) {
    
    regulate_motor();
 }
   // Stop when done
   digitalWrite(RightMotorSpeed, LOW);
   digitalWrite(LeftMotorSpeed, LOW);
   //enc_l = 0;  //  reset counter A to zero
   //enc_r = 0;  //  reset counter B to zero 
   //delay(10);
}

void stop(int steps)
{
  analogWrite(RightMotorSpeed, steps);
  analogWrite(LeftMotorSpeed, steps);

}

void left(int steps)
{
    // Reset encoder counts
  enc_l = 0;
  enc_r = 0;
  digitalWrite(RightMotorPin, LOW);
  digitalWrite(LeftMotorPin, HIGH);
  analogWrite(RightMotorSpeed, power_r);
  analogWrite(LeftMotorSpeed, power_l);

// Go right until step value is reached
  while (steps > enc_l && steps > enc_r) {  
    if (steps > enc_l) {
    analogWrite(LeftMotorSpeed, power_l);
    } else {
    analogWrite(LeftMotorSpeed, 0);
    }
    if (steps > enc_r) {
    analogWrite(RightMotorSpeed, power_r);
    } else {
    analogWrite(RightMotorSpeed, 0);
    }
    yield();
   }
   // Stop when done
   analogWrite(RightMotorSpeed, 0);
   analogWrite(LeftMotorSpeed, 0);
   //counterL = 0;  //  reset counter L to zero
   //counterR = 0;  //  reset counter R to zero 
}

void right(int steps)
{
    // Reset encoder counts
  enc_l = 0;
  enc_r = 0;
  
  digitalWrite(RightMotorPin, HIGH);
  digitalWrite(LeftMotorPin, LOW);
  analogWrite(RightMotorSpeed, power_r);
  analogWrite(LeftMotorSpeed, power_l);

// Go left until step value is reached
  while (steps > enc_l && steps > enc_r) {  
    if (steps > enc_l) {
    analogWrite(LeftMotorSpeed, power_l);
    } else {
    analogWrite(LeftMotorSpeed, 0);
    }
    if (steps > enc_r) {
    analogWrite(RightMotorSpeed, power_r);
    } else {
    analogWrite(RightMotorSpeed, 0);
    }
    yield();
   }
   // Stop when done
   analogWrite(RightMotorSpeed, 0);
   analogWrite(LeftMotorSpeed, 0);
  // counterL = 0;  //  reset counter L to zero
  // counterR = 0;  //  reset counter R to zero 
}
void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    connect();
  }
  client.loop();

}
