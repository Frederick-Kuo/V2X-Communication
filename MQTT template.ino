//****************************************//
//* esp32v1 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* esp32v1 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <stdlib.h>
// --
#include <NewPing.h>
#define TRIGGER_PIN  33  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     25  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define ledPin 19
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
// --
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MPU6050 mpu6050(Wire);

// Replace the next variables with your SSID/Password combination
const char* ssid = "c14cry";                      //CHANGE ME
const char* password = "rtfmrtfm1";              //CHANGE ME     

// Add your MQTT Broker IP address, example:                    
const char* mqtt_server = "192.168.137.214";          //CHANGE ME

WiFiClient esp32v1;
PubSubClient client(esp32v1);
long lastMsg = 0;
char msg[50];
int value = 0;
float Distance = 0, GyroZ=0;
int angle=86,right=0,left=0,encl=0,encr=0,enc=0;
void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(ledPin,OUTPUT);
}

void setup_wifi() 
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) 
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageReceive;
  
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)message[i]);
    messageReceive += (char)message[i];
  }
  Serial.println();
  if (String(topic) == "esp32v1/angle")
  {
    Serial.println(messageReceive);
    angle=messageReceive.toInt();
  }
  if (String(topic) == "esp32v1/left")
  {
    Serial.println(messageReceive);
    left=messageReceive.toInt();
  }
  if (String(topic) == "esp32v1/right")
  {
    Serial.println(messageReceive);
    right=messageReceive.toInt();
  }
  // Add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --
   if (String(topic) == "esp32v1/output") 
   {
    Serial.print("Changing output to ");
    if(messageReceive == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageReceive == "off")
    {
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("esp32v1")) 
    {
      Serial.println("connected");
      
      // Add your subscribe topics here
      // --
      client.subscribe("esp32v1/output");
      // --
         
    } else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() 
{
  mpu6050.update();
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 50) 
  {
    lastMsg = now;
    // Add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    Distance=sonar.ping_cm();
    GyroZ=mpu6050.getGyroZ();
    enc=getcount();
    char disString[8],gyroString[8],encString[8];
    dtostrf(Distance, 1, 2, disString);
    dtostrf(GyroZ, 1, 2, gyroString);
    dtostrf(enc, 1, 2, encString);
    //Serial.print("distance: ");
    //Serial.println(disString);
    client.publish("esp32v1/Distance", disString);
    client.publish("esp32v1/GyroZ", gyroString);
    client.publish("esp32v1/Encoder Count", encString);
    //Serial.print("GyroZ: ");
    //Serial.println(gyroString);
    transmit(left,right,angle);    
  }
}
void transmit(int left,int right,int angle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  Wire.write((byte)((left & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(left & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((right & 0x0000FF00) >> 8));   
  Wire.write((byte)(right & 0x000000FF));          
  Wire.write((byte)((angle & 0x0000FF00) >> 8));    
  Wire.write((byte)(angle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
}

int getcount()
{
  Wire.requestFrom(I2C_SLAVE_ADDR,4);
  int encl=Wire.read();
  int encr=Wire.read();
  int enc=(encl+encr)/2;
  return enc;
}