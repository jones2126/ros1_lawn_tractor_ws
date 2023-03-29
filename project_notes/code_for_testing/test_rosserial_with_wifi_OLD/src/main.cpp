#define ROSSERIAL_ARDUINO_TCP
#include <Arduino.h>
#include <ESP_WiFiManager.h>
#include <ros.h>
//#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#define LED_PIN 2
void wifi_setup();
void node_handle_setup();

// wifi related
const char* ssid     = "cui_bono";
const char* password = "Andrew13";
//uint8_t serverAddress[] = {127, 0, 0, 1};
uint8_t serverAddress[] = {192, 168, 1, 242};
IPAddress server(serverAddress);
const uint16_t serverPort = 11411;

// ros::NodeHandle nh;
  //nh.getHardware()->setConnection(server, serverPort);
  //ros::NodeHandle nh("serial://ttyUSB0:115200");
ros::NodeHandle nh("serial:/dev/ttgo_main");

void twistCallback(const geometry_msgs::Twist& msg){
    digitalWrite(LED_PIN, HIGH);   
    delay(1000);                       
    digitalWrite(LED_PIN, LOW);    
    delay(1000); 
    double linear_vel = msg.linear.x;
    double angular_vel = msg.angular.z;
    String message_output = "Received cmd_vel: linear=" + String(linear_vel,3) + " angular=" + String(angular_vel, 3);
    Serial.println(message_output);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", twistCallback);

void setup() {
  Serial.begin(115200);
  Serial.println("test_rosserial program");
  pinMode(LED_PIN, OUTPUT);
  wifi_setup();
  node_handle_setup();
}
void loop() {
  nh.spinOnce();
  digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(200);
  Serial.println("looping"); 
}
void wifi_setup(){
  // wifi setup
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  //# -> wifi delay -> a must -> other wise the ESP32 (TTGO) board will restart continuously
  while (WiFi.status() != WL_CONNECTED)  
  { delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected - IP address:  ");
  Serial.println(WiFi.localIP());
}
void node_handle_setup(){
  nh.initNode();
  nh.subscribe(sub);  
}