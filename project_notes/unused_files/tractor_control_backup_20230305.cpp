/*
This is meant to run on a TTGO ESP32 LoRa OLED V1 board to control a tractor.  This device interacts with another board
to primarily receive the throttle and steering settings and and to apply those settings to the steering and throttle
controller connected to this TTGO board.

There are other features also configured in this program.  For example if this program receives an e-stop notification
this board should trigger a relay to stops the motor on the this tractor.

You will see below this program uses the RadioLib SX127x (i.e. jgromes/RadioLib@^5.3.0) library to manage the LoRa communications
ref: https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem  or https://jgromes.github.io/RadioLib/


*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <RadioLib.h>
#include <ESP32Servo.h>
#include <Adafruit_SSD1306.h>



// functions below loop() - required to tell VSCode compiler to look for them below.  Not required when using Arduino IDE
void startSerial();
void InitLoRa();
void getTractorData();
void sendOutgoingMsg();
void handleIncomingMsg();
void print_Info_messages();
double computePID(float inp);
void steerVehicle();
void throttleVehicle();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void eStopRoutine();
void transmissionServoSetup();
void startOLED();
void displayOLED();
void createCSV();
void ROSsetup();

// radio related
float FREQUENCY = 915.0;                   // MHz - EU 433.5; US 915.0
float BANDWIDTH = 125;                     // 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
uint8_t SPREADING_FACTOR = 10;             // 6 - 12; higher is slower; started at 7
uint8_t CODING_RATE = 7;                   // 5 - 8; high data rate / low range -> low data rate / high range
byte SYNC_WORD = 0x12;                     // set LoRa sync word to 0x12...NOTE: value 0x34 is reserved and should not be used
float F_OFFSET = 1250 / 1e6;               // Hz - optional if you want to offset the frequency
int8_t POWER = 15;                         // 2 - 20dBm
SX1276 radio = new Module(18, 26, 14, 33); // Module(CS, DI0, RST, ??); - Module(18, 26, 14, 33);

struct RadioControlStruct
{
  float steering_val;
  float throttle_val;
  float press_norm;
  float humidity;
  float TempF;
  byte estop;
  byte control_mode;
  unsigned long counter;
} RadioControlData;

// tractorData 4 bytes/float = 5X4=20+ 4 bytes/unsigned long = 4 so 24 bytes; 3x/second or 72 bps

uint8_t RadioControlData_message_len = sizeof(RadioControlData);
uint8_t tx_RadioControlData_buf[sizeof(RadioControlData)] = {0};

struct TractorDataStruct
{
  float speed;
  float heading;
  float voltage;
  unsigned long counter;
} TractorData;

// tractorData 4 bytes/float = 3X4=12 + 4 bytes/unsigned long = 4 so 16 bytes; 2x/second or 32 bps

uint8_t TractorData_message_len = sizeof(TractorData);
uint8_t tx_TractorData_buf[sizeof(TractorData)] = {0};

/////////////////////Loop Timing variables///////////////////////
const long readingInterval = 100;
const long transmitInterval = 500;
const long infoInterval = 2000;
const long steerInterval = 50; // 100 10 HZ, 50 20Hz, 20 = 50 Hz
const long throttleInterval = 1000;
const long csvInterval = 200;
unsigned long prev_time_reading = 0;
unsigned long prev_time_xmit = 0;
unsigned long prev_time_printinfo = 0;
unsigned long prev_time_steer = 0;
unsigned long prev_time_throttle = 0;
unsigned long prev_time_csv = 0;
/////////////////////////////////////////////////////////////////

///////////////////Steering variables///////////////////////
// pot values left: straight:1880; right:
float safety_margin_pot = 400;                   // reduce this once I complete field testing
float left_limit_pot = 3245 - safety_margin_pot; // the actual extreme limit is 3245
float left_limit_angle = -0.73;                  // most neg value for cmd_vel.ang.z from 2D Nav goal issued
float right_limit_pot = 470 + safety_margin_pot; // the actual extreme limit is 470
float right_limit_angle = 0.73;                  // // most pos value for cmd_vel.ang.z from 2D Nav goal issued
float steering_target_angle = 0;
float steering_target_pot = 0;
float steering_actual_angle = 0;
float steering_actual_pot = 0;
float steer_effort_float = 0;
int steer_effort = 0;
float tolerance = 0.007; // 1% of 0.73
const int motor_power_limit = 150;
/////////////////////////////////////////////////////////////

/////////////////// PID variables ///////////////////////
float kp = 0; // 4.5 based on 12/15/22 testing, but will leave the pot connected
float ki = 0.0;
float kd = 0; // 40 based on 12/15/testing, but will leave the pot connected
unsigned long currentTime, previousTime;
float elapsedTime;
float error;
float lastError;
float output, setPoint;
float cumError, rateError;
///////////////////////////////////////////////////////

///////////////////////Inputs/outputs///////////////////////
int transmissionPowerPin = 22;
int estopRelay_pin = 23;
int led = 2;
int transmissionSignalPin = 17;
int mode_pin = 39;         // top on the expansion board
int steering_pot_pin = 37; // third on the expansion board
int throttle_pot_pin = 36; // second on the expansion board
int steer_angle_pin = 38;  // pin for steer angle sensor - top on the expansion board
int PWMPin = 25;
int DIRPin = 12;
int ledState = LOW; // ledState used to set the LED
int test_sw = 0;    // turn the wheel all the way to the left before starting this test
unsigned long test_start_time = 0;
float test_duration = 0;
///////////////////////////////////////////////////////////

/////////////////////OLED variables///////////////////////
// OLED definitions
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
#define row_1 0
#define row_2 9
#define row_3 18
#define row_4 27
#define row_5 36
#define row_6 45
#define row_7 54
unsigned long prev_time_OLED = 0;
const long OLEDInterval = 500;
///////////////////////////////////////////////////////////
// setup servo for throttle
// Using “myservo.write(val);”  - 60=reverse; 73=neutral; 92=first
Servo transmissionServo; // create servo object to control a servo
// Servo CytronServo;  // create servo object to control a servo
// myservo.attach(transmissionSignalPin);  // attaches the servo on pin 9 to the servo object
int transmissionNeutralPos = 73;
// full_reverse and full_forward
int transmissionFullReversePos = 50;
int transmissionFullForwardPos = 110;
int transmissionServoValue = transmissionNeutralPos; // neutral position
int tranmissioPotValue = 0;                          // incoming throttle setting

/////////// ROS Variables /////////////////////

// ros setup
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter_pub("chatter", &str_msg);

float angular_vel_z = 0;
float linear_vel_x = 0;
unsigned long prev_cmd_vel_time = 0;

float linear_x, angular_z;
char buf[100];
const long chatterInterval = 2000;
unsigned long prev_time_chatter = 0;
const long cmd_velInterval = 500;
unsigned long prev_time_cmdvel = 0;

void chatter() {
  if (millis() - prev_time_chatter > chatterInterval) {
    prev_time_chatter = millis();
    sprintf(buf, "linear x: %f, angular z: %f", linear_x, angular_z);
    str_msg.data = buf;
    chatter_pub.publish(&str_msg);
    //digitalWrite(LED_PIN, HIGH);
    //delay(100);
    //digitalWrite(LED_PIN, LOW);
  }
}

void cmd_vel(const geometry_msgs::Twist& vel) {
  linear_x = vel.linear.x;
  angular_z = vel.angular.z;
  prev_time_cmdvel = millis();
}
void check_cmdvel(){
  if (millis() - prev_time_cmdvel > cmd_velInterval) {
    linear_x = 0;
    angular_z = 0; 
  }
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel);

void setup()
{
  pinMode(steer_angle_pin, INPUT);
  pinMode(mode_pin, INPUT); // used for testing - read Kp from potentiometer
  // throttle_pot_pin
  pinMode(steering_pot_pin, INPUT); // used for testing - read Kd from potentiometer
  pinMode(throttle_pot_pin, INPUT); // used for testing - read Kd from potentiometer
  pinMode(PWMPin, OUTPUT);
  pinMode(DIRPin, OUTPUT);
  pinMode(estopRelay_pin, OUTPUT);
  transmissionServoSetup();
  startSerial();
  startOLED();
  InitLoRa();
  ROSsetup();

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  // nh.advertise(rightPub);
  // nh.advertise(leftPub);
  nh.subscribe(cmd_msg);
}
void transmissionServoSetup()
{
  pinMode(transmissionPowerPin, OUTPUT);
  ESP32PWM::allocateTimer(0); // Allow allocation of all timers
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  transmissionServo.setPeriodHertz(50);                       // standard 50 hz servo
  transmissionServo.attach(transmissionSignalPin, 500, 2400); // attaches the servo on pin 18 to the servo object
}
void loop()
{
  unsigned long currentMillis = millis();
  handleIncomingMsg();
  nh.spinOnce();
  chatter();
  check_cmdvel();  
  if ((currentMillis - prev_time_steer) >= steerInterval)
  {
    steerVehicle();
  }
  if ((currentMillis - prev_time_throttle) >= throttleInterval)
  {
    throttleVehicle();
  }
  if ((currentMillis - prev_time_reading) >= readingInterval)
  {
    getTractorData();
  }
  if ((currentMillis - prev_time_xmit) >= transmitInterval)
  {
    sendOutgoingMsg();
  }
  /*
  if ((currentMillis - prev_time_printinfo) >= infoInterval)
  {
    print_Info_messages();
  }
  */
  if ((currentMillis - prev_time_OLED) >= OLEDInterval)
  {
    displayOLED();
  }
  // if ((currentMillis - prev_time_csv)        >= csvInterval)       {createCSV();}
}
/*
void startSerial()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1000); // loop forever and don't continue
  }
  delay(2000);
  Serial.println("starting: tractor_control");
}
*/
void InitLoRa()
{ // initialize SX1276 with default settings
  //Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    //Serial.println(F("success!"));
  }
  else
  {
    //Serial.print(F("failed, code "));
    //Serial.println(state);
    while (true)
      ;
  }

  if (radio.setFrequency(FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY)
  {
    //Serial.println(F("Selected frequency is invalid for this module!"));
    while (true)
      ;
  }
  //Serial.print("Selected frequency is: ");
  //Serial.println(FREQUENCY);

  if (radio.setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH)
  {
    //Serial.println(F("Selected bandwidth is invalid for this module!"));
    while (true)
      ;
  }
  //Serial.print("Selected bandwidth is: ");
  //Serial.println(BANDWIDTH);

  if (radio.setSpreadingFactor(SPREADING_FACTOR) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
  {
    //Serial.println(F("Selected spreading factor is invalid for this module!"));
    while (true)
      ;
  }
  //Serial.print("Selected spreading factor is: ");
  //Serial.println(SPREADING_FACTOR);

  if (radio.setCodingRate(CODING_RATE) == RADIOLIB_ERR_INVALID_CODING_RATE)
  {
    //Serial.println(F("Selected coding rate is invalid for this module!"));
    while (true)
      ;
  }
  //Serial.print("Selected coding rate is: ");
  //Serial.println(CODING_RATE);

  if (radio.setSyncWord(SYNC_WORD) != RADIOLIB_ERR_NONE)
  {
    //Serial.println(F("Unable to set sync word!"));
    while (true)
      ;
  }
  //Serial.print("Selected sync word is: ");
  //Serial.println(SYNC_WORD, HEX);

  if (radio.setOutputPower(POWER, true) == RADIOLIB_ERR_NONE)
  {
    //Serial.print("Selected Power set at: ");
    //Serial.println(POWER);
  }
  else
  {
    //Serial.println(F("Unable to set power level!"));
    //Serial.print(F("failed, code "));
    //Serial.println(state);
    while (true)
      ;
  }

  delay(10000);
}

void ROSsetup() {
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(chatter_pub);
  nh.loginfo("Hello, ROS!");    // enable ROS logging mechanism
}


void getTractorData()
{ // just using placeholders for now
  TractorData.speed = 255;
  TractorData.heading = 359.9;
  TractorData.voltage = 12.8;
  prev_time_reading = millis();
}
void sendOutgoingMsg()
{
  digitalWrite(led, HIGH);
  // Serial.print(F(", xmit"));
  memcpy(tx_TractorData_buf, &TractorData, TractorData_message_len);
  int state = radio.transmit(tx_TractorData_buf, TractorData_message_len);
  if (state == RADIOLIB_ERR_NONE)
  {
    // the packet was successfully transmitted
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
  {
    // the supplied packet was longer than 256 bytes
    //Serial.println(F("too long!"));
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT)
  {
    // timeout occurred while transmitting packet
    //Serial.println(F("timeout!"));
  }
  else
  {
    // some other error occurred
    //Serial.print(F("failed, code "));
    //Serial.println(state);
  }
  TractorData.counter++;
  digitalWrite(led, LOW);
}
void handleIncomingMsg()
{
  int state = radio.receive(tx_RadioControlData_buf, RadioControlData_message_len);
  // Serial.print(F("state (")); Serial.print(state); Serial.println(F(")"));
  if (state == RADIOLIB_ERR_NONE)
  { // packet was successfully received
    memcpy(&RadioControlData, tx_RadioControlData_buf, RadioControlData_message_len);
    if (RadioControlData.estop == 0)
    {
      eStopRoutine();
    }
    else
    {
      digitalWrite(estopRelay_pin, HIGH);
    }
    digitalWrite(led, HIGH);
  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT)
  { // timeout occurred while waiting for a packet
    //Serial.print(F("waiting..."));
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH)
  { // packet was received, but is malformed
    //Serial.println(F("nothing received, no timeout, but CRC error!"));
  }
  else
  { // some other error occurred
    //Serial.print(F("nothing received, no timeout, printing failed code "));
    //Serial.println(state);
  }
}
/*
void print_Info_messages()
{
  printf("\n");
  // Serial.println(F(" success!, sent the following data..."));
  // Serial.print("speed: "); Serial.print(TractorData.speed);
  // Serial.print("heading: "); Serial.print(TractorData.heading);
  // Serial.print("voltage: "); Serial.print(TractorData.voltage);
  // Serial.print("Tractr ctr: "); Serial.print(TractorData.counter);
  // Serial.print(", RC ctr: "); Serial.print(RadioControlData.counter);
  // Serial.print(", RC estop: "); Serial.print(RadioControlData.estop);
  //  print measured data rate
  // Serial.print(F(", BPS "));
  // Serial.print(radio.getDataRate());
  // Serial.print(F(" bps"));
  // Serial.println(F("packet received!"));
  //  print the RSSI (Received Signal Strength Indicator) of the last received packet
  // Serial.print(F(", RSSI: "));  Serial.print(radio.getRSSI());
  // Serial.print(F(", SNR: "));  Serial.print(radio.getSNR());
  // Serial.print(F(", dB"));
  // Serial.print(F(", Freq error: ")); Serial.print(radio.getFrequencyError());
  // Serial.print(F(", Hz"));
  Serial.print(", mode_sw: ");
  Serial.print(RadioControlData.control_mode);
  Serial.print(", steering: ");
  Serial.print(angular_vel_z);
  Serial.print(", throttle: ");
  Serial.print(linear_vel_x);
  // Serial.print(", throttle: "); Serial.print(RadioControlData.throttle_val);
  // Serial.print(", throttle-mapped: "); Serial.print(transmissionServoValue);
  // Serial.print(", press_norm: "); Serial.print(RadioControlData.press_norm);
  // Serial.print(", press_hg: "); Serial.print(RadioControlData.press_hg);
  // Serial.print(", temp: "); Serial.print(RadioControlData.temp);
  // Serial.print(", setPoint: "); Serial.print(setPoint);
  // Serial.print(", steering_actual_angle: "); Serial.print(steering_actual_angle);
  // Serial.print(", error: "); Serial.print(error);
  // Serial.print(", steer effort: "); Serial.print(steer_effort);
  Serial.print(", Kp: ");
  Serial.print(kp, 2);
  Serial.print(", Ki: ");
  Serial.print(ki, 5);
  Serial.print(", Kd: ");
  Serial.print(kd, 2);
  // Serial.print(", steer pot: "); Serial.print(analogRead(steer_angle_pin));
  // Serial.print(", Kp pot: "); Serial.print(analogRead(kp_pot_pin));
  printf("\n");
}
*/
void steerVehicle()
{
  /*
  Although I have tested the PID I still have the external potentiometers connected in order
  to continue tuning the PID.
  */
  // kp=270.0; // based on testing on 2/22/23
  // ki = 0.00001;
  ki = 0.00;
  // kd=150;   // 100 and 200 were OK; 300 slowed steering down based on 2/22/23 testing
  // kp = mapfloat(RadioControlData.throttle_val, 0, 4095, 0, 10);
  kp = mapfloat(analogRead(mode_pin), 0, 4095, 100, 600);
  // ki = mapfloat(RadioControlData.throttle_val, 0, 4095, 0, 0.0003);
  ki = mapfloat(analogRead(throttle_pot_pin), 0, 4095, 0, 0.0003);
  // kd = mapfloat(RadioControlData.throttle_val, 0, 4095, 0, 2000);
  kd = mapfloat(analogRead(steering_pot_pin), 0, 4095, 0, 2000);

  /*
      Physical  source for    pot value    mode
      position  control                    value

      top       cmd_vel          0          2
      middle    manual        4095          1
      bottom    set to 0     ~1890          0
                for safety
  */

  // setPoint is used in ComputePID to calculate error to target
  if (RadioControlData.control_mode == 2)
  {
    setPoint = angular_vel_z; // value range -.73 to +.73
  }
  else if (RadioControlData.control_mode == 1)
  {
    setPoint = RadioControlData.steering_val; // value range needs to match cmd_vel range
  }
  else if (RadioControlData.control_mode == 0)
  {
    setPoint = 0;
  }
  else
  {
    //Serial.print("Fatal error - unknown mode sw value: ");
    //Serial.print(RadioControlData.control_mode);
  }
  //Serial.print("e: ");
  //Serial.println(error);
  steering_actual_pot = analogRead(steer_angle_pin);
  steering_actual_angle = mapfloat(steering_actual_pot, left_limit_pot, right_limit_pot, left_limit_angle, right_limit_angle);
  steer_effort_float = computePID(steering_actual_angle);
  steer_effort = steer_effort_float;
  /*  Safety clamp:  The max_power_limit could be as high as 255 which
   would deliver 12+ volts to the steer motor.  I have reduced the highest setting that allows the wheels
   to be moved easily while sitting on concrete (e.g. motor_power_limit = 150 )  */
  if (steer_effort < (motor_power_limit * -1))
  {
    steer_effort = (motor_power_limit * -1);
  } // clamp the values of steer_effort
  if (steer_effort > motor_power_limit)
  {
    steer_effort = motor_power_limit;
  } // motor_power_limit

  if (error > tolerance)
  {
    //Serial.print("e-r: ");
    //Serial.print(error);
    //Serial.print("s-r: ");
    //Serial.println(steer_effort);
    digitalWrite(DIRPin, HIGH); // steer right - channel B led is lit; Red wire (+) to motor; positive voltage
    // if ((steering_actual_pot > left_limit_pot) || (steering_actual_pot < right_limit_pot)) {steer_effort = 0;}  // safety check
    analogWrite(PWMPin, steer_effort);
  }

  else if (error < (tolerance * -1))
  {
    //Serial.print("e-l: ");
    //Serial.print(error);
    //Serial.print("s-l: ");
    //Serial.println(steer_effort);
    digitalWrite(DIRPin, LOW); // steer left - channel A led is lit; black wire (-) to motor; negative voltage
    // if ((steering_actual_pot > left_limit_pot) || (steering_actual_pot < right_limit_pot)) {steer_effort = 0;}  // safety check
    analogWrite(PWMPin, abs(steer_effort));
  }
  else
  {
    steer_effort = 0;
    // analogWrite(PWMPin, steer_effort);       // Turn the motor off
  }
  prev_time_steer = millis();
} // end of steerVehicle
void throttleVehicle()
{
  /*
      Physical  source for    pot value    mode
      position  control                    value

      top       cmd_vel          0          2
      middle    manual        4095          1
      bottom    set to 0     ~1890          0
                for safety
  */

  if (RadioControlData.control_mode == 2 && linear_vel_x < 0)
  {
    transmissionServoValue = map(linear_vel_x, -1, 0, transmissionFullReversePos, transmissionNeutralPos);
  }
  else if (RadioControlData.control_mode == 2 && linear_vel_x >= 0)
  {
    transmissionServoValue = map(linear_vel_x, 0, 1.8, transmissionNeutralPos, transmissionFullForwardPos);
  }
  else if (RadioControlData.control_mode == 1)
  {
    transmissionServoValue = map(RadioControlData.throttle_val, 0, 4095, transmissionFullReversePos, transmissionFullForwardPos); // - 60=reverse; 73=neutral; 92=first
  }
  else if (RadioControlData.control_mode == 0)
  {
    transmissionServoValue = transmissionNeutralPos;
  }
  else
  {
    transmissionServoValue = transmissionNeutralPos;
  }

  // transmissionServoValue = map(RadioControlData.throttle_val, 0, 4095, transmissionFullReversePos, transmissionFullForwardPos);    // - 60=reverse; 73=neutral; 92=first
  digitalWrite(transmissionPowerPin, LOW);         // turn power on to transmission servo
  transmissionServo.write(transmissionServoValue); // sets the servo position according to the scaled value
                                                   // Serial.print("pot val-original: "); Serial.print(tranmissioPotValue);
  // Serial.print(", pot val-mapped: "); Serial.println(transmissionServoValue);
}
double computePID(float inp)
{
  // ref: https://www.teachmemicro.com/arduino-pid-control-tutorial/
  currentTime = millis();                                          // get current time
  elapsedTime = (double)(currentTime - previousTime);              // compute time elapsed from previous computation
  error = setPoint - inp;                                          // determine error
  cumError += error * elapsedTime;                                 // compute integral
  rateError = (error - lastError) / elapsedTime;                   // compute derivative
  float out = ((kp * error) + (ki * cumError) + (kd * rateError)); // PID output
  lastError = error;                                               // remember current error
  previousTime = currentTime;                                      // remember current time
  return out;                                                      // have function return the PID output
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void eStopRoutine()
{
  digitalWrite(estopRelay_pin, LOW);       // turn the LED on (HIGH is the voltage level)
  digitalWrite(transmissionPowerPin, LOW); // make sure power is on to transmission servo
  transmissionServo.write(transmissionNeutralPos);
  delay(500);
  digitalWrite(transmissionPowerPin, HIGH); // turn power on to transmission servo
}
void startOLED()
{
  //Serial.println("In startOLED");
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
  { // Address 0x3C for 128x32
    //Serial.println(F("SSD1306 allocation failed"));
    while (1)
      delay(1000); // loop forever and don't continue
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("start OLED");
  Serial.println("startOLED - exit");
}
void displayOLED()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, row_1);
  display.print("Tractor Cntrl 02-21-23");
  // display.setCursor(0,row_2);  display.print("RC Volt:");   display.setCursor(58,row_2);  display.print(voltage_val);
  display.setCursor(0, row_2);
  display.print("RSSI:");
  display.setCursor(58, row_2);
  display.print(radio.getRSSI());
  display.setCursor(0, row_3);
  display.print("Throttle:");
  display.setCursor(58, row_3);
  display.print(transmissionServoValue);
  display.setCursor(0, row_4);
  display.print("Steering:");
  display.setCursor(58, row_4);
  display.print(abs(steer_effort));
  display.setCursor(0, row_5);
  display.print("P");
  display.setCursor(10, row_5);
  display.print(kp, 2);
  display.setCursor(50, row_5);
  display.print("I");
  display.setCursor(60, row_5);
  display.print(ki, 5);
  display.setCursor(0, row_6);
  display.print("D");
  display.setCursor(10, row_6);
  display.print(kd, 2);
  display.setCursor(0, row_7);
  display.print("Mode SW:");
  display.setCursor(58, row_7);
  display.print(RadioControlData.control_mode);
  // display.setCursor(0,57);     display.print("T cntr:");    display.setCursor(58,57);     display.print(TractorData.counter);
  display.display();
  //  Serial.print(", TractorData.counter: "); Serial.print(TractorData.counter);
}
/*
void createCSV()
{
  
  currentTime, steering_actual_angle, setPoint, kp, ki, kd
  
  Serial.print(", Kp: ");
  Serial.print(kp, 2);
  Serial.print(", Ki: ");
  Serial.print(ki, 5);
  Serial.print(", Kd: ");
  Serial.print(kd, 2);
}
*/
