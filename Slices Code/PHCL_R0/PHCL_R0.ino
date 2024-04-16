#include <Wire.h>
#include <FastLED.h>
#include <PID_v1.h>

#define I2C_ADR 12

#define ESTOP   2
#define LED_PIN 5
#define PH1     A1
#define PH2     A2
#define RELAY1  12
#define RELAY2  11

#define SAMPLE_INTERVAL_MS    20
#define SAMPLE_NUMBER         40
#define SERIAL_UPDATE_INTERVAL_MS   1000

long lastRead = 0;
long lastSerialPrint = 0;
int numSamples = 0;
uint32_t sampleTotal1 = 0, sampleTotal2 = 0;

bool E_STOP = false;

CRGB led;
int hue = 0;

struct PHCL_t
{
  double pHSetpoint_1 = 0;   // set desired temperature for heater 1 (if using relay as heater actuator)
  double rOnTime_1;           // time relay is on in ms (0-period)
  int rPeriod_1 = 2000;          // duration of relay 1 cycle in ms (1000ms = 1s)
  double Kp_1 = 0;             // PID proportional gain for heater 1
  double Ki_1 = 0;             // PID integral gain for heater 1
  double Kd_1 = 0;             // PID derivative gain for heater 1 
  double pHSetpoint_2 = 0;
  double rOnTime_2;
  int rPeriod_2 = 2000;
  double Kp_2 = 0;
  double Ki_2 = 0;
  double Kd_2 = 0;
  double pH1;  // thermocouple 1 measurement
  double pH2;  // thermocouple 2 measurement
} PHCL, PHCL_old;

union FLOATUNION_t //Define a float that can be broken up and sent via I2C
{
  float number;
  uint8_t bytes[4];
};

//Specify the links and initial tuning parameters
PID relay1PID(&(PHCL.pH1), &(PHCL.rOnTime_1), &(PHCL.pHSetpoint_1), PHCL.Kp_1, PHCL.Ki_1, PHCL.Kd_1, REVERSE);
PID relay2PID(&(PHCL.pH2), &(PHCL.rOnTime_2), &(PHCL.pHSetpoint_2), PHCL.Kp_2, PHCL.Ki_2, PHCL.Kd_2, REVERSE);

unsigned long relay1StartTime;
unsigned long relay2StartTime;

void estop()
{
  if(digitalRead(ESTOP) == HIGH)    // when set to HIGH state
  {
    led = CRGB::Red;
    FastLED.show();

    // save states
    PHCL_old.pHSetpoint_1 = PHCL.pHSetpoint_1;
    PHCL_old.pHSetpoint_2 = PHCL.pHSetpoint_2;
    PHCL_old.rOnTime_1 = PHCL.rOnTime_1;
    PHCL_old.rOnTime_2 = PHCL.rOnTime_2;

    // set critical states to zero and turn off relays
    PHCL.pHSetpoint_1 = 0;
    PHCL.pHSetpoint_2 = 0;
    PHCL.rOnTime_1 = 0;
    PHCL.rOnTime_2 = 0;

    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);

    E_STOP = true;
    Serial.println("ESTOP PRESSED!");
  }
  else    // when ESTOP state is LOW
  {
    led = CRGB::Black;
    FastLED.show();

    // reassign old states
    PHCL_old.pHSetpoint_1 = PHCL_old.pHSetpoint_1;
    PHCL_old.pHSetpoint_2 = PHCL_old.pHSetpoint_2;
    PHCL_old.rOnTime_1 = PHCL_old.rOnTime_1;
    PHCL_old.rOnTime_2 = PHCL_old.rOnTime_2;

    E_STOP = false;

    Serial.println("ESTOP RELEASED!");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(&led, 1);

  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), estop, CHANGE);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  pinMode(PH1, INPUT);
  pinMode(PH2, INPUT);

  Wire.begin(I2C_ADR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  //tell the PID to range between 0 and the full window size
  relay1PID.SetOutputLimits(0, PHCL.rPeriod_1);
  relay2PID.SetOutputLimits(0, PHCL.rPeriod_2);

  //turn the PID on
  relay1PID.SetMode(AUTOMATIC);
  relay2PID.SetMode(AUTOMATIC);

  relay1StartTime = millis();
  relay2StartTime = millis();

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  measureSensors();
  if(!E_STOP)
  {
    setPIDTunings();
    if(isnan(PHCL.pH1))
      PHCL.rOnTime_1 = 0;
    else
      relay1PID.Compute();

    if(isnan(PHCL.pH2))
      PHCL.rOnTime_2 = 0;
    else
      relay2PID.Compute();
    actuateRelays();
    printOutput();
  }
}

void setPIDTunings()
{
  relay1PID.SetTunings(PHCL.Kp_1, PHCL.Ki_1, PHCL.Kd_1);
  relay2PID.SetTunings(PHCL.Kp_2, PHCL.Ki_2, PHCL.Kd_2);
}

void printOutput()
{
  if(millis() - lastSerialPrint >= SERIAL_UPDATE_INTERVAL_MS)
  {
    Serial.print("PID1: ");
    Serial.print(PHCL.Kp_1);
    Serial.print(",");
    Serial.print(PHCL.Ki_1);
    Serial.print(",");
    Serial.print(PHCL.Kd_1);

    Serial.print("\tpH1: ");
    Serial.print(PHCL.pH1);
    Serial.print("\tSetpoint: ");
    Serial.print(PHCL.pHSetpoint_1);
    Serial.print("\tonTime: ");
    Serial.print((int)(PHCL.rOnTime_1));

    Serial.print("\tPID2: ");
    Serial.print(PHCL.Kp_2);
    Serial.print(",");
    Serial.print(PHCL.Ki_2);
    Serial.print(",");
    Serial.print(PHCL.Kd_2);

    Serial.print("\tpH2: ");
    Serial.print(PHCL.pH2);
    Serial.print("\tSetpoint: ");
    Serial.print(PHCL.pHSetpoint_2);
    Serial.print("\tonTime: ");
    Serial.println((int)(PHCL.rOnTime_2));

    lastSerialPrint = millis();
  }
}

void measureSensors()
{
  if(millis() - lastRead >= SAMPLE_INTERVAL_MS)
  {
    sampleTotal1 += analogRead(PH1);
    sampleTotal2 += analogRead(PH2);
    lastRead = millis();
    numSamples++;
  }
  if(numSamples>=SAMPLE_NUMBER){
    PHCL.pH1 = sampleTotal1 / SAMPLE_NUMBER *5.0/1024 * 14./5.;
    PHCL.pH2 = sampleTotal2 / SAMPLE_NUMBER *5.0/1024 * 14./5.;
    numSamples = 0;
    sampleTotal1 = 0;
    sampleTotal2 = 0;
  }
}

void actuateRelays()
{
  if(PHCL.pHSetpoint_1 == 0)
    PHCL.rOnTime_1 = 0;
  if(PHCL.pHSetpoint_2 == 0)
    PHCL.rOnTime_2 = 0;
  // Relay 1
  if (millis() - relay1StartTime > PHCL.rPeriod_1)
  { //time to shift the Relay Window
    relay1StartTime += PHCL.rPeriod_1;
  }
  if ((int)(PHCL.rOnTime_1) > millis() - relay1StartTime) digitalWrite(RELAY1, HIGH);
  else digitalWrite(RELAY1, LOW);

  // Relay 2
  if (millis() - relay2StartTime > PHCL.rPeriod_2)
  { //time to shift the Relay Window
    relay2StartTime += PHCL.rPeriod_2;
  }
  if ((int)(PHCL.rOnTime_2) > millis() - relay2StartTime) digitalWrite(RELAY2, HIGH);
  else digitalWrite(RELAY2, LOW);
}

void requestEvent()
{
  FLOATUNION_t t1;
  FLOATUNION_t t2;

  if(isnan(PHCL.pH1))
    t1.number = 0;
  else
    t1.number = PHCL.pH1;

  if(isnan(PHCL.pH2))
    t2.number = 0;
  else
    t2.number = PHCL.pH2;

  for (int i = 0; i < 4; i++) Wire.write(t1.bytes[i]);
  //for (int i = 0; i < 4; i++) Wire.write(t2.bytes[i]);
}

/* Command layout
  Byte 1:     1 (first relay)
              2 (second relay)
  Byte 2-5:   Setpoint
  Byte 6-9:   Kp
  Byte 10-13:  Ki
  Byte 14-17: Kd
*/

void receiveEvent(int howMany)
{
  byte in_char;
  char in_data[20];

  led = CRGB::Green;
  FastLED.show();

  //Serial.println("Receiving: ");
  int i=0;
  while(Wire.available())
  {
    in_char = Wire.read();
    in_data[i] = in_char;
    Serial.print((byte)in_data[i]);
    Serial.print(",");
    i++;
  }
  Serial.println();

  setParametersPHCL(in_data);

  led = CRGB::Black;
  FastLED.show();
}

void setParametersPHCL(char *in_data)
{
  FLOATUNION_t float1;
  FLOATUNION_t float2;
  FLOATUNION_t float3;
  FLOATUNION_t float4;

    for(int i=0; i<4; i++)  // populate variables for PID tuning
    {
      float1.bytes[i] = in_data[i+1];
      float2.bytes[i] = in_data[i+5];
      float3.bytes[i] = in_data[i+9];
      float4.bytes[i] = in_data[i+13];
    }
    if(in_data[0] == 1)
    {
      PHCL.pHSetpoint_1 = (double)float1.number;
      PHCL.Kp_1 = (double)float2.number;
      PHCL.Ki_1 = (double)float3.number;
      PHCL.Kd_1 = (double)float4.number;
    }
    if(in_data[0] == 2)
    {
      PHCL.pHSetpoint_2 = (double)float1.number;
      PHCL.Kp_2 = (double)float2.number;
      PHCL.Ki_2 = (double)float3.number;
      PHCL.Kd_2 = (double)float4.number;
    }
}