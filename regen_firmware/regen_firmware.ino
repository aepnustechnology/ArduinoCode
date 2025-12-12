#include <Ezo_i2c.h>                    //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>

//https://www.whiteboxes.ch/docs/tentacle/t2-mkII/#/protocols
//https://github.com/whitebox-labs/whitebox-arduino-example-code/blob/main/continuous/continuous.ino
  

#define PWM_POT_PIN A0
#define ENCODER_PIN 2  //interrupt capable
#define PWM_OUT_PIN 3  //pwm capable
#define LEVEL_SENSOR_PIN 7

#define ENC_TICKS_PER_REV 6

#define SERIAL_DEBUG_PRINTS //uncomment to enable serial lines

//#define PUMP_ADDRESS 103 //default pump
#define PUMP_ADDRESS 0x67 //found using scanner-- nondefault 

//level control stuff
short state = 0b01; //init such that we'll pump upon reset 
bool done, timeout;
unsigned long lowLevelTimestamp, pumpTimeout_millis = 5 * 60 * 1000; //timeout after 5 minutes 
signed long pumpTimeElapsed = 1;
Ezo_board pump = Ezo_board(PUMP_ADDRESS, "PUMP");  

//motor control stuff
int potValue,pwmValue;

//global timestamp stuff
unsigned long timeNow;
int rolloverCount;

//encoder stuff
unsigned long encoderTickCount, thisTickCount,lastTickCount;
unsigned long lastTickTime;
float ticksPerMinute,RPM;

void handlePulse() {
  encoderTickCount ++;
}

void setup() {
  Wire.begin();
  pinMode(LEVEL_SENSOR_PIN,INPUT);
  pinMode(PWM_OUT_PIN,OUTPUT);
  pump.send_cmd("X"); //stop

  pinMode(ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), handlePulse, RISING);

  #ifdef SERIAL_DEBUG_PRINTS
    Serial.begin(9600);
  #endif
}



void loop() {
  delay(50); //millisec
  timeNow = millis();

  //5V hydrogen pump (on half-bridge)
  //read potentiometer1 and scale its value out to the PWM pin. add a deadzone since lower commands stall it 
  potValue = analogRead(PWM_POT_PIN);
  pwmValue = potValue > 75 ? map(potValue,0,1023,0,255) : 0; 
  analogWrite(PWM_OUT_PIN,pwmValue);

  //12V hydrogen pump: check the encoder ticks and derive the RPM
  thisTickCount = encoderTickCount - lastTickCount;
  float ticksPerMinute = 60e3 * (thisTickCount) / (timeNow - lastTickTime); //ticks per ms -> ticks per minute
  RPM = (ticksPerMinute / ENC_TICKS_PER_REV); //tpm -> RPM 
  lastTickCount = encoderTickCount;
  lastTickTime = timeNow;

  //from here onward, run at 500ms-ish for liquid level controller
  rolloverCount = (++rolloverCount % 10);   
  if(rolloverCount) return;

  pumpTimeElapsed = timeNow - lowLevelTimestamp;

  //timeout: too much time OR millis() rollover 
  timeout = ((pumpTimeElapsed < 0) || (pumpTimeElapsed > pumpTimeout_millis));

  //grab the new state and shift back the old one
  state = (state << 1) & 0b11;
  state = state ^ digitalRead(LEVEL_SENSOR_PIN);

  #ifdef SERIAL_DEBUG_PRINTS
    Serial.println("sensorState:");
    Serial.println(state);
    Serial.println("pumpTimeElapsed:");
    Serial.println(pumpTimeElapsed);
    Serial.println();

    Serial.println("5V: pot/out:");  
    Serial.println(potValue);  
    Serial.println(pwmValue);
    Serial.println();

    Serial.println("12V: pump RPM/ticks:");  
    Serial.println(thisTickCount);    
    Serial.println(RPM);  
    Serial.println();
  #endif

  //water level low. Start pumping and set a timeout 
  if (state == 0b10) {
    lowLevelTimestamp = timeNow;
    pump.send_cmd_with_num ("D,",50);//pump (Dispense) at 50ml/min
    //Serial.println("Starting pump timer...");
    done = timeout = false;
  }

  //if the current state shows water or if we timeout, stop the pump 
  if ((state & 1) || timeout) {
    if (!done){
      pump.send_cmd("X");
      done = true;
    }
  }

}
