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

Ezo_board pump = Ezo_board(PUMP_ADDRESS, "PUMP");   

//encoder stuff
unsigned long encoderLastTime=1,encoderThisTime;
float freq,RPM;

void handlePulse() {
  encoderThisTime = millis();
  freq = 60e-3 / (encoderLastTime - encoderThisTime); //ticks per ms --> ticks per minute
  RPM = (freq / ENC_TICKS_PER_REV); //tpm -> RPM 
  encoderLastTime = encoderThisTime;
}

//level control stuff
short state = 0b01; //init such that we'll pump upon reset 
bool done, timeout;
unsigned long timeStamp, pumpTime_millis;
signed long elapsed = 1;

int potValue,pwmValue;

int rolloverCount;

void setup() {
  Wire.begin();
  pinMode(LEVEL_SENSOR_PIN,INPUT);
  pinMode(PWM_OUT_PIN,OUTPUT);
  pump.send_cmd("X"); //stop

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(ENCODER_PIN, handlePulse, RISING);

  #ifdef SERIAL_DEBUG_PRINTS
    Serial.begin(9600);
  #endif
}


void loop() {
  delay(50); //millisec

  //5V hydrogen pump (on half-bridge)
  //read potentiometer1 and scale its value out to the PWM pin
  potValue = analogRead(PWM_POT_PIN);
  pwmValue = potValue > 75 ? map(potValue,0,1023,0,255) : 0; //add deadzone since pump wants to stall
  analogWrite(PWM_OUT_PIN,pwmValue);

  #ifdef SERIAL_DEBUG_PRINTS
    Serial.println("pot/out:");  
    Serial.println(potValue);  
    Serial.println(pwmValue);
    Serial.println("12V pump RPM:");  
    Serial.println(RPM);
  #endif

  //12V hydrogen pump:
  //we don't do anything. 0-5v analog done by pot; encoder feedback done in ISR

  //from here onward, run at 500ms-ish for liquid level controller
  rolloverCount = (++rolloverCount % 10);   
  if(rolloverCount) return;

  elapsed = millis() - timeStamp;

  //timeout: too much time OR millis() rollover 
  timeout = ((elapsed < 0) || (elapsed > pumpTime_millis));

  //grab the new state and shift back the old one
  state = (state << 1) & 0b11;
  state = state ^ digitalRead(LEVEL_SENSOR_PIN);

  #ifdef SERIAL_DEBUG_PRINTS
    Serial.println(state);
    Serial.println("elapsed:");
    Serial.println(elapsed);
  #endif

  //water level low. Start pumping and set a timeout 
  if (state == 0b10) {
    timeStamp = millis();
    pumpTime_millis = 5 * 60 * 1000; //timeout after 5 minutes 
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
