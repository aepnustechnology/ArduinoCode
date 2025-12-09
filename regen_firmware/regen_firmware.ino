#include <Ezo_i2c.h>                    //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>

//https://www.whiteboxes.ch/docs/tentacle/t2-mkII/#/protocols
//https://github.com/whitebox-labs/whitebox-arduino-example-code/blob/main/continuous/continuous.ino
  
#define LEVEL_SENSOR_PIN 7
#define PWM_POT_PIN A0
#define PWM_OUT_PIN 3

//#define PUMP_ADDRESS 103 //default pump
#define PUMP_ADDRESS 0x67 //found using scanner-- nondefault 

Ezo_board pump = Ezo_board(PUMP_ADDRESS, "PUMP");   

void setup() {
  Wire.begin();
  pinMode(LEVEL_SENSOR_PIN,INPUT);
  pinMode(PWM_OUT_PIN,OUTPUT);

  Serial.begin(9600);
  pump.send_cmd("X"); //stop
}


short state = 0b01; //init such that we'll pump upon reset 
bool done, timeout;
unsigned long timeStamp, pumpTime_millis;
signed long elapsed = 1;

int potValue,pwmValue;

int rolloverCount;

void loop() {
  delay(50); //millisec

  //read potentiometer1 and scale its value out to the PWM pin
  potValue = analogRead(PWM_POT_PIN);
  pwmValue = potValue > 10 ? map(potValue,0,1023,0,255) : 0; //add a small deadzone
  analogWrite(PWM_OUT_PIN,pwmValue);
  //Serial.println("pot/out:");  
  //Serial.println(potValue);  
  //Serial.println(pwmValue);

  //10x multiplier on the loop to run pumps at 500ms-ish
  rolloverCount = (++rolloverCount % 10);   
  if(rolloverCount) return;

  elapsed = millis() - timeStamp;

  //treat millis() rollover as a timeout  
  if ((elapsed < 0) || (elapsed > pumpTime_millis)) {
    timeout = true;
  } else {
    timeout = false;
  }

  //store the old state (2nd lsb) and grab the current one (lsb)
  state = (state << 1) & 0b11;
  state = state ^ digitalRead(LEVEL_SENSOR_PIN);

  Serial.println(state);
  Serial.println("elapsed:");
  Serial.println(elapsed);

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
