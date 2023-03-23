#include <Ezo_i2c.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>    //include arduinos i2c library
#include <Ezo_i2c_util.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib

// https://github.com/Atlas-Scientific/Ezo_I2c_lib/blob/master/Examples/I2c_lib_examples/I2c_read_mulitple_circuits/I2c_read_mulitple_circuits.ino
// https://github.com/MattStarfield/H2Only/blob/master/docs/data-logging-using-putty.md

#define NUM_SENSORS    14       // number of analog Pressure Sensors
#define MS            1000
#define TIME_DELAY    5 * MS    // in seconds
#define RUN_TIME      20 * MS    // in seconds 

int current_time = 0;

Ezo_board temp1 = Ezo_board(1);
Ezo_board temp2 = Ezo_board(2);
Ezo_board temp3 = Ezo_board(3);
Ezo_board temp4 = Ezo_board(4);
Ezo_board orp5 = Ezo_board(5);
Ezo_board orp6 = Ezo_board(6);
Ezo_board orp7 = Ezo_board(7);
Ezo_board orp8 = Ezo_board(8);
Ezo_board cond9 = Ezo_board(9);
Ezo_board cond10 = Ezo_board(10);
Ezo_board cond11 = Ezo_board(11);
Ezo_board cond12 = Ezo_board(12);
Ezo_board ph13 = Ezo_board(13);
Ezo_board ph14 = Ezo_board(14);

Ezo_board sensors[] = {temp1, temp2, temp3, temp4,
                      orp5, orp6, orp7, orp8,
                      cond9, cond10, cond11, cond12,
                      ph13, ph14};

String sensor_names[] = {"temp1", "temp2", "temp3", "temp4",
                      "orp5", "orp6", "orp7", "orp8",
                      "cond9", "cond10", "cond11", "cond12",
                      "ph13", "ph14"};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Serial.print("Time: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensor_names[i] + ": ");
  }  
  Serial.println();
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i].send_read_cmd();
            
  }     
}

void loop() {
  //put your main code here, to run repeatedly:
  if (current_time <= RUN_TIME) {
    Serial.print(current_time / 1000);
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(" ");
      receive_and_print_reading(sensors[i]);   
      sensors[i].send_read_cmd();
    }
    Serial.println("");
    current_time += TIME_DELAY;
    delay(TIME_DELAY);
  }
}
