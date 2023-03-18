#include <Ezo_i2c.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>    //include arduinos i2c library
#include <Ezo_i2c_util.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib

int num_sensors = 14;

Ezo_board temp1 = Ezo_board(1, "temp1");
Ezo_board temp2 = Ezo_board(2, "temp2");
Ezo_board temp3 = Ezo_board(3, "temp3");
Ezo_board temp4 = Ezo_board(4, "temp4");
Ezo_board orp5 = Ezo_board(5, "orp5");
Ezo_board orp6 = Ezo_board(6, "orp6");
Ezo_board orp7 = Ezo_board(7, "orp7");
Ezo_board orp8 = Ezo_board(8, "orp8");
Ezo_board cond9 = Ezo_board(9, "cond9");
Ezo_board cond10 = Ezo_board(10, "cond10");
Ezo_board cond11 = Ezo_board(11, "cond11");
Ezo_board cond12 = Ezo_board(12, "cond12");
Ezo_board ph13 = Ezo_board(13, "ph13");
Ezo_board ph14 = Ezo_board(14, "ph14");

Ezo_board sensors[] = {temp1, temp2, temp3, temp4,
                      orp5, orp6, orp7, orp8,
                      cond9, cond10, cond11, cond12,
                      ph13, ph14};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();                         //start the I2C
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < num_sensors; i++) {
      receive_and_print_reading(sensors[i]);
      Serial.println("");
}
