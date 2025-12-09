#include <Ezo_i2c.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
#include <Wire.h>    //include arduinos i2c library

//Ezo_board PH = Ezo_board(99, "PH");       //create a PH circuit object, who's address is 99 and name is "PH"
//Ezo_board EC = Ezo_board(100, "EC");      //create an EC circuit object who's address is 100 and name is "EC"

Ezo_board PHAB = Ezo_board(100, "PHAB");
Ezo_board ECAB = Ezo_board(101, "ECAB");
Ezo_board PHA2 = Ezo_board(102, "PHA2");
Ezo_board ECA2 = Ezo_board(103, "ECA2");
Ezo_board PHB2 = Ezo_board(104, "PHB2");
Ezo_board ECB2 = Ezo_board(105, "ECB2");

Ezo_board PUMPA = Ezo_board(106, "PUMPA");
Ezo_board PUMPB = Ezo_board(107, "PUMPB");
Ezo_board PUMPC = Ezo_board(108, "PUMPC");
Ezo_board PUMPD = Ezo_board(109, "PUMPD");
Ezo_board PUMPE = Ezo_board(110, "PUMPE");

String ezoCircuitReadings[6];        // an array of strings to hold the readings of each EZO circuit channel

bool reading_request_phase = true;        //selects our phase

uint32_t next_poll_time = 0;              //holds the next time we receive a response, in milliseconds
const unsigned int response_delay = 1000; //how long we wait to receive a response, in milliseconds

bool pump_toggle = true;

void setup() 
{
  Wire.begin();                           //start the I2C
  Serial.begin(115200);                   //start the serial communication to the computer
}

String receive_reading(Ezo_board &Sensor)   // function to decode the reading after the read command was issued
{
  
  //Serial.print(Sensor.get_name()); Serial.print(": "); // print the name of the circuit getting the reading

  String result;
  
  Sensor.receive_read();              //get the response data and put it into the [Sensor].reading variable if successful
                                      
  switch (Sensor.get_error()) {             //switch case based on what the response code is.
    case Ezo_board::SUCCESS:        
      //Serial.print(Sensor.get_reading());   //the command was successful, print the reading
      result = Sensor.get_reading();
      break;

    case Ezo_board::FAIL:          
     //Serial.print("Failed ");        //means the command has failed.
      result = "FAILED";
      break;  

    case Ezo_board::NOT_READY:      
      //Serial.print("Pending ");       //the command has not yet been finished calculating.
      result = "PENDING";
      break;

    case Ezo_board::NO_DATA:      
      //Serial.print("No Data ");       //the sensor has no data to send.
      result = "NODATA";
      break;
  }

  return result;
}

void loop() 
{
  if (reading_request_phase)        //if were in the phase where we ask for a reading
  {           
    //send a read command. we use this command instead of PH.send_cmd("R"); 
    //to let the library know to parse the reading
    PHAB.send_read();                      
    ECAB.send_read();
    PHA2.send_read();                      
    ECA2.send_read();
    PHB2.send_read();                      
    ECB2.send_read();
    
    next_poll_time = millis() + response_delay; //set when the response will arrive
    reading_request_phase = false;               //switch to the receiving phase
  }
  else                                          //if were in the receiving phase
  {                               
    if (millis() >= next_poll_time)             //and its time to get the response
    { 
      /* 
      receive_reading(PHAB);             //get the reading from the PH circuit
      Serial.print("  ");
      receive_reading(ECAB);             //get the reading from the EC circuit
      Serial.print("  ");
      receive_reading(PHA2);             //get the reading from the PH circuit
      Serial.print("  ");
      receive_reading(ECA2);             //get the reading from the EC circuit
      Serial.print("  ");
      receive_reading(PHB2);             //get the reading from the PH circuit
      Serial.print("  ");
      receive_reading(ECB2);             //get the reading from the EC circuit
      Serial.println();
      */

      ezoCircuitReadings[0] = receive_reading(PHAB);
      ezoCircuitReadings[1] = receive_reading(ECAB);
      ezoCircuitReadings[2] = receive_reading(PHA2);
      ezoCircuitReadings[3] = receive_reading(ECA2);
      ezoCircuitReadings[4] = receive_reading(PHB2);
      ezoCircuitReadings[5] = receive_reading(ECB2);

      for( int i = 0; i<6; i++)
      {
        Serial.print(ezoCircuitReadings[i]);
        Serial.print(", ");
      }
      Serial.println();

      

      pump_toggle = !pump_toggle;
      
      if(pump_toggle)
      {
        PUMPA.send_cmd_with_num("d,", 0.5);
        PUMPB.send_cmd_with_num("d,", 0.5);
        PUMPC.send_cmd_with_num("d,", 0.5);
        PUMPD.send_cmd_with_num("d,", 0.5);
      }
      else
      {
        PUMPA.send_cmd("x");
        PUMPB.send_cmd("x");
        PUMPC.send_cmd("x");
        PUMPD.send_cmd("x");
      }

      reading_request_phase = true;            //switch back to asking for readings
    }

    // do other stuff here
  }
}
