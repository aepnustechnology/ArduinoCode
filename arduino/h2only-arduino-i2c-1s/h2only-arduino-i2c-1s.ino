/* H2Only Demo Testbed Arduino Firmware
   by Matt Garfield (matt.garfield@nextfab.com)
   10/17/2019
*/

////////////////////////////////////////////////////////////////////////////////
// LIBRARIES
////////////////////////////////////////////////////////////////////////////////

  #include <Ezo_i2c.h> //include the EZO I2C library from https://github.com/Atlas-Scientific/Ezo_I2c_lib
  #include <Wire.h>    //include arduinos i2c library

////////////////////////////////////////////////////////////////////////////////
// GLOBAL DEFINES
////////////////////////////////////////////////////////////////////////////////

  #define VERSION                 "v1.1.1-1s"
  
//----------------
// Pin Definitions
// ---------------
  #define I2C_SDA_PIN             20  // Arduino Mega 2560
  #define I2C_SCL_PIN             21  // Arduino Mega 2560

  // Pressure Sensors
  #define PSA1_PIN                A0
  #define PSB1_PIN                A1
  #define PSC1_PIN                A2
  #define PSC2_PIN                A3
  #define PSD1_PIN                A4
  #define PSD2_PIN                A5

  // Current Sensors
  #define CRA1_PIN                A8
  #define CRB1_PIN                A9
  #define CRC1_PIN                A10
  #define CRD1_PIN                A11
  #define CRD2_PIN                A12
  #define CRE1_PIN                A13
  #define tempPin                 12

  #define PC_BAUD                 115200    // set baud rate for host serial monitor(pc/mac/other)
  //#define PC_BAUD                 9600    // set baud rate for host serial monitor(pc/mac/other)
  #define EOL                     \r\n      // End-of-Line Character for LabVIEW https://knowledge.ni.com/KnowledgeArticleDetails?id=kA00Z0000019KZDSA2&l=en-US
  
  #define EZO_READING_INTERVAL_MS 900      // timing interval to collect EZO sensor readings in ms
  //#define SEND_TO_PC_INTERVAL_MS  90       // interval the readings are sent to the computer (NOTE: this is not the frequency of taking the readings!)
  #define SEND_TO_PC_INTERVAL_MS  990       // 990 ms (1 sec) test - interval the readings are sent to the computer (NOTE: this is not the frequency of taking the readings!)
  
  //----------------
  // Define EZO I2C Devices
  // ---------------
  #define NUM_EZO_CIRCUITS        6         // number of EZO circuits attached to the Tentacle
  //Ezo_board(uint8_t i2c_address, const char* name); 
  Ezo_board PHAB = Ezo_board(100, "PHAB");
  Ezo_board ECAB = Ezo_board(101, "ECAB");
  Ezo_board PHA2 = Ezo_board(102, "PHA2");
  Ezo_board ECA2 = Ezo_board(103, "ECA2");
  Ezo_board PHB2 = Ezo_board(104, "PHB2");
  Ezo_board ECB2 = Ezo_board(105, "ECB2");
  
  #define NUM_EZO_PUMPS           6         // number of EZO circuits attached to the Tentacle
  //Ezo_board(uint8_t i2c_address, const char* name); 
  Ezo_board PUMPA = Ezo_board(106, "PUMPA");
  Ezo_board PUMPB = Ezo_board(107, "PUMPB");
  Ezo_board PUMPC = Ezo_board(108, "PUMPC");
  Ezo_board PUMPD = Ezo_board(109, "PUMPD");
  Ezo_board PUMPE = Ezo_board(110, "PUMPE");
  Ezo_board temp = Ezo_board(12, "temp")

  
  //----------------
  // Define Analog Device Reading Arrays
  // ---------------
  #define A10_MAX_KPA             69      // max valid reading threshold in kPa for A-10 https://www.atlas-scientific.com/product_pages/pressure/a-10_pressure.html
  #define A100_MAX_KPA            690     // max valid reading threshold in kPa for A-100 https://www.atlas-scientific.com/product_pages/pressure/a-100_pressure.html
  
  #define NUM_PRESSURE_SENSORS    6       // number of analog Pressure Sensors
  const int pressureSensorPinArray[] = { 
                                    PSA1_PIN,
                                    PSB1_PIN,
                                    PSC1_PIN,
                                    PSC2_PIN,
                                    PSD1_PIN,
                                    PSD2_PIN
                                 };
  

  #define NUM_CURRENT_MONITORS    6       // number of analog Current Monitors
  const int currentMonitorPinArray[] = { 
                                    CRA1_PIN,
                                    CRB1_PIN,
                                    CRC1_PIN,
                                    CRD1_PIN,
                                    CRD2_PIN,
                                    CRE1_PIN
                                 };

////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
////////////////////////////////////////////////////////////////////////////////

//Not used - UART
//#define EZO_RESPONSE_WAIT_MS        100           // delay to wait for serial response after sending command to EZO Circuit
//#define EZO_READING_DELAY_MS        100         // delay between each reading attempt

String ezoCircuitReadings[NUM_EZO_CIRCUITS];        // an array of strings to hold the readings of each EZO circuit channel
String ezoPumpFlowRates[NUM_EZO_PUMPS];          // an array of strings to hold the flow rate setting (mL/min) of each EZO pump

//String pressureReadings[NUM_PRESSURE_SENSORS];      // an array of strings to hold the readings of each pressure sensor

uint32_t timestampStart = 0;        // millis() count of the start of a data collection period

//Not used - UART
//char sensordata[30];                // A 30 byte character array to hold incoming data from the EZO sensors
//byte sensor_bytes_received = 0;     // We need to know how many characters bytes have been received

//byte computer_bytes_received = 0;        //We need to know how many characters bytes have been received
byte pcBytesReceived = 0;           //We need to know how many characters bytes have been received

//char computerdata[32];                    //we make a character array to hold incoming data from a pc/mac/other.
char pcByteArray[32];               //we make a character array to hold incoming data from a pc/mac/other.

//int computer_in_byte;
int pcInByte;                       //incoming byte from PC over serial connection

//boolean computer_msg_complete = false;
bool pcMsgCompleteFlag = false;

//char *cmd;                               //Char pointer used in string parsing
char *cmdFromPc;                              //Char pointer used in string parsing

//Not used - UART
//int channel = 0;                         // INT pointer to hold the current position in the channel_ids/channel_names array
//int retries;                             // com-check functions store number of retries here
//boolean answerReceived;                  // com-functions store here if a connection-attempt was successful
//byte error;                              // error-byte to store result of Wire.transmissionEnd()

//Not used - replaced by I2C library
//byte code = 0;                      // used to hold the I2C response code.
//byte in_char = 0;                   // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

//Not used - UART
// list of baudrates to try when connecting to a stamp (they're ordered by probability to speed up things a bit)
//const long validBaudrates[] = {38400, 19200, 9600, 115200, 57600};
// store for the determined baudrates for every stamp
//long channelBaudrate[] = {0, 0, 0, 0, 0, 0, 0, 0};

//Not used - UART
//String stamp_type;                       // hold the name / type of the stamp
//char stamp_version[4];                   // hold the version of the stamp

//unsigned long next_serial_time;
uint32_t nextSendToPcTime = 0;

//unsigned long next_reading_time;              // holds the time when the next reading should be ready from the circuit
uint32_t nextEzoReadingTime = 0;

//boolean request_pending = false;              // wether or not we're waiting for a reading
//bool reading_request_phase = true;        //selects our phase
bool ezoReadingRequestFlag = true;

//boolean sendReadings = false;                 // flag to control when to start sending sensor readings to PC
bool sendReadingsToPcFlag = false;                // flag to control when to start sending sensor readings to PC
bool ezoPumpCommandFlag = false;              // flag to signal new pump commands need to be handled

//SoftwareSerial sSerial(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);     // RX, TX  - Name the software serial library sSerial (this cannot be omitted)

void setup()
{
  // set the input pins for the analog pressure sensors
  pinMode(PSA1_PIN,  INPUT_PULLUP);
  pinMode(PSB1_PIN,  INPUT_PULLUP);
  pinMode(PSC1_PIN,  INPUT_PULLUP);
  pinMode(PSC2_PIN,  INPUT_PULLUP);
  pinMode(PSD1_PIN,  INPUT_PULLUP);
  pinMode(PSD2_PIN,  INPUT_PULLUP);

  // set the input pins for the analog current monitors
  pinMode(CRA1_PIN,  INPUT);
  pinMode(CRB1_PIN,  INPUT);
  pinMode(CRC1_PIN,  INPUT);
  pinMode(CRD1_PIN,  INPUT);
  pinMode(CRD2_PIN,  INPUT);
  pinMode(CRE1_PIN,  INPUT);
  pinMode(temp, INPUT)

  Serial.begin(PC_BAUD);                // Set the serial port for communication to PC
  Wire.begin();                         //start the I2C
  
  //sSerial.begin(BAUD_EZO_CIRCUITS);              // Set the soft serial port for communication to sensor circuits

  //next_serial_time = millis() + SEND_READINGS_INTERVAL_MS;  // calculate the next point in time we should do serial communications
  nextSendToPcTime = millis() + SEND_TO_PC_INTERVAL_MS;   // calculate the next point in time we should do serial communications
  
  //next_reading_time = millis() + EZO_READING_DELAY_MS;
  nextEzoReadingTime = millis() + EZO_READING_INTERVAL_MS;

  //Initialize EZO Circuit Readings String array
  for(int i = 0; i < NUM_EZO_CIRCUITS; i++)
  {
    ezoCircuitReadings[i] = "NULL";
  }

  //Initialize EZO Pump Flow Rates String array
  for(int i = 0; i < NUM_EZO_PUMPS; i++)
  {
    ezoPumpFlowRates[i] = "0";
  }

  do_ezo_pump_command_update(); // set all pumps to 0 at start up
  
  Serial.print(F("-- H2Only Demo Testbed "));
  Serial.print(F(VERSION));                    // print version number of firmware from #define
  Serial.print(F(" --\r\n"));
  Serial.print(F("\r\n"));
  Serial.print(F("Command Options:\r\n"));  
  Serial.print(F("\t* 's' to toggle START sensor readings\r\n"));
  Serial.print(F("\t* 'p,#,#,#,#,#' to set PUMP flow rates for Pumps A - E respectively (where '#' is +/- 0 to 50 mL/min)\r\n"));
  Serial.print(F("\t* 'x' to STOP all pumps\r\n"));
  Serial.print(F("\t* 'q' to QUIT (stop) all pumps & sensor readings\r\n"));
  
  //Serial.println( F("\t* 'initiaize'\t bring all EZO circuits to known initial state"));
  //Serial.println( F("\t* 'calibrate'\t enter calibration mode"));
  Serial.print(F("------------------\r\n"));
  Serial.print(F("\r\n"));
  //Serial.println("Enter 's' to Toggle Start/Stop Sensor Readings...\n\r");

} // END setup()


void loop()
{

  // Loop - Check for user input to jump to a different mode
  ////////////////////////////////////////////////////////////////////////////////

  // Check for serial data from the computer

      while (Serial.available() > 0)
      {
        pcInByte = Serial.read();           // bytes are incoming from the PC serial connection (into the serial buffer)
    
        if (pcInByte == '\n' || pcInByte == '\r') // if a newline character arrives, we assume a complete command has been received
        {
          pcByteArray[pcBytesReceived] = 0;  // append null terminator (ASCII 0) to the byte string from PC
          pcMsgCompleteFlag = true;          // set msg complete flag
          pcBytesReceived = 0;               // reset byte array counter
        }
        else  // the command from the pc is not complete yet, so just add the byte to data array
        {
          pcByteArray[pcBytesReceived] = pcInByte;  // add incoming byte to byte array
          pcBytesReceived++;                        // increment bye counter (array position)
        }
      } //END serial check while-loop

  // Check if incoming command from PC matches valid command

      if (pcMsgCompleteFlag) // if there is a complete command from the computer
      {
        cmdFromPc = pcByteArray; // Set cmd pointer to stored byte array
    
        // Parse 's' command to start taking sesnor readings
        // ---------------------------------
        if (String(cmdFromPc) == F("s")) 
        {
          //sendReadingsToPcFlag = !sendReadingsToPcFlag;   //toggle transsmission flag
          sendReadingsToPcFlag = true;   //turn on transsmission flag
          
          printTableHeading();
          Serial.println("[START]");
          timestampStart = millis();
          
          //if(sendReadingsToPcFlag)
          //{
          //  printTableHeading();
          //  Serial.println("[START]");
          //  timestampStart = millis();
          //}
          //else
          //{
          //  Serial.println("[STOP]");
          //  Serial.println();
          //}
          
          pcMsgCompleteFlag = false;    //reset flag for next incoming msg
          //return;
        }


        // Parse 'p' command to update EZO Pump Flow Rates
        // ---------------------------------
        else if (cmdFromPc[0] == 'p') // if first char of command is 'p' (the full string has pump speed data)
        {

          // Pump Command format: "p,intVal1,intVal2,intVal3,intVal4,intVal5"
          // comma-seperated integer values from 1 - 105 mL/min with no white-space

          // split incoming command string into sub-string "tokens" between delimiters using strtok()
          // https://www.geeksforgeeks.org/strtok-strtok_r-functions-c-examples/
          char* token = strtok(cmdFromPc, ","); // splits string at delimiter and returns rest of string as "token"
          int pumpIndex = 0;

          while(token != NULL)
          {
            token = strtok(NULL, ",");
            ezoPumpFlowRates[pumpIndex] = token;
            pumpIndex++;
          }
          
          //for(int i=0; i<pumpIndex; i++)
          //{
          //  Serial.println(ezoPumpFlowRates[i]);
          //}

          ezoPumpCommandFlag = true;    //set flag to handle new pump commands
          pcMsgCompleteFlag = false;    //reset flag for next incoming msg
          //return;
        }

        // Parse 'x' command to stop all pumps
        // ---------------------------------
        else if (String(cmdFromPc) == F("x")) 
        {
          for(int i=0; i<NUM_EZO_PUMPS; i++)  // set all pump vars to 0
          {
            ezoPumpFlowRates[i] = "0";    // flow rate variables are Strings
          }
          
          ezoPumpCommandFlag = true;    //set flag to handle new pump commands
          pcMsgCompleteFlag = false;    //reset flag for next incoming msg
          //return;
        }

        
        // Parse 'q' command to stop (quit) taking readings
        // ---------------------------------
        else if (String(cmdFromPc) == F("q")) 
        {
          // STOP Sensor Readings
          //sendReadingsToPcFlag = !sendReadingsToPcFlag;   //toggle transsmission flag
          sendReadingsToPcFlag = false;   //turn off transsmission flag

          Serial.println("[STOP]");
          Serial.println();

          // STOP all Pumps
          for(int i=0; i<NUM_EZO_PUMPS; i++)  // set all pump vars to 0
          {
            ezoPumpFlowRates[i] = "0";    // flow rate variables are Strings
          }
          
          ezoPumpCommandFlag = true;    //set flag to handle new pump commands
            
          //if(sendReadingsToPcFlag)
          //{
          //  printTableHeading();
          //  Serial.println("[START]");
          //  timestampStart = millis();
          //}
          //else
          //{
          //  Serial.println("[STOP]");
          //  Serial.println();
          //}
          
          pcMsgCompleteFlag = false;    //reset flag for next incoming msg
          //return;
        }

        /* Not configured
        // configure EZO circuits to known initial state
        // ---------------------------------
        else if (String(cmdFromPc) == F("initialize")) 
        {
          initialize();
          pcMsgCompleteFlag = false;    //reset flag for next incoming msg
          //return;
        }

        // calibrate sensors
        // ---------------------------------
        else if (String(cmdFromPc) == F("calibrate")) 
        {
          calibrate();
          pcMsgCompleteFlag = false;    //reset flag for next incoming msg
          //return;
        }
        */

        //default case if command is not found
        // ---------------------------------
        else  
        {
          Serial.print("Unknown command: ");
          Serial.print(cmdFromPc);            //print unknown command to serial terminal for troubleshooting
          Serial.print("\r\n");               // use hard-coded "end of line" command in LabVIEW format for ease of translation
          //Serial.print("EOL");                //print LabVIEW formatted end-of-line escape sequence
          pcMsgCompleteFlag = false;    //reset flag for next incoming msg
          //return;
        }
    
      } // end command-match if-statement

  
  // Loop - Update Pumps with Flow Rate Commands
  ////////////////////////////////////////////////////////////////////////////////
  
    if(ezoPumpCommandFlag)
    {
      do_ezo_pump_command_update();
      ezoPumpCommandFlag = false;     // reset flag
    }
  
  // Loop - Read Sensors and Transmit to PC over serial connection
  ////////////////////////////////////////////////////////////////////////////////
  
    if(sendReadingsToPcFlag)  // wait for the flag to start sending readings
    {
      do_ezo_sensor_readings();
      //do_pressure_sensor_readings();
      do_serial();
      
    }

} // END loop()


void printTableHeading()
{
  Serial.print("\r\nTIME  \tPHAB \tECAB\tPHA2 \tECA2\tPHB2 \tECB2\tPSA1  \tPSB1  \tPSC1  \tPSC2  \tPSD1  \tPSD2  \tCRA1  \tCRB1  \tCRC1  \tCRD1  \tCRD2  \tCRE1  \tFLA1\tFLB1\tFLC1\tFLD1\tFLE1\r\n"); // Data Table Heading
  Serial.print("------\t-----\t----\t-----\t----\t-----\t----\t------\t------\t------\t------\t------\t------\t------\t------\t------\t------\t------\t------\t----\t----\t----\t----\t----\r\n"); // Data Table Separator
}

// do serial communication in a "asynchronous" way
void do_serial()
{
  // Triggering serial transission routine after nextSendToPcTime has expired results in 
  // data row timestamps that reflect whenever the next time the program checks for this AFTER the expiration.
  // To provide data rows in a more precise interval, do the following:
  
  // 1.) trigger expiration prior to nextSendToPcTime (say 20ms) to allow time to prepare data row
  // 2.) concatenate strings to built data row (except for timestamp)
  // 3.) wait for nextSendToPcTime to expire, then capture timestamp and prepend to string and transmit over serial

  int processingDelay1 = 0; //15;       // time padding prior to sending data to PC for building data row
  
  if (millis() >= (nextSendToPcTime - processingDelay1))                 // has (SEND_TO_PC_INTERVAL_MS - processingDelay) passed since last send?
  {
    uint32_t triggerTime = millis();  // capture the timestamp when the serial transmission routine was triggered to calculate nextSendToPcTime at end
    
    int processingDelay2 = 0; //5;         // time padding prior to sending data to PC for converting timstamp to String
    String dataRow;
    String transmitString;

    // Prepend Timestamp last - see below

    // Append EZO Sensor Readings to dataRow
    ////////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i<NUM_EZO_CIRCUITS; i++)    // loop through all the sensors
    {         
      //Serial.print(ezoCircuitReadings[i]);   // print the actual reading
      //Serial.print("\t");                    // Tab separate Values

      dataRow += ezoCircuitReadings[i];       // append the sensor reading
      dataRow += "\t";                        // append tab (tsv)
    }

    // Print Pressure Sensor Readings
    ////////////////////////////////////////////////////////////////////////////////
    for(int i=0; i<NUM_PRESSURE_SENSORS;i++)                    //cycle through all pressure sensors
    {
      // Collect Pressure Sensor Readings
      int adc = analogRead(pressureSensorPinArray[i]);  //get analog voltage value from sensor pin
      float volts = adc*0.0049;                              //convert ADC points to millivolts (5V /1024 = 0.0049)
      
      //float pressure = (2.5*volts) - 1.25;                    //A-10 convert volts to pressure in PSI
      //float pressure = (2.5*volts) - 12.5;                    //A-100 convert volts to pressure in PSI
      float pressure = (17.237*volts) - 8.6185;              //A-10 convert volts to pressure in kPa
      //float pressure = (17.237*volts) - 86.185;              //A-100 convert volts to pressure in kPa
  
      if(pressure<0){pressure=0;}                       //remove negative value if sensors is not under pressure
      if(pressure>A10_MAX_KPA){pressure=0;}             //remove over-max values due to INPUT_PULLUP with no sensor connected
  
      // Convert Float to formatted String
      int width = 6;                          //number of chars to represent float (including decimal and negative sign)
      char buff[width];
      
      //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
      dtostrf(pressure, width, 2, buff);  //double (or float) to character string
  
      //Serial.print(buff);                     // Print Pressure Sensor Value
      //Serial.print("\t");                     // Tab-Seperate Values

      dataRow += buff;                        // append the sensor reading
      dataRow += "\t";                        // append tab (tsv)
      
    } // end Pressure Sensor for-loop

    // Print Current Monitor Readings
    ////////////////////////////////////////////////////////////////////////////////
    for(int i=0; i<NUM_CURRENT_MONITORS;i++)                    //cycle through all pressure sensors
    {
      // Collect Curent Monitor Readings
      int adc = analogRead(currentMonitorPinArray[i]);      //get analog voltage value from sensor pin
      float volts = adc*0.0049;                              //convert ADC points to millivolts (5V /1024 = 0.0049)
      
      float current = volts;                    //Current Monitor is designed to read out 1 Volt per Amp 
  
      if(current<0){current=0;}             //remove negative values due to noise if no current is flowing
  
      // Convert Float to formatted String
      int width = 6;                          //number of chars to represent float (including decimal and negative sign)
      char buff[width];
      
      //dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
      dtostrf(current, width, 3, buff);  //double (or float) to character string
  
      //Serial.print(buff);                     // Print Pressure Sensor Value
      //Serial.print("\t");                     // Tab-Seperate Values

      dataRow += buff;                        // append the sensor reading
      dataRow += "\t";                        // append tab (tsv)
      
    } // end Current Monitor for-loop

    // Print Pump Flow Rates
    ////////////////////////////////////////////////////////////////////////////////

    for (int i=0; i<NUM_EZO_PUMPS; i++)    // loop through all the pumps
    {         
      //Serial.print(ezoPumpFlowRates[i]);   // print the actual reading
      //Serial.print("\t");                    // Tab separate Values

      dataRow += ezoPumpFlowRates[i];         // append the flow rate
      dataRow += "\t";                        // append tab (tsv)         
    }

    // END Data Row
    ////////////////////////////////////////////////////////////////////////////////
    
    //Serial.print("\r\n");                       // End-of-Line Character for LabVIEW
    //Serial.print(EOL);                       // End-of-Line Character for LabVIEW

    dataRow.remove(dataRow.length()-1);         // remove last tab ('\t') from dataRow
    dataRow += "\r\n";                          // End-of-Line Character for LabVIEW
    
    // Wait for exact time to capture timestamp and send transmission
    // include processing time for these operations
    while (millis() < (nextSendToPcTime-processingDelay2)){};    
    
    // Prepend Timestamp to beginning of dataRow
    ////////////////////////////////////////////////////////////////////////////////

    transmitString = (millis()-timestampStart)*.001;  // capture timestamp (in seconds) and set to string
    transmitString += "\t";                           // append tab
    transmitString += dataRow;                        // append complete dataRow from above
    
    Serial.print(transmitString);               // print complete string to PC

    // Wait for exact time to recalculate nextSendToPcTime
    while (millis() < (nextSendToPcTime-2)){};
    
    // Calculate next time to send data to PC
    nextSendToPcTime = millis() + SEND_TO_PC_INTERVAL_MS;
    //nextSendToPcTime = triggerTime + SEND_TO_PC_INTERVAL_MS;  // use triggerTime instead of millis() to prevent timestamp drift

  } // END prepare data for serial transmission
  
} // END do_serial()

bool do_ezo_pump_command_update()
{
  
  for (int i=0; i<NUM_EZO_PUMPS; i++)    // loop through all the pumps
  {         
    char cmdPump[12];   // char array for pump command (EZO lib functions can't use String Objects)
    
    for(int j=0; j<sizeof(cmdPump);j++){ cmdPump[j] = '\0';}  // initialize char array to NULL ternimator

    if(ezoPumpFlowRates[i].toInt() == 0)                      // command "DC,0,*" has to be translated to a "Stop Dispensing" command "x"
    {
      cmdPump[0] = 'x';   // Stop Dispensing command
    }
    else if(ezoPumpFlowRates[i].toInt() == 105)              // string "105" for max fwd speed
    {
      strcpy(cmdPump, "D,*");
    }
    else if(ezoPumpFlowRates[i].toInt() == -105)             // string "-105" for max reverse speed
    {
      strcpy(cmdPump, "D,-*");
    }
    else                                                      // command is value other than 0
    {        
      String cmdString = ("DC," + ezoPumpFlowRates[i] + ",*");    // Populate with Constant Flow Rate Command: "DC,flowRateInt,*"
      cmdString.toCharArray(cmdPump, sizeof(cmdPump));            // Convert String object to char array
    }
    
    //Serial.println(ezoPumpFlowRates[i].toInt());
    //Serial.println(cmdPump);

    switch(i)                     // send command to specific Pump Object
    {
      case 0: 
        PUMPA.send_cmd(cmdPump);
        break;
        
      case 1: 
        PUMPB.send_cmd(cmdPump);
        break;

      case 2: 
        PUMPC.send_cmd(cmdPump);
        break;

      case 3: 
        PUMPD.send_cmd(cmdPump);
        break;

      case 4: 
        PUMPE.send_cmd(cmdPump);
        break;

      default: 
        break;
        
    } // END switch-case
   
  } // END for-loop
   
  return true;
  
} // END do_ezo_pump_command_update()

String ezoReceiveReading(Ezo_board &Sensor)   // function to decode the reading after the read command was issued
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
      result = "FAILED";              //means the command has failed.
      break;  

    case Ezo_board::NOT_READY:      
      //Serial.print("Pending ");       //the command has not yet been finished calculating.
      result = "PENDING";             //the command has not yet been finished calculating.
      break;

    case Ezo_board::NO_DATA:      
      //Serial.print("No Data ");       //the sensor has no data to send.
      result = "NODATA";                //the sensor has no data to send.
      break;
  }

  return result;
}

// take sensor readings in a "asynchronous" way
void do_ezo_sensor_readings()
{

  if (ezoReadingRequestFlag)        //request a new reading from each EZO circuit
  {           
    //send a read command. we use this command instead of PH.send_cmd("R"); 
    //to let the library know to parse the reading
    PHAB.send_read();                      
    ECAB.send_read();
    PHA2.send_read();                      
    ECA2.send_read();
    PHB2.send_read();                      
    ECB2.send_read();
    
    nextEzoReadingTime = millis() + EZO_READING_INTERVAL_MS; //set when the response will arrive
    ezoReadingRequestFlag = false;               //switch to the receiving phase - get new reading after pollling interval expires
  }
  else                                          //if were in the receiving phase
  {
                          
    if (millis() >= nextEzoReadingTime)             //and its time to get the response
    {

      // Append EZO Sensor Readings to Global String Array
      ezoCircuitReadings[0] = ezoReceiveReading(PHAB);
      ezoCircuitReadings[1] = ezoReceiveReading(ECAB);
      ezoCircuitReadings[2] = ezoReceiveReading(PHA2);
      ezoCircuitReadings[3] = ezoReceiveReading(ECA2);
      ezoCircuitReadings[4] = ezoReceiveReading(PHB2);
      ezoCircuitReadings[5] = ezoReceiveReading(ECB2);

      ezoReadingRequestFlag = true;            //switch back to asking for readings
    }

  } // END receive EZO sensor Readings


} // END do_ezo_sensor_readings()

/*
// Send the sequence of serial commands used in "tentacle_setup.ino" to set EZO circuits to initial state
boolean initialize()
{
  Serial.println("initialize");
  Serial.println("NOT CURRENTLY ENABLED. Used for UART EZO Circuit Setup.");


} // END initialize()
*/

/*
// Send the sequence of serial commands used in "tentacle_setup.ino" to calibrate EZO circuits and sensors
void calibrate()
{
  Serial.println("calibrate");
  Serial.println("NOT CURRENTLY ENABLED. Calibrate using seperate Arduino sketch.");

} // END calibrate()
*/






