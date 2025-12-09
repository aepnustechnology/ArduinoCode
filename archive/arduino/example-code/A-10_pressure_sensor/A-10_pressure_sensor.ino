//This code was written to be easy to understand.
//Modify this code as you see fit.
//This code will output data to the Arduino serial monitor.
//This code was written in the Arduino 1.8.5 IDE
//An Arduino UNO and an Arduino Mega were used to test this code.
//This code was last tested 4/2018


int adc = 0;            
float mv=0;
float pressure=0;

int pin = A1;

void setup()
{
  pinMode(pin, INPUT);               //set Analog pin input 
  Serial.begin(9600);               //enable serial port
}

void loop()
{
  adc = analogRead(pin);             //read from the ADC
  mv= adc*.0049;                    //convert ADC points to millivolts
  pressure= (2.5*mv) - 1.25;        //convert millivolts to pressure in PSI
  //pressure = (17.237*mv) - 8.6185;  //convert millivolts to pressure in kPa
  if(pressure <0){ pressure=0;}     //remove negative value if sensors is not under pressure   
  delay(500);                       //this delay is just to slow down the output so you can read it. It is not necessary 
  Serial.println(pressure);         //print pressure value
}

