//h+//////////////////////////////////////////////////////////////////////////
//!
//  \file      Final_code.ino
//
//  \brief     This software read data from 4 atlas digital sensors by using softSerial (pin 10 and 11)
//
//  \author    Luca Petricca
//
//  \date      22.02.2016
//
//  \par       Copyright: BroenTech AS
//
//  \modified  Kristian Fremmerlid, Camilla Evensen Eid
//
//  \date      12.12.2016 - 05.01.2017
//
//h-//////////////////////////////////////////////////////////////////////////

#include <SoftwareSerial.h>              //Include the software serial library. This is used to communicate with the atlas tentacle shield sensors
#include <Adafruit_Sensor.h>             //general adafruit library for sensor
#include <Adafruit_BME280.h>              
#include <Adafruit_TSL2561_U.h>
#include "Datagramstructurefloat.h"
#include "config.h"

SoftwareSerial sSerial(11, 10);          // RX, TX  - Name the software serial library sSerial (this cannot be omitted)
                                         // assigned to pins 10 and 11 for maximum compatibility
const int s0 = 7;                        //Arduino pin 7 to control pin S0
const int s1 = 6;                        //Arduino pin 6 to control pin S1
const int enable_1 = 5;                  //Arduino pin 5 to control pin E on shield 1
const int enable_2 = 4;                  //Arduino pin 4 to control pin E on shield 2

char sensordata[30];                     //A 30 byte character array to hold incoming data from the sensors
byte computer_bytes_received = 0;        //We need to know how many characters bytes have been received
byte sensor_bytes_received = 0;          //We need to know how many characters bytes have been received
int channel;                             //INT pointer for channel switching - 0-7 serial, 8-127 I2C addresses
char *cmd;                               //Char pointer used in string parsing
int retries;                             // com-check functions store number of retries here
boolean answerReceived;                  // com-functions store here if a connection-attempt was successful

String stamp_type;                       // hold the name / type of the stamp
char stamp_version[4];                   // hold the version of the stamp

char computerdata[20];             //we make a 20 byte character array to hold incoming data from a pc/mac/other.

byte i2c_response_code = 0;              //used to hold the I2C response code.
byte in_char = 0;                    //used as a 1 byte buffer to store in bound bytes from an I2C stamp.

//Declare Some variables that we will use for hold the sensor data
float ph;
float dor;
float orp;
float ec;
float temp;
float lux;
float bmetemp;
float bmepres;
float bmehum;

const long validBaudrates[] = {          // list of baudrates to try when connecting to a stamp (they're ordered by probability to speed up things a bit)
  38400, 19200, 9600, 115200, 57600
};
long channelBaudrate[] = {               // store for the determined baudrates for every stamp
  0, 0, 0, 0, 0, 0, 0, 0
};

boolean I2C_mode = false;    //bool switch for serial/I2C


boolean change_channel(void);

//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:  setup()
//
//  This function issue basic setup of the arduino board
//
//  \return N/A
//
//  \author Luca Petricca
//
//  \date  18.01.2016
//
//f-//////////////////////////////////////////////////////////////////////////
void setup() {
  
  pinMode(s1, OUTPUT);                   //Set the digital pin as output.
  pinMode(s0, OUTPUT);                   //Set the digital pin as output.
  pinMode(enable_1, OUTPUT);             //Set the digital pin as output.
  pinMode(enable_2, OUTPUT);             //Set the digital pin as output.
  digitalWrite(ANALOGTEMPPIN, LOW);                 //set pull-up on analog pin for the temperature sensor
  digitalWrite(ANALOGTEMPPIN2, LOW);                 //set pull-up on analog pin for the temperature sensor

  //Start the serial interface
  Serial.begin(9600);                    // Set the hardware serial port to 38400
  while (!Serial) ;                      // Leonardo-type arduinos need this to be able to write to the serial port in setup()
  sSerial.begin(38400);                  // Set the soft serial port to 38400
  stamp_type.reserve(16);                // reserve string buffer to save some SRAM
}

//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:  readtemp()
//
//  This function read and SEND temperature from the analog channel in *Celsius
//
//  \return void
//
//  \author Luca Petricca
//
//  \date  18.01.2016
//
//f-//////////////////////////////////////////////////////////////////////////

void readtemp(void){
 float v_out;                          //voltage output from temp sensor
 float temp;                           //the final temperature is stored here
 v_out = analogRead(ANALOGTEMPPINNUM); //read the input pin
 v_out*=.0048;                         //convert ADC points to volts (we are using .0048 becausethis device is running at 5 volts)
 v_out*=1000;                          //convert volts to millivolts
 temp= 0.0512 * v_out -20.5128;        //the equation from millivolts to temperature
 sendDatagramfloat("TEMP","C---",temp,0,0);
 //Serial.println(temp);
 delay(350); 
}


//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:  readph(int ch), readdor(int ch), readorp(int ch), readec(int ch)
//
//  Next four functions read and send the values of the sensors attached to the tentacle board
//
//  \ch --> channel where they are attached
//
//  \author Luca Petricca
//
//  \date  18.01.2016
//
//f-//////////////////////////////////////////////////////////////////////////

void readph(int ch){
  if (ch<8){
      channel = ch;
      change_channel();
      sSerial.print("r\r");                                             //Send the command from the computer to the Atlas Scientific device using the softserial port
    long timeout=1000000;
    while(sSerial.available()==0 && timeout-- > 0);                     //loop until you receive a byte or the timeout expire
    if (sSerial.available() > 0) {                                      //If data has been transmitted from an Atlas Scientific device
    sensor_bytes_received = sSerial.readBytesUntil(13, sensordata, 30); //we read the data sent from the Atlas Scientific device until we see a <CR>. We also count how many character have been received
    sensordata[sensor_bytes_received] = 0;                              //we add a 0 to the spot in the array just after the last character we received. This will stop us from transmitting incorrect data that may have been left in the buffer
    if (ch<4){
    sendDatagramfloat("PHva","P---",atof(sensordata),0,0);              //send datagram
    delay(350);
    }
    else if (ch<8){
    sendDatagramfloat("PHv2","P---",atof(sensordata),0,0);              //send datagram
    delay(350);
      }    
     }
      
  }
}

void readdor(int ch){
  if (ch<8){
      channel = ch;
      change_channel();
      sSerial.print("r\r");                                             //Send the command from the computer to the Atlas Scientific device using the softserial port
    long timeout=1000000;
    while(sSerial.available()==0 && timeout-- > 0);                     //loop until you receive a byte or the timeout expire
    if (sSerial.available() > 0) {                                      //If data has been transmitted from an Atlas Scientific device
    sensor_bytes_received = sSerial.readBytesUntil(13, sensordata, 30); //we read the data sent from the Atlas Scientific device until we see a <CR>. We also count how many character have been received
    sensordata[sensor_bytes_received] = 0;                              //we add a 0 to the spot in the array just after the last character we received. This will stop us from transmitting incorrect data that may have been left in the buffer
    //Serial.println(sensordata);
    if (ch<4){
    sendDatagramfloat("DORv","p---",atof(sensordata),0,0);              //send datagram
    delay(350);    
    }
    else if (ch<8){
    sendDatagramfloat("DOR2","p---",atof(sensordata),0,0);              //send datagram
    delay(350);      
      }
    }
      
  }
}

void readorp(int ch){
  if (ch<8){
      channel = ch;
      change_channel();
      sSerial.print("r\r");                                             //Send the command from the computer to the Atlas Scientific device using the softserial port
    long timeout=1000000;
    while(sSerial.available()==0 && timeout-- > 0);                     //loop until you receive a byte or the timeout expire
    if (sSerial.available() > 0) {                                      //If data has been transmitted from an Atlas Scientific device
    sensor_bytes_received = sSerial.readBytesUntil(13, sensordata, 30); //we read the data sent from the Atlas Scientific device until we see a <CR>. We also count how many character have been received
    sensordata[sensor_bytes_received] = 0;                              //we add a 0 to the spot in the array just after the last character we received. This will stop us from transmitting incorrect data that may have been left in the buffer
    if (ch<4){
    sendDatagramfloat("ORPv","p---",atof(sensordata),0,0);              //send datagram
    delay(350);    
    }
    else if (ch<8){
    sendDatagramfloat("ORP2","p---",atof(sensordata),0,0);              //send datagram
    delay(350); 
      }
    }
      
  }
}
//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:  readtemp()
//
//  This function read and SEND temperature from the analog channel in *Celsius
//
//  \return void
//
//  \author Luca Petricca
//
//  \date  18.01.2016
//
//f-//////////////////////////////////////////////////////////////////////////

void readec(int ch){
  if (ch<8){
      channel = ch;
      change_channel();
      sSerial.print("r\r");                                                  //Send the command from the computer to the Atlas Scientific device using the softserial port
    long timeout=1000000;
    while(sSerial.available()==0 && timeout-- > 0);                         //loop until you receive a byte or the timeout expire
    if (sSerial.available() > 0) {                                          //If data has been transmitted from an Atlas Scientific device
         sensor_bytes_received = sSerial.readBytesUntil(13, sensordata, 30); //we read the data sent from the Atlas Scientific device until we see a <CR>. We also count how many character have been received
         sensordata[sensor_bytes_received] = 0;                               //we add a 0 to the spot in the array just after the last character we received. This will stop us from transmitting incorrect data that may have been left in the buffer
         //Define the variables and parse the received string into the different readings
         char *EC;
         char *TDS;
         char *SAL;
         char *GRAV;
         float f_ec;
         EC = strtok(sensordata, ",");    
         TDS = strtok(NULL, ",");
         SAL = strtok(NULL, ",");
         GRAV = strtok(NULL, ",");
          //Send the values
         if (ch<4){    //if the channel is 0-3 than we send it as first type
         sendDatagramfloat("ECva","p---",atof(EC),0,0);
         delay(350);                    
         sendDatagramfloat("TDSv","p---",atof(TDS),0,0);
         delay(350);      
         sendDatagramfloat("SALv","p---",atof(SAL),0,0);
         delay(350);    
         sendDatagramfloat("GRAV","p---",atof(GRAV),0,0); 
         delay(350);
         }        
         else{
         sendDatagramfloat("ECv2","p---",atof(EC),0,0);
         delay(350);                    
         sendDatagramfloat("TDS2","p---",atof(TDS),0,0);
         delay(350);      
         sendDatagramfloat("SAL2","p---",atof(SAL),0,0);
         delay(350);    
         sendDatagramfloat("GRA2","p---",atof(GRAV),0,0); 
         delay(350);
         }   
    }
  }
}


//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:  readsecondarydo()
//
//  This function is the main application loop
//
//  \return N/A
//
//  \author Luca Petricca
//
//  \date  18.01.2016
//
//f-//////////////////////////////////////////////////////////////////////////
void readsecondarydo(){
char DO_data[20];
byte in_char=0;
byte code=0;
byte i=0;
float DO_float;
char *DO;
char *sat;
 
 
/* first byte received is the code...to be used in debugging purposes
 switch (code){
        case 1://decimal 1 means the command was successful
            Serial.println("Success");
            break;
        case 2://decimal 2 means the command has failed.
             Serial.println("Failed");
             break;
          case 254://decimal 254 means the command has not yet been finished calculating.
             Serial.println("Pending");
             break;
          case 255://decimal 255 means there is no further data to send.
             Serial.println("No Data");
             break;
 }
 */
  
 sendDatagramfloat("DOEX","p---",atof(DO_data),0,0);              //send datagram
 //Serial.println(atof(DO_data));
 delay(350);    
 
  }

//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:  loop()
//
//  This function is the main application loop
//
//  \return N/A
//
//  \author Luca Petricca
//
//  \date  18.01.2016
//
//f-//////////////////////////////////////////////////////////////////////////

void loop() { 

  while(1) {        //this is the real loop--- we check if the sensor is activated in the config file

      if (ANALOGTEMP) 
        readtemp();           //read and SEND temperature from the analog channel in *Celsius   
      if (PHTENTACLE)
        readph(PHCHANNEL);    //read and SEND ph value from the tentacle shield
      if (DOTENTACLE)
        readdor(DOCHANNEL);   //read and SEND dissolved oxygen from the tentacle shield
      if (ORPTENTACLE)
        readorp(ORPCHANNEL);  //read and SEND temperature from the tentacle shield
      if (ECTENTACLE)
        readec(ECCHANNEL);    //read and SEND conductivity, and other 3 values from the tentacle shield
    //read and send the second shield mounted on the board
      if (PHTENTACLE2)
        readph(PHCHANNEL2);    //read and SEND ph value from the tentacle shield
      if (DOTENTACLE)
        readdor(DOCHANNEL2);   //read and SEND dissolved oxigen from the tentacle shield
      if (ORPTENTACLE)
        readorp(ORPCHANNEL2);  //read and SEND temperature from the tentacle shield
      if (ECTENTACLE2)
        readec(ECCHANNEL2);    //read and SEND conductivity, and other 3 values from the tentacle shield
      if (EXTRADO)
        readsecondarydo();    //read and SEND dissolved oxigen from the I2C channel

      delay(SLEEPTIME);       //Sleeptime configured in the config file
  }
 }

//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:  changechannel()
//
//  This function is used to change serial channel of the tentable board
//
//  \return N/A
//
//  \author Luca Petricca
//
//  \date  18.01.2016
//
//f-//////////////////////////////////////////////////////////////////////////

boolean change_channel() {                                 //function controls which UART/I2C port is opened. returns true if channel could be changed.

  I2C_mode = false;            //0 for serial, 1 for I2C
  stamp_type = "";
  memset(stamp_version, 0, sizeof(stamp_version));         // clear stamp info
  change_serial_mux_channel();                             // configure serial muxer(s) (or disable if we're in I2C mode)

  if (channel < 8) {                                       // serial?
    if (!check_serial_connection()) {                      // determine and set the correct baudrate for this stamp
      return false;
    }
    return true;
  }
}

//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:  change_serial_mux_channel()
//
//  This function is used to change the mux channel of the tentable board
//
//  \return N/A
//
//  \author Luca Petricca
//
//  \date  18.01.2016
//
//f-//////////////////////////////////////////////////////////////////////////
void change_serial_mux_channel() {           // configures the serial muxers depending on channel.

  switch (channel) {                         //Looking to see what channel to open

    case 0:                                  //If channel==0 then we open channel 0
      digitalWrite(enable_1, LOW);           //Setting enable_1 to low activates primary channels: 0,1,2,3
      digitalWrite(enable_2, HIGH);          //Setting enable_2 to high deactivates secondary channels: 4,5,6,7
      digitalWrite(s0, LOW);                 //S0 and S1 control what channel opens
      digitalWrite(s1, LOW);                 //S0 and S1 control what channel opens
      break;                                 //Exit switch case

    case 1:
      digitalWrite(enable_1, LOW);
      digitalWrite(enable_2, HIGH);
      digitalWrite(s0, HIGH);
      digitalWrite(s1, LOW);
      break;

    case 2:
      digitalWrite(enable_1, LOW);
      digitalWrite(enable_2, HIGH);
      digitalWrite(s0, LOW);
      digitalWrite(s1, HIGH);
      break;

    case 3:
      digitalWrite(enable_1, LOW);
      digitalWrite(enable_2, HIGH);
      digitalWrite(s0, HIGH);
      digitalWrite(s1, HIGH);
      break;

    case 4:
      digitalWrite(enable_1, HIGH);
      digitalWrite(enable_2, LOW);
      digitalWrite(s0, LOW);
      digitalWrite(s1, LOW);
      break;

    case 5:
      digitalWrite(enable_1, HIGH);
      digitalWrite(enable_2, LOW);
      digitalWrite(s0, HIGH);
      digitalWrite(s1, LOW);
      break;

    case 6:
      digitalWrite(enable_1, HIGH);
      digitalWrite(enable_2, LOW);
      digitalWrite(s0, LOW);
      digitalWrite(s1, HIGH);
      break;

    case 7:
      digitalWrite(enable_1, HIGH);
      digitalWrite(enable_2, LOW);
      digitalWrite(s0, HIGH);
      digitalWrite(s1, HIGH);
      break;

    default:
      digitalWrite(enable_1, HIGH);   //disable soft serial
      digitalWrite(enable_2, HIGH);   //disable soft serial
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////// FUNCTIONs Below this point where modified from existing example present on the ATLAS SCIENTIFIC WEBSITE //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean check_serial_connection() {                // check the selected serial port. find and set baudrate, request info from the stamp
  // will return true if there is a stamp on this serial channel, false otherwise
  answerReceived = true;                           // will hold if we received any answer. also true, if no "correct" baudrate has been found, but still something answered.
  retries = 0;

  if (channelBaudrate[channel] > 0) {

    sSerial.begin(channelBaudrate[channel]);

    while (retries < 3 && answerReceived == true) {
      answerReceived = false;
      if (request_serial_info()) {
        return true;
      }
    }
  }

  answerReceived = true;                             // will hold if we received any answer. also true, if no "correct" baudrate has been found, but still something answered.

  while (retries < 3 && answerReceived == true) {    // we don't seem to know the correct baudrate yet. try it 3 times (in case it doesn't work right away)

    answerReceived = false;                          // we'll toggle this to know if we received an answer, even if no baudrate matched. probably a com-error, so we'll just retry.
    if (scan_baudrates()) {
      return true;
    }
    retries++;
  }

  return false;   // no stamp was found at this channel
}



boolean scan_baudrates() {                               // scans baudrates to auto-detect the right one for this uart channel. if one is found, it is saved globally in channelBaudrate[]

  for (int j = 0; j < 5; j++) {
    // TODO: make this work for legacy stuff and EZO in uart / continuous mode
    sSerial.begin(validBaudrates[j]);                  // open soft-serial port with a baudrate
    sSerial.print(F("\r"));
    sSerial.flush();                                   // buffers are full of junk, clean up
    sSerial.print(F("c,0\r"));                            // switch off continuous mode for new ezo-style stamps
    delay(150);
    //clearIncomingBuffer();                             // buffers are full of junk, clean up
    sSerial.print(F("e\r"));                              // switch off continous mode for legacy stamps

    delay(150);                                          // give the stamp some time to burp an answer
    clearIncomingBuffer();

    int r_retries = 0;
    answerReceived = true;
    while (r_retries < 3 && answerReceived == true) {
      answerReceived = false;
      if (request_serial_info()) {                         // check baudrate for correctness by parsing the answer to "i"-command
        channelBaudrate[channel] = validBaudrates[j];      // we found the correct baudrate!
        return true;
      }
      r_retries++;
    }
  }
  return false;                                          // we could not determine a correct baudrate
}



boolean request_serial_info() {                        // helper to request info from a uart stamp and parse the answer into the global stamp_ variables

  clearIncomingBuffer();
  sSerial.write("i");                                // send "i" which returns info on all versions of the stamps
  sSerial.write("\r");

  delay(150);                            // give it some time to send an answer

  sensor_bytes_received = sSerial.readBytesUntil(13, sensordata, 9);  //we read the data sent from the Atlas Scientific device until we see a <CR>. We also count how many character have been received

  if (sensor_bytes_received > 0) {                     // there's an answer
    answerReceived = true;                             // so we can globally know if there was an answer on this channel

    if ( parseInfo() ) {                               // try to parse the answer string
      delay(100);
      clearIncomingBuffer();                           // some stamps burp more info (*OK or something). we're not interested yet.
      return true;
    }
  }

  return false;                                        // it was not possible to get info from the stamp
}




boolean parseInfo() {                  // parses the answer to a "i" command. returns true if answer was parseable, false if not.

  // example:
  // PH EZO  -> '?I,pH,1.1'
  // ORP EZO -> '?I,OR,1.0'   (-> wrong in documentation 'OR' instead of 'ORP')
  // DO EZO  -> '?I,D.O.,1.0' || '?I,DO,1.7' (-> exists in D.O. and DO form)
  // EC EZO  -> '?I,EC,1.0 '

  // Legazy PH  -> 'P,V5.0,5/13'
  // Legazy ORP -> 'O,V4.4,2/13'
  // Legazy DO  -> 'D,V5.0,1/13'
  // Legazy EC  -> 'E,V3.1,5/13'

  if (sensordata[0] == '?' && sensordata[1] == 'I') {          // seems to be an EZO stamp

    // PH EZO
    if (sensordata[3] == 'p' && sensordata[4] == 'H') {
      stamp_type = F("pH EZO");
      stamp_version[0] = sensordata[6];
      stamp_version[1] = sensordata[7];
      stamp_version[2] = sensordata[8];
      stamp_version[3] = 0;

      return true;

      // ORP EZO
    }
    else if (sensordata[3] == 'O' && sensordata[4] == 'R') {
      stamp_type = F("ORP EZO");
      stamp_version[0] = sensordata[6];
      stamp_version[1] = sensordata[7];
      stamp_version[2] = sensordata[8];
      stamp_version[3] = 0;
      return true;

      // DO EZO
    }
    else if (sensordata[3] == 'D' && sensordata[4] == 'O') {
      stamp_type = F("D.O. EZO");
      stamp_version[0] = sensordata[6];
      stamp_version[1] = sensordata[7];
      stamp_version[2] = sensordata[8];
      stamp_version[3] = 0;
      return true;

      // D.O. EZO
    }
    else if (sensordata[3] == 'D' && sensordata[4] == '.' && sensordata[5] == 'O' && sensordata[6] == '.') {
      stamp_type = F("D.O. EZO");
      stamp_version[0] = sensordata[8];
      stamp_version[1] = sensordata[9];
      stamp_version[2] = sensordata[10];
      stamp_version[3] = 0;
      return true;

      // EC EZO
    }
    else if (sensordata[3] == 'E' && sensordata[4] == 'C') {
      stamp_type = F("EC EZO");
      stamp_version[0] = sensordata[6];
      stamp_version[1] = sensordata[7];
      stamp_version[2] = sensordata[8];
      stamp_version[3] = 0;
      return true;

      // unknown EZO stamp
    }
    else {
      stamp_type = F("unknown EZO stamp");
      return true;
    }

  }

  // it's a legacy stamp (non-EZO)
  else
  {
    // Legacy pH
    if ( sensordata[0] == 'P') {
      stamp_type = F("pH (legacy)");
      stamp_version[0] = sensordata[3];
      stamp_version[1] = sensordata[4];
      stamp_version[2] = sensordata[5];
      stamp_version[3] = 0;
      return true;

      // legacy ORP
    }
    else if ( sensordata[0] == 'O') {
      stamp_type = F("ORP (legacy)");
      stamp_version[0] = sensordata[3];
      stamp_version[1] = sensordata[4];
      stamp_version[2] = sensordata[5];
      stamp_version[3] = 0;
      return true;

      // Legacy D.O.
    }
    else if ( sensordata[0] == 'D') {
      stamp_type = F("D.O. (legacy)");
      stamp_version[0] = sensordata[3];
      stamp_version[1] = sensordata[4];
      stamp_version[2] = sensordata[5];
      stamp_version[3] = 0;
      return true;

      // Lecagy EC
    }
    else if ( sensordata[0] == 'E') {
      stamp_type = F("EC (legacy)");
      stamp_version[0] = sensordata[3];
      stamp_version[1] = sensordata[4];
      stamp_version[2] = sensordata[5];
      stamp_version[3] = 0;
      return true;
    }
  }

  /*
  Serial.println("can not parse data: ");
   Serial.print("'");
   Serial.print(sensordata);
   Serial.println("'");
   */
  return false;        // can not parse this info-string
}



void clearIncomingBuffer() {          // "clears" the incoming soft-serial buffer
  while (sSerial.available() ) {
    //Serial.print((char)sSerial.read());
    sSerial.read();
  }
}






