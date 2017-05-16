//h+//////////////////////////////////////////////////////////////////////////
//!
//  \file      Datagramstructurefloat.h
//
//  \brief     This software read data from 4 atlas digital sensors by using softSerial (pin 10 and 11)
//
//  \author    Luca Petricca
//
//  \date      22.02.2016
//
//  \par       Copyright: BroenTech AS
//
//
//h-//////////////////////////////////////////////////////////////////////////


#ifndef _BROENTECH_DATA_STRUCTURE_H
#define _BROENTECH_DATA_STRUCTURE_H

#define DATATYPE_INT "INTE"
#define DATATYPE_FLOAT "FLOA"
#define DATATYPE_SHORT "SHOR"
#define DATATYPE_CHAR "CHAR"
#define VRIBOARD_ID "$V"
#define ARDUINO_ID "$A"
#define CUSTOMBOARD_ID "$C"
#define BOARD_UNIQUE_ID_UNKNOWN "XX"
#define sizeOfPackage 32


// Package header, 16 bytes
typedef struct Broentech_Package_Header
{
   char datagramID[4]; // $VXX = unassigned VRI id, $V01 = assigned VRI id etc | 4 bytes 
   char sensID[4];     // Sensor ID: Temp,  CO_2, Humi                         | 4 bytes 
   char dataType[4];   // Type of the data: INT,DOU,FLO,CHA                    | 4 bytes 
   char dataID[4];     // data labels for the 4 payloads: example: xyz-        | 4 bytes
} Broentech_Package_Header;

// Define 32 byte packages
typedef struct Broentech_Package_INT
{
   Broentech_Package_Header header;
   int data[3]; // | 12 bytes
   char footer[4]; // END\n
} Broentech_Package_INT;

typedef struct Broentech_Package_FLOAT
{
   Broentech_Package_Header header;
   float data[3]; // | 12 bytes
   char footer[4]; // END\n
} Broentech_Package_FLOAT;

typedef struct Broentech_Package_CHAR
{
   Broentech_Package_Header header;
   char data[12]; // | 12 bytes
   char footer[4]; // END\n
} Broentech_Package_CHAR;

typedef struct Broentech_Package_SHORT
{
   Broentech_Package_Header header;
   short data[3]; // | 6 bytes
   char pad[6];  // SPACING | 6 bytes
   char footer[4]; // END\n
} Broentech_Package_SHORT;

typedef struct Broentech_Package_RECEIVED
{
   char datagramID[4];  // $VXX = unassigned VRI id, $V01 = assigned VRI id etc  | 4 bytes
   char ctype[4];       // command type: configure, request data, etc            | 4 bytes
   char opcode[4];      // Command ID: Opcode of the command                     | 4 bytes
   int intdata[3];      // Payload 3 integers 									 | 12 bytes
   char chardata[4];    // payload 4 chars         	                             | 4 bytes
   char address[2];      // address and register to write                        | 2 bytes
   char footer[2];      // END\n												 | 2 bytes
} Broentech_Package_RECEIVED;

typedef union Broentech_Package
{
   Broentech_Package_INT ints;
   Broentech_Package_FLOAT floats;
   Broentech_Package_CHAR chars;
   Broentech_Package_SHORT shorts;
   Broentech_Package_RECEIVED received;
   unsigned char bytes[sizeOfPackage];
} Broentech_Package;

#endif


//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:	sendDatagramfloat
//
//  This function takes a sensor value and a tag, which describes the value, creates a datagram
//  and prints it on the serial port.
//
//  \param  int value - Sensor value
//          char tag  - Sensor value description tag, must be 4 bytes
//
//  \return N/A
//
//  \author Luca Petricca
//
//  \date  09.06.2014
//
//
//f-//////////////////////////////////////////////////////////////////////////

void sendDatagramfloat(char* senid, char* dataid, float value_1, float value_2, float value_3)
{

	// Define package structure 32 byte
	union Broentech_Package SerialPacked;


  int i;
  memcpy(&SerialPacked.floats.header.datagramID  ,"$A02", 4);
  memcpy(&SerialPacked.floats.header.sensID  	 ,senid, 4);
  memcpy(&SerialPacked.floats.header.dataType    ,"DFLO", 4);
  memcpy(&SerialPacked.floats.header.dataID    	 ,dataid, 4);

  memcpy(&SerialPacked.floats.data[0], 		&value_1,  4);
  memcpy(&SerialPacked.floats.data[1], 		&value_2,  4);
  memcpy(&SerialPacked.floats.data[2], 		&value_3,  4);
  memcpy(&SerialPacked.floats.footer,			"0000", 4);


  for(i=0; i<32; i++)
  {
	  Serial.write(SerialPacked.bytes[i]);
  }
}


//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:	sendDatagramfloat
//
//  This function takes a sensor value and a tag, which describes the value, creates a datagram
//  and prints it on the serial port.
//
//  \param  int value - Sensor value
//          char tag  - Sensor value description tag, must be 4 bytes
//
//  \return N/A
//
//  \author Luca Petricca
//
//  \date  09.06.2014
//
//
//f-//////////////////////////////////////////////////////////////////////////

void sendDatagramfloat2(float value_1, float value_2, float value_3)
{

	// Define package structure 32 byte
	union Broentech_Package SerialPacked;


  int i;
  memcpy(&SerialPacked.floats.header.datagramID  ,"$A02", 4);
  memcpy(&SerialPacked.floats.header.sensID  	 ,"FLOW", 4);
  memcpy(&SerialPacked.floats.header.dataType    ,"AFLO", 4);
  memcpy(&SerialPacked.floats.header.dataID    	 ,"F---", 4);

  memcpy(&SerialPacked.floats.data[0], 		&value_1,  4);
  memcpy(&SerialPacked.floats.data[1], 		&value_2,  4);
  memcpy(&SerialPacked.floats.data[2], 		&value_3,  4);
  memcpy(&SerialPacked.floats.footer,			"0000", 4);


  for(i=0; i<32; i++)
  {
	  Serial.write(SerialPacked.bytes[i]);
  }
}
