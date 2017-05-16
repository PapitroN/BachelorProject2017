#ifndef CONFIG_H
#define CONFIG_H
//f+//////////////////////////////////////////////////////////////////////////
//
//  Name:	config.h
//
//  This files contain definition of parameters used in the nym�ne nanobubbleproject

//
//  \author Luca Petricca
//
//  \date  14.01.2016
//
//  \modified  Kristian Fremmerlid, Camilla Evensen Eid
//
//  \date      12.12.2016 - 05.01.2017
//
//f-//////////////////////////////////////////////////////////////////////////

#define SLEEPTIME 600000 		//Set the Delay. It must be in miliseconds (There are 1000 milliseconds in a second.) 10minutes=600s=600000ms

//Define wich sensor is active or you want to activate.. true = sensor present/activate... false = sensor disabled

#define ANALOGTEMP   true      //ANALOG temperature sensor attached on the pin A0.. the pin can be changed in the section below
#define ANALOGTEMP2  true      //ANALOG temperature sensor attached on the pin A1.. the pin can be changed in the section below
#define BMESENSOR    true      //BME280 sensor attached to the i2c bus
#define LUXSENSOR    true      //TLS2561 sensor attached to the i2c bus

#define PHTENTACLE   true      //Ph sensor attached on the tentacle board... select the channel below
#define ORPTENTACLE  true      //ORP sensor attached on the tentacle board... select the channel below
#define DOTENTACLE   true      //Dissolved oxigen attached on the tentacle board... select the channel below
#define ECTENTACLE   true      //EC sensor attached on the tentacle board... select the channel below
#define PHTENTACLE2  true      //Ph sensor attached on the tentacle board... select the channel below
#define ORPTENTACLE2 true      //ORP sensor attached on the tentacle board... select the channel below
#define DOTENTACLE2  true      //Dissolved oxigen attached on the tentacle board... select the channel below
#define ECTENTACLE2  true      //EC sensor attached on the tentacle board... select the channel below

#define EXTRADO     false      //Dissolved oxigen attached on the I2C .. this is the extra DO2 sensor that is attached only in the testing phase

//here we set the channels of the different sensors present in the Tentacle board (ph, dissolved oxige, ORP and Conductivity):
// SET the correct channel (0-7) for each sensor based on where are mounted in the tentacle board.
// NB::: If is set to a number > 7 than the sensor is switched off and will not be read

#define ORPCHANNEL   0
#define DOCHANNEL    1
#define PHCHANNEL 	 2
#define ECCHANNEL 	 3
#define PHCHANNEL2   4
#define ECCHANNEL2   5
#define ORPCHANNEL2  6
#define DOCHANNEL2   7


//Define where is attached the analog temperature sensor (default A0)-- change both pin name (A0, A1 A2 etc) and pin number (0, 1 ,2 etc) 

#define ANALOGTEMPPINNUM  0
#define ANALOGTEMPPINNUM2 1
#define ANALOGTEMPPIN     A0
#define ANALOGTEMPPIN2    A1

#endif
