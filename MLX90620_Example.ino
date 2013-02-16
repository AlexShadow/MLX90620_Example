/*
 2-16-2013
 Spark Fun Electronics
 Nathan Seidle
 
 This code is heavily based on maxbot's and IlBaboomba's code: http://arduino.cc/forum/index.php?topic=126244
 They didn't have a license on it so I'm hoping it's public domain.
 
 This example shows how to read and calculate the 64 temperatures for the 64 pixels of the MLX90620 thermopile sensor.
 
 Attention! I commented out the alpha_ij array, so if you're going to compile the sketch you'll get for sure an error.
 You should replace all 64 values with the alpha_ij calculated using the values stored in your MLX90620's EEPROM. 
 I suggest you to make an EEPROM dump, print it on the Serial port and store it in a file. From there, 
 with the help of a spreadsheet (Libreoffice, Google Docs, Excel...) calculate your own alpha_ij values. 
 Please also pay attention to your emissivity value: since in my case it was equal to 1, to save SRAM i 
 cut out that piece of calculation. You need to restore those lines if your emissivity value is not equal to 1. 
 
 Don't get confused by the bottom view of the device! The GND pin is connected to the housing.
 
 To get this code to work, attached a MLX90620 to an Arduino Uno using the following pins:
 A5 to 330 ohm to SCL
 A4 to 330 ohm to SDA
 3.3V to VDD
 GND to VSS
 
 I used the internal pull-ups on the SDA/SCL lines. Normally you should use ~4.7k pull-ups for I2C.
 
 */

#include <i2cmaster.h>
//i2cmaster comes from here: http://www.cheap-thermocam.bplaced.net/software/I2Cmaster.rar

#include "MLX90620_registers.h"

int refreshRate = 16; //Set this value to your desired refresh frequency

int irData[64]; //Contains the most current IR temperature readings from the sensor
float temperatures[64];

int v_th;
float to;

//These variables are used 
float Tambient;

//These are constants calculated from the calibration data stored in EEPROM
//See varInitialize and section 7.3 for more information
int a_cp, b_cp, tgc, b_i_scale;
float k_t1, k_t2, emissivity;
int a_ij[64], b_ij[64];

float alpha_ij[64] = {
  1.591E-8, 1.736E-8, 1.736E-8, 1.620E-8, 1.783E-8, 1.818E-8, 1.992E-8, 1.748E-8, 1.864E-8, 2.056E-8, 2.132E-8, 2.033E-8, 2.097E-8, 2.324E-8, 2.388E-8, 2.161E-8, 2.155E-8, 2.394E-8, 2.353E-8, 2.068E-8, 2.353E-8, 2.633E-8, 2.708E-8, 2.394E-8, 2.499E-8, 2.778E-8, 2.731E-8, 2.580E-8, 2.539E-8, 2.796E-8, 2.871E-8, 2.598E-8, 2.586E-8, 2.801E-8, 2.830E-8, 2.633E-8, 2.609E-8, 2.894E-8, 2.924E-8, 2.633E-8, 2.464E-8, 2.778E-8, 2.894E-8, 2.673E-8, 2.475E-8, 2.737E-8, 2.796E-8, 2.679E-8, 2.394E-8, 2.708E-8, 2.714E-8, 2.644E-8, 2.347E-8, 2.563E-8, 2.493E-8, 2.388E-8, 2.179E-8, 2.440E-8, 2.504E-8, 2.295E-8, 2.033E-8, 2.283E-8, 2.295E-8, 2.155E-8};  //<-- REPLACE THIS VALUES WITH YOUR OWN!

byte loopCount = 0; //Used in main loop

//From the 256 bytes of EEPROM data, initialize 
void varInitialization(byte calibration_data[])
{
  v_th = (calibration_data[VTH_H] << 8) + calibration_data[VTH_L];
  k_t1 = ((calibration_data[KT1_H] << 8) + calibration_data[KT1_L]) / 1024.0; //2^10 = 1024
  k_t2 = ((calibration_data[KT2_H] << 8) + calibration_data[KT2_L]) / 1048576.0; //2^20 = 1,048,576

  a_cp = calibration_data[CAL_ACP];
  if(a_cp > 127) a_cp -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

  b_cp = calibration_data[CAL_BCP];
  if(b_cp > 127) b_cp -= 256;

  tgc = calibration_data[CAL_TGC];
  if(tgc > 127) tgc -= 256;

  b_i_scale = calibration_data[CAL_BI_SCALE];

  emissivity = (((unsigned int)calibration_data[CAL_EMIS_H] << 8) + calibration_data[CAL_EMIS_L]) / 32768.0;

  for(int i = 0 ; i < 64 ; i++)
  {
    //Read the individual pixel offsets
    a_ij[i] = calibration_data[i]; 
    if(a_ij[i] > 127) a_ij[i] -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

    //Read the individual pixel offset slope coefficients
    b_ij[i] = calibration_data[0x40 + i]; //Bi(i,j) begins 64 bytes into EEPROM at 0x40
    if(b_ij[i] > 127) b_ij[i] -= 256;
  }
}

//Receives the refresh rate for sensor scanning
//Sets the two byte configuration registers
//This function overwrites what is currently in the configuration registers
//The MLX doesn't seem to mind this (flags are read only)
void setConfiguration(int irRefreshRateHZ)
{
  byte Hz_LSB;

  switch(irRefreshRateHZ)
  {
  case 0:
    Hz_LSB = 0b00001111;
    break;
  case 1:
    Hz_LSB = 0b00001110;
    break;
  case 2:
    Hz_LSB = 0b00001101;
    break;
  case 4:
    Hz_LSB = 0b00001100;
    break;
  case 8:
    Hz_LSB = 0b00001011;
    break;
  case 16:
    Hz_LSB = 0b00001010;
    break;
  case 32:
    Hz_LSB = 0b00001001;
    break;
  default:
    Hz_LSB = 0b00001110;
  }

  byte defaultConfig_H = 0b01110100; // x111.01xx, Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz

  i2c_start_wait(MLX90620_WRITE);
  i2c_write(0x03); //Command = configuration value
  i2c_write((byte)Hz_LSB - 0x55);
  i2c_write(Hz_LSB);
  i2c_write(defaultConfig_H - 0x55); //Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz
  i2c_write(defaultConfig_H);
  i2c_stop();
}

//Read the 256 bytes from the MLX EEPROM and setup the various constants (*lots* of math)
//Note: The EEPROM on the MLX has a different I2C address from the MLX. I've never seen this before.
void read_EEPROM_MLX90620()
{
  byte eepromData[256];

  i2c_start_wait(MLX90620_EEPROM_WRITE);
  i2c_write(0x00); //EEPROM info starts at location 0x00
  i2c_rep_start(MLX90620_EEPROM_READ);

  //Read all 256 bytes from the sensor's EEPROM
  for(int i = 0 ; i <= 255 ; i++)
    eepromData[i] = i2c_readAck();

  i2c_stop(); //We're done talking

  varInitialization(eepromData); //Calculate a bunch on constants from the EEPROM data

  writeTrimmingValue(eepromData[OSC_TRIM_VALUE]);
}

//Given a 8-bit number from EEPROM (Slave address 0x50), write value to MLX sensor (Slave address 0x60)
void writeTrimmingValue(byte val)
{
  i2c_start_wait(MLX90620_WRITE); //Write to the sensor
  i2c_write(0x04); //Command = write oscillator trimming value
  i2c_write((byte)val - 0xAA);
  i2c_write(val);
  i2c_write(0x56); //Always 0x56
  i2c_write(0x00); //Always 0x00
  i2c_stop();
}

//Gets the latest PTAT (package temperature ambient) reading from the MLX
//Then calculates a new Tambient
//Many of these values (k_t1, v_th, etc) come from varInitialization and EEPROM reading
void calculate_TA(void)
{
  unsigned int ptat = readPTAT_MLX90620();

  Tambient = (-k_t1 + sqrt(square(k_t1) - (4 * k_t2 * (v_th - (float)ptat)))) / (2*k_t2) + 25; //it's much more simple now, isn't it? :)
}

//Reads the PTAT data from the MLX
//Returns an unsigned int containing the PTAT
unsigned int readPTAT_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read PTAT
  i2c_write(0x90); //Start address is 0x90
  i2c_write(0x00); //Address step is 0
  i2c_write(0x01); //Number of reads is 1
  i2c_rep_start(MLX90620_READ);

  byte ptatLow = i2c_readAck(); //Grab the lower and higher PTAT bytes
  byte ptatHigh = i2c_readAck();

  i2c_stop();

  return( (unsigned int)(ptatHigh << 8) | ptatLow); //Combine bytes and return
}

//Calculate the temperatures seen for each pixel
//Relies on the raw irData array
//Returns an 64-int array called temperatures
void calculate_TO()
{
  float v_ir_off_comp;
  float v_ir_tgc_comp;
  //float v_ir_comp; //Removed to save SRAM, in my case v_ir_comp == v_ir_tgc_comp

  //Calculate the offset compensation for the one compensation pixel
  //This is a constant in the TO calculation, so calculate it here.
  int cpix = readCPIX_MLX90620(); //Go get the raw data of the compensation pixel
  float v_cp_off_comp = (float)cpix - (a_cp + (b_cp/pow(2, b_i_scale)) * (Tambient - 25)); 

  for (int i = 0 ; i < 64 ; i++)
  {
    v_ir_off_comp = irData[i] - (a_ij[i] + (float)(b_ij[i]/pow(2, b_i_scale)) * (Tambient - 25)); //#1: Calculate Offset Compensation 

    v_ir_tgc_comp = v_ir_off_comp - ( ((float)tgc/32) * v_cp_off_comp); //#2: Calculate Thermal Gradien Compensation (TGC)

    //v_ir_comp = v_ir_tgc_comp / emissivity; //#3: Calculate Emissivity Compensation - Removed to save SRAM, since emissivity in my case is equal to 1. 

    //temperatures[i] = sqrt(sqrt((v_ir_comp/alpha_ij[i]) + pow((Tambient + 273.15), 4))) - 273.15;
    temperatures[i] = sqrt(sqrt((v_ir_tgc_comp/alpha_ij[i]) + pow((Tambient + 273.15), 4))) - 273.15;	//#4: Calculated TO - Edited to work with v_ir_tgc_comp instead of v_ir_comp
  }
}

//Reads 64 bytes of pixel data from the MLX
//Loads the data into the irData array
void readIR_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read a register
  i2c_write(0x00); //Start address = 0x00
  i2c_write(0x01); //Address step = 1
  i2c_write(0x40); //Number of reads is 64
  i2c_rep_start(MLX90620_READ);

  for(int i = 0 ; i < 64 ; i++)
  {
    byte pixelDataLow = i2c_readAck();
    byte pixelDataHigh = i2c_readAck();
    irData[i] = (int)(pixelDataHigh << 8) | pixelDataLow;
  }

  i2c_stop();
}

//Read the compensation pixel 16 bit data
int readCPIX_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read register
  i2c_write(0x91);
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(MLX90620_READ);

  byte cpixLow = i2c_readAck(); //Grab the two bytes
  byte cpixHigh = i2c_readAck();
  i2c_stop();

  return ( (int)(cpixHigh << 8) | cpixLow);
}

//Reads the current configuration register (2 bytes) from the MLX
//Returns two bytes
unsigned int readConfig_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE); //The MLX configuration is in the MLX, not EEPROM
  i2c_write(CMD_READ_REGISTER); //Command = read configuration register
  i2c_write(0x92); //Start address
  i2c_write(0x00); //Address step of zero
  i2c_write(0x01); //Number of reads is 1

    i2c_rep_start(MLX90620_READ);

  byte configLow = i2c_readAck(); //Grab the two bytes
  byte configHigh = i2c_readAck();

  i2c_stop();

  return( (unsigned int)(configHigh << 8) | configLow); //Combine the configuration bytes and return as one unsigned int
}


//Poll the MLX for its current status
//Returns true if the POR/Brown out bit is set
boolean checkConfig_MLX90620()
{
  if ( (readConfig_MLX90620() & (unsigned int)1<<POR_TEST) == 0)
    return true;
  else
    return false;
}

//Prints the temperatures array in a single 64 byte block
void printTemperatures()
{
  for(int i = 0 ; i < 64 ; i++)
  {
    Serial.println(temperatures[i]);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("MLX90620 Example");

  i2c_init(); //Init the I2C pins
  PORTC = (1 << PORTC4) | (1 << PORTC5); //Enable pull-ups

  delay(5); //Init procedure calls for a 5ms delay after power-on

  read_EEPROM_MLX90620(); //Read the entire EEPROM

  setConfiguration(refreshRate); //Configure the MLX sensor with the user's choice of refresh rate
}

void loop()
{
  if(loopCount++ == 16)
  { //Tambient changes more slowly than the pixel readings. Update TA only every 16 loops.
    calculate_TA(); //Calculate the new Tambient

    if(checkConfig_MLX90620()) //Every 16 readings check that the POR flag is not set
    {
      Serial.println("POR Detected!");
      setConfiguration(refreshRate); //Re-write the configuration bytes to the MLX
    }

    loopCount = 0; //Reset count
  }

  readIR_MLX90620(); //Get the 64 bytes of raw pixel data into the irData array
  
  calculate_TO(); //Run all the large calculations to get the temperature data for each pixel
  
  printTemperatures(); //Send all 64 floats to serial port
}


