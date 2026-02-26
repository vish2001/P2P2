#pragma once
#include <Arduino.h>
#include <SPI.h>

class DW3000Class{
public:
  static int config[9];
      
  // Chip Setup
  static void spiSelect(uint8_t cs);

  static void begin();
  static void init();

  static void writeSysConfig();
  static void configureAsTX();
  static void setupGPIO();

  // Double-Sided Ranging
  static void ds_sendFrame(int stage);
  static void ds_sendRTInfo(uint32_t t_roundB, uint32_t t_replyB);
  static int ds_processRTInfo(uint32_t t_roundA, uint32_t t_replyA, uint32_t t_roundB, uint32_t t_replyB, int clock_offset);
  static int ds_getStage();
  static bool ds_isErrorFrame();
  static void ds_sendErrorFrame();

  // Radio Settings
  static void setChannel(uint8_t data);
  static void setPreambleLength(uint8_t data);
  static void setPreambleCode(uint8_t data);
  static void setPACSize(uint8_t data);
  static void setDatarate(uint8_t data);
  static void setPHRMode(uint8_t data);
  static void setPHRRate(uint8_t data);

  // Protocol Settings
  static void setMode(int mode);
  static void setTXFrame(unsigned long long frame_data);
  static void setFrameLength(int frame_len);
  static void setTXAntennaDelay(int delay);
  static void setSenderID(int senderID);
  static void setDestinationID(int destID);

  // Status Checks
  static int receivedFrameSucc();
  static int sentFrameSucc();
  static int getSenderID();
  static int getDestinationID();
  static bool checkForIDLE();
  static bool checkSPI();

  // Radio Analytics
  static double getSignalStrength();
  static double getFirstPathSignalStrength();
  static int getTXAntennaDelay();
  static long double getClockOffset();
  static long double getClockOffset(int32_t ext_clock_offset);
  static int getRawClockOffset();
  static float getTempInC();

  static unsigned long long readRXTimestamp();
  static unsigned long long readTXTimestamp();

  // Chip Interaction
  static uint32_t write(int base, int sub, uint32_t data, int data_len);
  static uint32_t write(int base, int sub, uint32_t data);

  static uint32_t read(int base, int sub);
  static uint8_t read8bit(int base, int sub);
  static uint32_t readOTP(uint8_t addr);

  // Delayed Sending Settings
  static void writeTXDelay(uint32_t delay);
  static void prepareDelayedTX();

  // Radio Stage Settings / Transfer and Receive Modes
  static void delayedTXThenRX();
  static void delayedTX();
  static void standardTX();
  static void standardRX();
  static void TXInstantRX();

  // DWM3000 Firmware Interaction
  static void softReset();
  static void hardReset();
  static void clearSystemStatus();

  // Hardware Status Information
  static void pullLEDHigh(int led);
  static void pullLEDLow(int led);

  // Calculation and Conversion
  static double convertToCM(int DWM3000_ps_units);
  static void calculateTXRXdiff();

  // Printing
  static void printRoundTripInformation();
  static void printDouble(double val, unsigned int precision, bool linebreak);

private:
  // Single Bit Settings
  static void setBit(int reg_addr, int sub_addr, int shift, bool b);
  static void setBitLow(int reg_addr, int sub_addr, int shift);
  static void setBitHigh(int reg_addr, int sub_addr, int shift);

  // Fast Commands
  static void writeFastCommand(int cmd);

  // SPI Interaction
  static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit);
  static uint32_t sendBytes(int b[], int lenB, int recLen);

  // Soft Reset Helper Method
  static void clearAONConfig();

  // Other Helper Methods
  static unsigned int countBits(unsigned int number);
  static int checkForDevID();
  static uint32_t readDeviceID();
};