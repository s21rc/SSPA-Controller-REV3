
/*  Arduino Code (Version 3.0.3) for S21RC SSPA Controller REV3.0.2 PCB.                    */
/*                                                                                          */
/*  Created by Fazlay Rabby S21RC, JUNE, 2023.                                              */
/*                                                                                          */
/*  RELEASED AS OPEN SOURSE UNDER MIT LICENSE                                               */
/*  Feel free to use/change the Gerber file for the PCB,                                    */
/*  display file(nextion TFT) for the display and all codes here.                           */
/*                                                                                          */
/*  If you need any clarification or help please issue a ticket in github.                  */
/*                                                                                          */

/* For PCB Version REV3.0.2, CODE V 3.0.3, Display: Nextion 800x480 (5 and 7 inch)          */

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* THE CODE IS STILL IN TESTING PHASE, WORK IN PROGRESS. LAST UPDATE: 1700UTC 3 MARCH 2024  */
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include "Arduino.h"
#include <EEPROM.h>
#include "math.h"
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>  //https://github.com/milesburton/Arduino-Temperature-Control-Library
#include "EasyNextionLibrary.h" // https://github.com/Seithan/EasyNextionLibrary
#include <MCP23008.h>           //https://registry.platformio.org/libraries/robtillaart/MCP23008
#include <pinout.h>

MCP23008 MCP(0x20, &Wire2);

EasyNex myNex(UART_DISP);

/* ======= Tempareture variables ======== */
// Digital Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
uint8_t resolution = 9;
unsigned long last_Temp_Refresh = 0;

float temp_digi1 = 0.0;
float temp_digi2 = 0.0;
float last_temp_digi1 = 0.0;
float last_temp_digi2 = 0.0;

// NTC temp
#define ntcbeta 7900 // 3950 for 10K 3950 NTC

uint16_t adc_ntc1 = 0;
uint16_t adc_ntc2 = 0;
float res_ntc1 = 0.0;
float res_ntc2 = 0.0;

float temp_ntc1 = 0.0;
float temp_ntc2 = 0.0;
float last_temp_ntc1 = 0.0;
float last_temp_ntc2 = 0.0;

/* =========== Variable for inturrupt status ============*/
volatile bool extprot_status = LOW;
volatile bool antswr_status = LOW;
volatile bool inpo_status = LOW;
volatile bool lpfswr_status = LOW;

volatile bool RF_IN_status = LOW;
volatile bool BIAS_ON_status = LOW;
volatile bool ANT_RXTX_status = LOW;
volatile bool PROT_OUT_status = LOW;

// Structure for display power and SWR
struct power
{
  float powerFWD;     // power forward (watts)
  float rawFWD;       // power forward for SWR calculation
  float PEAKpowerFWD; // power forward max (for peak hold)
  float powerREF;     // power reflected (watts)
  float rawREF;       // power reflected for SWR calculation
  float PEAKpowerREF; // power reflected max (for peak hold)
  float SWR;
  float PEAKSWR;
  float power_ratio;
  uint8_t graphSWR;
  uint8_t maxgraphSWR;
  uint8_t swr_display;
  uint16_t graph_Watt;
  uint16_t maxGraphWatt;
  float calibration;
  unsigned long lastPEP;
};

power ANT;
power LPF;
power IN;

/* ======= Variables to save seetings data in EEPROM ======= */
// Structure for band related values
struct band
{
  uint8_t meterband;
  uint8_t lpf;
  uint8_t bcd;
  uint16_t volt;
  uint16_t rotary;
};
band b1;
band b2;
band b3;
band b4;
band b5;
band b6;
band b7;
band b8;
band b9;
band b10;
band b11;

uint8_t band_mode;

uint16_t protection_po;
float protection_swr;
uint8_t protection_swr_mem;
uint8_t protection_temp;
uint8_t protection_in;
uint8_t protection_vdd;
uint8_t protection_id;

uint16_t graph_maxWatt;
uint8_t graph_maxTemp;

uint16_t T_pepHOLD;
uint8_t PWM_FREQ_KHZ;

uint8_t dtemp1_status;
uint8_t dtemp2_status;
uint8_t ntc1_status;
uint8_t ntc2_status;
uint8_t vband_status;
uint8_t bcdband_status;
uint8_t rotband_status;
uint8_t band8_status;
boolean CATband_status;

/* CAT/CIV related variables*/
byte reqQRG[10];
uint8_t CATaddress = 0x70;
uint16_t CATbaud;
uint8_t CATrig;
const byte cat_length = 12;
byte cat_data[cat_length];
uint16_t cat_MHz;
uint16_t cat_10KHz;
uint16_t cat_10Hz;
boolean newData = false;
byte startMarker = 0xFE;
byte endMarker = 0xFD;
byte cat_byte;

/* **********************************RADIO CAT PROFILE *************************************************** */

String rig1 = "IC-7000";
String rig2 = "No Profile";  // rename to add new radio profile
String rig3 = "No Profile";  // rename to add new radio profile
String rig4 = "No Profile";  // rename to add new radio profile
String rig5 = "No Profile";  // rename to add new radio profile
String rig6 = "No Profile";  // rename to add new radio profile
String rig7 = "No Profile";  // rename to add new radio profile
String rig8 = "No Profile";  // rename to add new radio profile
String rig9 = "No Profile";  // rename to add new radio profile
String rig10 = "No Profile"; // rename to add new radio profile
String rig11 = "No Profile"; // rename to add new radio profile
String rig12 = "No Profile"; // rename to add new radio profile

byte Cn_returnQRG;
uint8_t address_indx;
uint8_t Cn_indx;
uint8_t Sc_indx;
uint8_t Cn_byte;
uint8_t Sc_byte;

uint8_t data1_indx;
uint8_t data2_indx;
uint8_t data3_indx;
uint8_t data4_indx;
uint8_t data5_indx;
uint8_t ptt_indx;

void RIGprofile()
{

  switch (CATrig)
  {
  case 1:
    // ICOM IC-7300 CI-V
    // Frequecy change broadcast from Radio: [0]FE [1]FE [2]00 [3]70 [4]00 [5]XX [6]XX [7]XX [8]XX [9]XX [10]FD
    // PTT request:
    // OK message to controller: FE FE E0 94 FB FD

    Cn_returnQRG = 03;
    address_indx = 3;
    Cn_byte = 0x00;
    Cn_indx = 4;
    Sc_indx = 5;
    data1_indx = 5;
    data2_indx = 6;
    data3_indx = 7;
    data4_indx = 8;
    data5_indx = 9;

    // Frequency Query: 0xFE, 0xFE, 0x70, 0xE0, 0x03, 0xFD
    reqQRG[0] = 0xFE;
    reqQRG[1] = 0xFE;
    reqQRG[2] = CATaddress;
    reqQRG[3] = 0xE0;
    reqQRG[4] = 0x03;
    reqQRG[5] = 0xFD;

    break;

  case 2:
    // Radiio profile will go here
    break;

  case 3:
    // Radiio profile will go here
    break;

  case 4:
    // Radiio profile will go here
    break;

  case 5:
    // Radiio profile will go here
    break;

  case 6:
    // Radiio profile will go here
    break;

  case 7:
    // Radiio profile will go here
    break;

  case 8:
    // Radiio profile will go here
    break;

  case 9:
    // Radiio profile will go here
    break;

  case 10:
    // Radiio profile will go here
    break;

  case 11:
      // Radiio profile will go here
      break;

      case 12:
    // Radiio profile will go here
    break;
  default:
    break;
  }
}
/* USER DEFINED VARIABLES */
uint8_t graph_maxSwr = 3;            // = 3; // Scale of the SWR for display
unsigned long Temp_Refresh = 0;      //
unsigned long Display_Refresh = 500; // refresh Temp display every N mili sec, set as per your liking
unsigned long Volt_Refresh = 500;    // refresh V display every N mili sec, set as per your liking
unsigned long ID_Refresh = 500;      // refresh I display every N mili sec, set as per your liking
unsigned long Effi_Refresh = 500;    // refresh Efficiency display every N mili sec, set as per your liking
unsigned long Power_Refresh = 200;
unsigned long Diag_Refresh = 200;
unsigned long bandcalib_Refresh = 200;
unsigned long metercalib_Refresh = 200;
float ResV1 = 10000.00; // Set R1 of voltage devider
float ResV2 = 470.00;   // Set R2 of voltage devider
float ResP1 = 1500.00;  // Set R1 of voltage devider for power/swr input
float ResP2 = 470.00;   // Set R2 of voltage devider for power/swr input
uint16_t RL = 2530;     // Load Resistor at Current sensor

uint8_t band_margin = 20; // around 90mV margin for change of voltage reference between SSPA and Radio

float ref_ADC = (3.3 / 1024); // Refrence is 3.3v for Teensy ADC

// Calibration factors
uint16_t measured_power_out;
uint16_t measured_power_in;

float VFdiode = 0.3; //

float maxCalibV_out;
float CalibV_out;
float maxCalibV_in;
float CalibV_in;

bool calibpage_load = false;
uint8_t last_calib_band = 99;
uint16_t last_calib_out = 0;
uint16_t last_calib_in = 0;

float calibration_ID = 1.0;
uint16_t calib_ID = 0;
uint16_t peak_ID = 0;
uint16_t last_peak_ID = 0;

/* ======== Other variables ======== */
uint8_t Eff = 0;
uint8_t display_page = 2;

bool touch_status = true;
bool PTT_status = false;
bool TX_Enable = true;
bool error_temp_status = false;
bool antenna2_status = false;
bool last_antenna2 = false;

uint8_t last_display_lpf = 0;
uint8_t last_lpf_relay = 0;
uint8_t last_display_mband = 0;

uint8_t current_lpf = 1;
uint8_t current_band = 1;
uint8_t last_band = 0;

uint8_t lpf_bank = 0;

unsigned long last_Power_Refresh = 0;
unsigned long lastTRequest = 0;
unsigned long lastVRequest = 0;
unsigned long last_ID_Refresh = 0;
unsigned long lastERequest = 0;
unsigned long last_Display_Refresh = 0;
unsigned long last_Diag_Refresh = 0;
unsigned long last_bandcalib_Refresh = 0;
unsigned long last_metercalib_Refresh = 0;

// char error_text[20] = "";
uint8_t default_value = 0;

volatile uint8_t alarm_code = 0;
uint8_t last_alarm_code = 99;

uint16_t V = 0;
uint16_t lastV = 0;
float ResV = ResV2 / (ResV1 + ResV2);
uint16_t I;
uint16_t lastI = 0;

uint8_t graph_watt = 0;
uint8_t graph_Temp1 = 0;
uint8_t graph_Temp2 = 0;
uint8_t graph_Temp3 = 0;
uint8_t graph_Temp4 = 0;

/* ======== power and swr related variable ======== */

float ResP = ResP2 / (ResP1 + ResP2); // Voltage devider for SWR/POWER/LPF

/* === PWM Function === */
int PWM = 0;
int last_PWM = 0;

/*================= FUNCTIONS ================================*/

// Returns Decimal value from BCD input
uint8_t band_bcd()
{
  uint8_t band_bcd_decimal = 0;
  uint8_t A = !digitalRead(BAND_A);
  uint8_t B = !digitalRead(BAND_B);
  uint8_t C = !digitalRead(BAND_C);
  uint8_t D = !digitalRead(BAND_D);

  band_bcd_decimal = (A * 1 + B * 2 + C * 4 + D * 8);
  return band_bcd_decimal;
}

// Band Mapping from BCD to Band#
uint8_t bcd2band(uint8_t band_bcd_decimal)
{
  uint8_t band = 0;
  if (band_bcd_decimal == b1.bcd)
    band = 1; // 160m
  else if (band_bcd_decimal == b2.bcd)
    band = 2; // 80m
  else if (band_bcd_decimal == b3.bcd)
    band = 3; // 40m
  else if (band_bcd_decimal == b4.bcd)
    band = 4; // 30m
  else if (band_bcd_decimal == b5.bcd)
    band = 5; // 20m
  else if (band_bcd_decimal == b6.bcd)
    band = 6; // 17m
  else if (band_bcd_decimal == b7.bcd)
    band = 7; // 15m
  else if (band_bcd_decimal == b8.bcd)
    band = 8; // 12m
  else if (band_bcd_decimal == b9.bcd)
    band = 9; // 10m
  else if (band_bcd_decimal == b10.bcd)
    band = 10; // 6m
  return band;
}

// Returns ADC value from band volt input
uint16_t band_volt()
{
  return analogRead(BAND_V);
  // return band_volt_adc;
}

// Band Mapping from BAND Volatage to Band#
uint8_t bandV2band(uint8_t band_volt_adc)
{
  uint8_t band = 0;
  if (band_volt_adc > (b1.volt - band_margin) && band_volt_adc < (b1.volt + band_margin))
    band = 1;
  else if (band_volt_adc > (b2.volt - band_margin) && band_volt_adc < (b2.volt + band_margin))
    band = 2;
  else if (band_volt_adc > (b3.volt - band_margin) && band_volt_adc < (b3.volt + band_margin))
    band = 3;
  else if (band_volt_adc > (b4.volt - band_margin) && band_volt_adc < (b4.volt + band_margin))
    band = 4;
  else if (band_volt_adc > (b5.volt - band_margin) && band_volt_adc < (b5.volt + band_margin))
    band = 5;
  else if (band_volt_adc > (b6.volt - band_margin) && band_volt_adc < (b6.volt + band_margin))
    band = 6;
  else if (band_volt_adc > (b7.volt - band_margin) && band_volt_adc < (b7.volt + band_margin))
    band = 7;
  else if (band_volt_adc > (b8.volt - band_margin) && band_volt_adc < (b8.volt + band_margin))
    band = 8;
  else if (band_volt_adc > (b9.volt - band_margin) && band_volt_adc < (b9.volt + band_margin))
    band = 9;
  else if (band_volt_adc > (b10.volt - band_margin) && band_volt_adc < (b10.volt + band_margin))
    band = 10;
  else if (band_volt_adc > (b11.volt - band_margin) && band_volt_adc < (b11.volt + band_margin))
    band = 11;
  return band;
}

uint8_t band2lpf(uint8_t band)
{
  switch (band)
  {
  case 1:
    return b1.lpf;
    break;

  case 2:
    return b2.lpf;
    break;

  case 3:
    return b3.lpf;
    break;

  case 4:
    return b4.lpf;
    break;

  case 5:
    return b5.lpf;
    break;

  case 6:
    return b6.lpf;
    break;

  case 7:
    return b7.lpf;
    break;

  case 8:
    return b8.lpf;
    break;

  case 9:
    return b9.lpf;
    break;

  case 10:
    return b10.lpf;
    break;

  case 11:
    return b11.lpf;
    break;

  default:
    return 0;
    break;
  }
}

uint8_t lpf2band(uint8_t lpf)
{
  if (b1.lpf == lpf)
    return 1;
  else if (b2.lpf == lpf)
    return 2;
  else if (b3.lpf == lpf)
    return 3;
  else if (b4.lpf == lpf)
    return 4;
  else if (b5.lpf == lpf)
    return 5;
  else if (b6.lpf == lpf)
    return 6;
  else if (b7.lpf == lpf)
    return 7;
  else if (b8.lpf == lpf)
    return 8;
  else if (b9.lpf == lpf)
    return 9;
  else if (b10.lpf == lpf)
    return 10;
  else if (b11.lpf == lpf)
    return 11;
  else
    return 0;
}

// Returns ADC value from Manual Rotary switch input
uint16_t band_rotary()
{

  return analogRead(ROTARY_POS);
}

// Band Mapping from ROTARY to Band#
uint8_t rotary2band(uint16_t band_rotary_adc)
{
  uint8_t band = 0;
  if (band_rotary_adc > (b1.rotary - band_margin) && band_rotary_adc < (b1.rotary + band_margin))
    band = 1;
  else if (band_rotary_adc > (b2.rotary - band_margin) && band_rotary_adc < (b2.rotary + band_margin))
    band = 2;
  else if (band_rotary_adc > (b3.rotary - band_margin) && band_rotary_adc < (b3.rotary + band_margin))
    band = 3;
  else if (band_rotary_adc > (b4.rotary - band_margin) && band_rotary_adc < (b4.rotary + band_margin))
    band = 4;
  else if (band_rotary_adc > (b5.rotary - band_margin) && band_rotary_adc < (b5.rotary + band_margin))
    band = 5;
  else if (band_rotary_adc > (b6.rotary - band_margin) && band_rotary_adc < (b6.rotary + band_margin))
    band = 6;
  else if (band_rotary_adc > (b7.rotary - band_margin) && band_rotary_adc < (b7.rotary + band_margin))
    band = 7;
  else if (band_rotary_adc > (b8.rotary - band_margin) && band_rotary_adc < (b8.rotary + band_margin))
    band = 8;
  else if (band_rotary_adc > (b9.rotary - band_margin) && band_rotary_adc < (b9.rotary + band_margin))
    band = 9;
  else if (band_rotary_adc > (b10.rotary - band_margin) && band_rotary_adc < (b10.rotary + band_margin))
    band = 10;
  else if (band_rotary_adc > (b11.rotary - band_margin) && band_rotary_adc < (b11.rotary + band_margin))
    band = 11;
  return band;
}

void loadEEPROM()
{
  EEPROM.get(500, CalibV_out);
  EEPROM.get(504, CalibV_in);
  EEPROM.get(508, measured_power_out);
  EEPROM.get(512, measured_power_in);
  EEPROM.get(516, calibration_ID);
  EEPROM.get(520, ANT.calibration);
  EEPROM.get(524, IN.calibration);

  EEPROM.get(21, ANT.maxGraphWatt);
  EEPROM.get(25, graph_maxTemp);
  EEPROM.get(30, protection_po);
  EEPROM.get(35, protection_swr_mem);

  EEPROM.get(40, protection_temp);
  EEPROM.get(45, T_pepHOLD);
  EEPROM.get(50, PWM_FREQ_KHZ);
  EEPROM.get(55, protection_in);
  EEPROM.get(60, protection_vdd);
  EEPROM.get(65, protection_id);

  EEPROM.get(70, band_mode);
  EEPROM.get(72, current_band);
  EEPROM.get(74, current_lpf);
  EEPROM.get(76, antenna2_status);

  EEPROM.get(150, ntc1_status);
  EEPROM.get(151, ntc2_status);
  EEPROM.get(152, dtemp1_status);
  EEPROM.get(153, dtemp2_status);
  EEPROM.get(154, vband_status);
  EEPROM.get(155, bcdband_status);
  EEPROM.get(156, rotband_status);
  EEPROM.get(157, band8_status);
  EEPROM.get(158, CATband_status);
  EEPROM.get(159, CATrig);
  EEPROM.get(160, CATbaud); // 2 byte
  EEPROM.get(162, CATaddress);

  EEPROM.get(200, b1);
  EEPROM.get(220, b2);
  EEPROM.get(240, b3);
  EEPROM.get(260, b4);
  EEPROM.get(280, b5);
  EEPROM.get(300, b6);
  EEPROM.get(320, b7);
  EEPROM.get(340, b8);
  EEPROM.get(360, b9);
  EEPROM.get(380, b10);
  EEPROM.get(400, b11);

  protection_swr = (float)protection_swr_mem / 10.0;
}

void saveEEPROM()
{
  EEPROM.put(500, CalibV_out);
  EEPROM.put(504, CalibV_in);
  EEPROM.put(508, measured_power_out);
  EEPROM.put(512, measured_power_in);
  EEPROM.put(516, calibration_ID);
  EEPROM.put(520, ANT.calibration);
  EEPROM.put(524, IN.calibration);

  EEPROM.put(21, ANT.maxGraphWatt);
  EEPROM.put(25, graph_maxTemp);
  EEPROM.put(30, protection_po);
  EEPROM.put(35, protection_swr_mem);
  EEPROM.put(40, protection_temp);
  EEPROM.put(45, T_pepHOLD);
  EEPROM.put(50, PWM_FREQ_KHZ);
  EEPROM.put(55, protection_in);
  EEPROM.put(60, protection_vdd);
  EEPROM.put(65, protection_id);

  EEPROM.put(70, band_mode);
  EEPROM.put(72, current_band);
  EEPROM.put(74, current_lpf);
  EEPROM.put(76, antenna2_status);

  EEPROM.put(150, ntc1_status);
  EEPROM.put(151, ntc2_status);
  EEPROM.put(152, dtemp1_status);
  EEPROM.put(153, dtemp2_status);
  EEPROM.put(154, vband_status);
  EEPROM.put(155, bcdband_status);
  EEPROM.put(156, rotband_status);
  EEPROM.put(157, band8_status);
  EEPROM.put(158, CATband_status);
  EEPROM.put(159, CATrig);
  EEPROM.put(160, CATbaud); // 2 byte
  EEPROM.put(162, CATaddress);

  EEPROM.put(200, b1);
  EEPROM.put(220, b2);
  EEPROM.put(240, b3);
  EEPROM.put(260, b4);
  EEPROM.put(280, b5);
  EEPROM.put(300, b6);
  EEPROM.put(320, b7);
  EEPROM.put(340, b8);
  EEPROM.put(360, b9);
  EEPROM.put(380, b10);
  EEPROM.put(400, b11);
}

/* === Enabling touch functions of Display === */
void LCDband_enable()
{
  if (touch_status != true)
  {
    myNex.writeStr("tsw bn1,1");
    myNex.writeStr("tsw bn2,1");
    myNex.writeStr("tsw bn3,1");
    myNex.writeStr("tsw bn4,1");
    myNex.writeStr("tsw bn5,1");
    myNex.writeStr("tsw bn6,1");
    myNex.writeStr("tsw bn7,1");
    myNex.writeStr("tsw bn8,1");
    touch_status = true;
  }
}

/* === Disabling touch functions of Display === */
void LCDband_disable()
{
  if (touch_status != false)
  {
    // myNex.writeStr("tsw 255,0");
    myNex.writeStr("tsw bn1,0");
    myNex.writeStr("tsw bn2,0");
    myNex.writeStr("tsw bn3,0");
    myNex.writeStr("tsw bn4,0");
    myNex.writeStr("tsw bn5,0");
    myNex.writeStr("tsw bn6,0");
    myNex.writeStr("tsw bn7,0");
    myNex.writeStr("tsw bn8,0");
    touch_status = false;
  }
}

// Read Analog NTC values
void read_ntc1()
{
  adc_ntc1 = analogRead(NTC1);
  res_ntc1 = (1023.00 / adc_ntc1) - 1.0;
  res_ntc1 = 10000.00 / res_ntc1;
  temp_ntc1 = res_ntc1 / 10000.00;
  temp_ntc1 = log(temp_ntc1);
  temp_ntc1 = temp_ntc1 / ntcbeta;
  temp_ntc1 += 1.0 / (25.0 + 273.15);
  temp_ntc1 = 1.0 / temp_ntc1;
  temp_ntc1 -= 273.15;
}

void read_ntc2()
{
  adc_ntc2 = analogRead(NTC2);
  res_ntc2 = (1023.00 / adc_ntc2) - 1;
  res_ntc2 = 10000.00 / res_ntc2;

  temp_ntc2 = res_ntc2 / 10000.00;
  temp_ntc2 = log(temp_ntc2);
  temp_ntc2 = temp_ntc2 / ntcbeta;
  temp_ntc2 += 1.0 / (25.0 + 273.15);
  temp_ntc2 = 1.0 / temp_ntc2;
  temp_ntc2 -= 273.15;
}

/* === Monitor current from protection board  === */
float I_now()
{
  float currentI = 0;
  currentI = analogRead(ID);
  currentI *= ref_ADC;
  if (display_page == 4)
  {
    return currentI;
  }
  else
  {
    currentI /= RL;
    currentI *= 13000;
    return currentI;
  }
}

/* ===  protection enable Function === */
void soft_protection()
{
  digitalWrite(RF_IN, LOW);
  digitalWrite(BIAS_ON, LOW);
  digitalWrite(ANT_RXTX, LOW);
  digitalWrite(PROT_OUT, HIGH);

  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
  PROT_OUT_status = HIGH;
}

/* === Volt meter Function === */

float V_now()
{
  float currentV = 0;
  currentV = analogRead(VCC);
  currentV = (currentV * ref_ADC);
  currentV /= ResV;
  return currentV;
}

/* === RF Power measurement Function === */

float ant_fwd_now()
{
  float ANTFWD = 0;
  ANTFWD = analogRead(ANT_FWD);
  // Serial.print("ANT FW ADC: ");
  // Serial.println(ANTFWD);
  ANTFWD = ((ANTFWD * ref_ADC) / ResP);
  if (ANTFWD > 0.1)
    ANTFWD += VFdiode;
  else
    ANTFWD = 0.0;
  // Serial.print("ANT FW V: ");
  // Serial.println(ANTFWD);
  return ANTFWD;
}

float ant_ref_now()
{
  float ANTREF = 0.0;
  ANTREF = analogRead(ANT_REF);
  ANTREF = ((ANTREF * ref_ADC) / ResP);
  if (ANTREF > 0.1)
    ANTREF += VFdiode;
  else
    ANTREF = 0.0;
  return ANTREF;
}

float lpf_fwd_now()
{
  float LPFFWD = 0.0;
  LPFFWD = analogRead(LPF_FWD);
  // Serial.print("LPF FWD ADC: ");
  // Serial.println(LPFFWD);
  LPFFWD = ((LPFFWD * ref_ADC) / ResP);
  if (LPFFWD > 0.1)
    LPFFWD += VFdiode;
  else
    LPFFWD = 0.0;
  // Serial.print("LPF FWD V: ");
  // Serial.println(LPFFWD);

  return LPFFWD;
}

float lpf_ref_now()
{
  float LPFREF = 0.0;
  LPFREF = analogRead(LPF_REF);
  LPFREF = ((LPFREF * ref_ADC) / ResP);
  if (LPFREF > 0.1)
    LPFREF += VFdiode;
  else
    LPFREF = 0.0;
  return LPFREF;
}

float in_fwd_now()
{
  float INFWD = 0.0;

  INFWD = analogRead(IN_FWD);
  INFWD = ((INFWD * ref_ADC) / ResP);
  if (INFWD > 0.1)
    INFWD += VFdiode;
  else
    INFWD = 0.0;
  return INFWD;
}

float in_ref_now()
{
  float INREF = 0.0;
  INREF = analogRead(IN_REF);
  INREF = ((INREF * ref_ADC) / ResP);
  if (INREF > 0.1)
    INREF += VFdiode;
  else
    INREF = 0.0;
  return INREF;
}

/* === Display Volt on Display === */
void display_volt()
{
  V = ((V_now() * 10) + 0.5); // Float x 10 and convert to int with rounding

  if ((millis() - lastVRequest) >= Volt_Refresh && lastV != V)
  {
    myNex.writeNum("x3.val", V);
    lastVRequest = millis();
    lastV = V;
  }
}

/* === SWR/Po calculation Function === */

void read_ANTpower()
{

  float Veff = V_now();
  float Iprot = I_now();

  ANT.rawFWD = ant_fwd_now();
  ANT.rawREF = ant_ref_now();

  // Serial.print("ANT.rawFWD: ");
  // Serial.println(ANT.rawFWD);
  // Serial.print("ANT.rawREF: ");
  // Serial.println(ANT.rawREF);

  // Calculate Forward power
  if (ANT.rawFWD > VFdiode + 0.1)
  { // only correct for diode voltage when more than zero
    ANT.powerFWD = (ANT.rawFWD * ANT.rawFWD) / ANT.calibration;
    // Serial.println("ANT.rawFWD > VFdiode");
  }
  else
    ANT.powerFWD = 0;

  // Calculate Reflected power
  if (ANT.rawREF > VFdiode + 0.1)
  {
    ANT.powerREF = (ANT.rawREF * ANT.rawREF) / ANT.calibration;
    // Serial.println("ANT.rawFWD > VFdiode");
  }
  else
    ANT.powerREF = 0;

  if (ANT.rawREF != 0.0)
  {
    ANT.SWR = abs((ANT.rawFWD + ANT.rawREF) / (ANT.rawFWD - ANT.rawREF));
  }
  else
    ANT.SWR = 1.0;

  // Serial.print("ANT.powerFWD: ");
  // Serial.println(ANT.powerFWD);
  // Serial.print("ANT.powerREF: ");
  // Serial.println(ANT.powerREF);
  // Serial.print("ANT SWR: ");
  // Serial.println(ANT.SWR);

  // Serial.print("ANT SWR: ");
  // Serial.println(ANT.SWR);
  // Serial.print("protection_swr: ");
  // Serial.println(protection_swr);
  // Serial.print("protection_swr_mem: ");
  // Serial.println(protection_swr_mem);

  /* === Check if OUTPUT is higher than set value  === */
  if (ANT.powerFWD >= protection_po)
  {
    soft_protection();
    alarm_code = 6;
  }

  /* === Check if SWR is higher than set value  === */
  if (ANT.SWR >= protection_swr)
  {
    soft_protection();
    alarm_code = 7;
  }

  /* === Check current vs Output Power to see if LPF mismatch  === */
  if (Iprot >= 10.0 && ANT.powerFWD <= 100)
  {
    soft_protection();
    alarm_code = 8;
  }

  // hold peak
  if (ANT.powerFWD >= ANT.PEAKpowerFWD)
  {
    ANT.lastPEP = millis();
    ANT.PEAKpowerFWD = ANT.powerFWD;
  }

  if (millis() > (ANT.lastPEP + T_pepHOLD))
    ANT.PEAKpowerFWD = ANT.powerFWD; // clear the peak after hold time

  // Efficiency calculation
  if (Veff != 0 && Iprot != 0)
    Eff = 100 * (ANT.powerFWD / (Veff * Iprot));
}

void read_LPFpower()
{

  LPF.rawFWD = lpf_fwd_now();
  LPF.rawREF = lpf_ref_now();

  // Calculate Forward power
  if (LPF.rawFWD > VFdiode + 0.1) // only correct for diode voltage when more than zero
  {
    LPF.powerFWD = (LPF.rawFWD * LPF.rawFWD) / ANT.calibration;
  }
  else
    LPF.powerFWD = 0;

  // Calculate Reflected power
  if (LPF.rawREF > VFdiode + 0.1) // only correct for diode voltage when more than zero
  {
    LPF.powerREF = (LPF.powerREF * LPF.powerREF) / ANT.calibration;
  }
  else
    LPF.powerREF = 0;

  if (LPF.rawREF != 0.0)
  {
    LPF.SWR = abs((LPF.rawFWD + LPF.rawREF) / (LPF.rawFWD - LPF.rawREF));
  }
  else
    LPF.SWR = 1.0;

  /* === Check if OUTPUT is higher than set value  === */
  if (LPF.powerFWD >= protection_po)
  {
    // Serial.println("Alarm 9 Prot PO");
    // Serial.println(ANT.calibration);
    // Serial.println(LPF.powerFWD);
    // Serial.println(protection_po);

    soft_protection();
    alarm_code = 9;
  }

  /* === Check if SWR is higher than set value  === */
  if (LPF.SWR >= protection_swr)
  {
    soft_protection();
    alarm_code = 10;
  }

  // hold peak
  if (LPF.powerFWD >= LPF.PEAKpowerFWD)
  {
    LPF.lastPEP = millis();
    LPF.PEAKpowerFWD = LPF.powerFWD;
  }

  if (millis() > (LPF.lastPEP + T_pepHOLD))
    LPF.PEAKpowerFWD = LPF.powerFWD; // clear the peak after hold time
}

void read_INpower()
{

  IN.rawFWD = in_fwd_now();
  IN.rawREF = in_ref_now();

  // Calculate Forward power
  if (IN.rawFWD > VFdiode + 0.1) // only correct for diode voltage when more than zero
  {
    IN.powerFWD = (IN.rawFWD * IN.rawFWD) / IN.calibration;
  }
  else
    IN.powerFWD = 0;

  // Calculate Reflected power
  if (IN.rawREF > VFdiode + 0.1)
  {
    IN.powerREF = (IN.rawREF * IN.rawREF) / IN.calibration;
  }
  else
    IN.powerREF = 0;

  // Calculate SWR

  if (IN.rawREF != 0.0)
  {
    IN.SWR = abs((IN.rawFWD + IN.rawREF) / (IN.rawFWD - IN.rawREF));
  }
  else
    IN.SWR = 1.0;

  /* === Check if OUTPUT is higher than set value  === */

  if (IN.powerFWD >= protection_in)
  {
    soft_protection();
    alarm_code = 11;
  }
  // hold peak
  if (IN.powerFWD >= IN.PEAKpowerFWD)
  {
    IN.lastPEP = millis();
    IN.PEAKpowerFWD = IN.powerFWD;
  }

  if (millis() > (IN.lastPEP + T_pepHOLD))
    IN.PEAKpowerFWD = IN.powerFWD; // clear the peak after hold time
}

/* === SWR/Po display Function === */
// Refresh screen every N miliseconds (N = PdelayInMillis)
void update_display()
{
  myNex.writeNum("n0.val", ANT.PEAKpowerFWD);
  myNex.writeNum("n1.val", ANT.powerFWD);
  myNex.writeNum("n2.val", IN.powerFWD);
  myNex.writeNum("n3.val", ANT.powerFWD);
  myNex.writeNum("n4.val", ANT.powerREF);
  myNex.writeNum("n5.val", LPF.powerFWD);
  myNex.writeNum("n6.val", LPF.powerREF);
  myNex.writeNum("n7.val", Eff);

  myNex.writeNum("x0.val", ANT.swr_display);
  myNex.writeNum("x1.val", ANT.swr_display);
  myNex.writeNum("x2.val", IN.swr_display);

  myNex.writeNum("j0.val", ANT.graph_Watt);
  myNex.writeNum("j1.val", ANT.graphSWR);
  myNex.writeNum("j2.val", IN.graph_Watt);
  myNex.writeNum("j3.val", IN.graphSWR);

  ANT.powerFWD = 0;
  ANT.powerREF = 0;
  ANT.PEAKpowerFWD = 0;
  ANT.graph_Watt = 0;
  ANT.swr_display = 10;
  ANT.graphSWR = 0;

  LPF.powerFWD = 0;
  LPF.powerREF = 0;
  LPF.graph_Watt = 0;
  LPF.swr_display = 10;
  LPF.graphSWR = 0;

  IN.powerFWD = 0;
  IN.PEAKpowerFWD = 0;
  IN.graph_Watt = 0;
  IN.swr_display = 10;
  IN.graphSWR = 0;

  Eff = 0;
}

void display_power(bool active)
{

  if (active)
  {
    if ((millis() - last_Power_Refresh) >= Power_Refresh)
    {
      ANT.swr_display = (ANT.SWR * 10) + 0.5; //// Float x 10 and convert to int with rounding
      if (ANT.swr_display < 10)
        ANT.swr_display = 10; // SWR cannot be lower than 1.0

      IN.swr_display = (IN.SWR * 10) + 0.5; //// Float x 10 and convert to int with rounding
      if (IN.swr_display < 10)
        IN.swr_display = 10; // SWR cannot be lower than 1.0

      float graph_limit_watt = (ANT.maxGraphWatt / 100.00);
      ANT.graph_Watt = (ANT.PEAKpowerFWD / graph_limit_watt);

      float graph_limit_swr = ((ANT.maxgraphSWR - 1) / 100.00);
      float swr_forgraph = ANT.SWR - 1;
      ANT.graphSWR = swr_forgraph / graph_limit_swr;

      IN.graph_Watt = IN.PEAKpowerFWD;

      /// float graph_limit_swr = ((graph_maxSwr - 1) / 100.00);
      float in_swr_forgraph = IN.SWR - 1;
      IN.graphSWR = in_swr_forgraph;

      update_display();

      last_Power_Refresh = millis();
    }
  }
  else
  {
    if ((millis() - last_Power_Refresh) >= Power_Refresh)
    {
      ANT.powerFWD = 0;
      ANT.PEAKpowerFWD = 0;
      ANT.graph_Watt = 0;
      ANT.swr_display = 10;
      ANT.graphSWR = 0;

      LPF.powerFWD = 0;
      LPF.graph_Watt = 0;
      LPF.swr_display = 10;
      LPF.graphSWR = 0;

      IN.powerFWD = 0;
      IN.PEAKpowerFWD = 0;
      IN.graph_Watt = 0;
      IN.swr_display = 10;
      IN.graphSWR = 0;

      Eff = 0;

      update_display();

      last_Power_Refresh = millis();
    }
  }
}

/* =========== Send band info to Display ====================== */

void display_band(int Nband)
{
  if (last_display_mband != Nband)
  {

    switch (Nband)
    {
    case 1:
      myNex.writeNum("n12.val", b1.meterband);
      break;
    case 2:
      myNex.writeNum("n12.val", b2.meterband);
      break;
    case 3:
      myNex.writeNum("n12.val", b3.meterband);
      break;
    case 4:
      myNex.writeNum("n12.val", b4.meterband);
      break;
    case 5:
      myNex.writeNum("n12.val", b5.meterband);
      break;
    case 6:
      myNex.writeNum("n12.val", b6.meterband);
      break;
    case 7:
      myNex.writeNum("n12.val", b7.meterband);
      break;
    case 8:
      myNex.writeNum("n12.val", b8.meterband);
      break;
    case 9:
      myNex.writeNum("n12.val", b9.meterband);
      break;
    case 10:
      myNex.writeNum("n12.val", b10.meterband);
      break;
    case 11:
      myNex.writeNum("n12.val", b11.meterband);
      break;
    }
  }
  last_display_mband = Nband;
}

// Update LPF button on display
void display_lpf(int Nband)
{
  if (last_display_lpf != Nband)
  {
    switch (Nband)
    {
    case 1:
      myNex.writeNum("bn1.val", 1);
      myNex.writeNum("bn2.val", 0);
      myNex.writeNum("bn3.val", 0);
      myNex.writeNum("bn4.val", 0);
      myNex.writeNum("bn5.val", 0);
      myNex.writeNum("bn6.val", 0);
      myNex.writeNum("bn7.val", 0);
      myNex.writeNum("bn8.val", 0);
      break;

    case 2:
      myNex.writeNum("bn1.val", 0);
      myNex.writeNum("bn2.val", 1);
      myNex.writeNum("bn3.val", 0);
      myNex.writeNum("bn4.val", 0);
      myNex.writeNum("bn5.val", 0);
      myNex.writeNum("bn6.val", 0);
      myNex.writeNum("bn7.val", 0);
      myNex.writeNum("bn8.val", 0);
      break;

    case 3:
      myNex.writeNum("bn1.val", 0);
      myNex.writeNum("bn2.val", 0);
      myNex.writeNum("bn3.val", 1);
      myNex.writeNum("bn4.val", 0);
      myNex.writeNum("bn5.val", 0);
      myNex.writeNum("bn6.val", 0);
      myNex.writeNum("bn7.val", 0);
      myNex.writeNum("bn8.val", 0);
      break;

    case 4:
      myNex.writeNum("bn1.val", 0);
      myNex.writeNum("bn2.val", 0);
      myNex.writeNum("bn3.val", 0);
      myNex.writeNum("bn4.val", 1);
      myNex.writeNum("bn5.val", 0);
      myNex.writeNum("bn6.val", 0);
      myNex.writeNum("bn7.val", 0);
      myNex.writeNum("bn8.val", 0);
      break;

    case 5:
      myNex.writeNum("bn1.val", 0);
      myNex.writeNum("bn2.val", 0);
      myNex.writeNum("bn3.val", 0);
      myNex.writeNum("bn4.val", 0);
      myNex.writeNum("bn5.val", 1);
      myNex.writeNum("bn6.val", 0);
      myNex.writeNum("bn7.val", 0);
      myNex.writeNum("bn8.val", 0);
      break;

    case 6:
      myNex.writeNum("bn1.val", 0);
      myNex.writeNum("bn2.val", 0);
      myNex.writeNum("bn3.val", 0);
      myNex.writeNum("bn4.val", 0);
      myNex.writeNum("bn5.val", 0);
      myNex.writeNum("bn6.val", 1);
      myNex.writeNum("bn7.val", 0);
      myNex.writeNum("bn8.val", 0);
      break;

    case 7:
      myNex.writeNum("bn1.val", 0);
      myNex.writeNum("bn2.val", 0);
      myNex.writeNum("bn3.val", 0);
      myNex.writeNum("bn4.val", 0);
      myNex.writeNum("bn5.val", 0);
      myNex.writeNum("bn6.val", 0);
      myNex.writeNum("bn7.val", 1);
      myNex.writeNum("bn8.val", 0);
      break;

    case 8:
      myNex.writeNum("bn1.val", 0);
      myNex.writeNum("bn2.val", 0);
      myNex.writeNum("bn3.val", 0);
      myNex.writeNum("bn4.val", 0);
      myNex.writeNum("bn5.val", 0);
      myNex.writeNum("bn6.val", 0);
      myNex.writeNum("bn7.val", 0);
      myNex.writeNum("bn8.val", 1);
      break;
    }
    last_display_lpf = Nband;
  }
}

/* =========== LPF Relay control ====================== */

// Select LPF

void set_lpf(int lpf_relay)
{

  MCP.write8(LOW);
  MCP.write1(lpf_relay - 1, HIGH);
}

void lpf(int Nrelay)
{
  if (last_lpf_relay != Nrelay)
  {
    EEPROM.put(72, current_band); // Update band info to Memory when LPF chnages
    EEPROM.put(74, Nrelay);       // Update band info to Memory when LPF chnages

    set_lpf(Nrelay);
    // Serial.print("Nrelay:");
    // Serial.println(Nrelay);
    // Serial.print("Current Band:");
    // Serial.println(current_band);

    current_lpf = Nrelay;
    last_lpf_relay = Nrelay;
  }
}

/* === Monitor and show PTT status on display === */
void display_ptt()
{
  if (PTT_status == HIGH)
  {
    myNex.writeNum("tx.val", 1);
    LCDband_disable();
  }
  else
  {
    myNex.writeNum("tx.val", 0);
    LCDband_enable();
  }
}

// Display ALARM Icon according to alarm code
void display_error()
{
  if (last_alarm_code != alarm_code)
  {
    switch (alarm_code)
    {
    case 0:

      myNex.writeNum("home.ER.val", 0);
      myNex.writeNum("home.er1.val", 0);
      myNex.writeNum("home.er2.val", 0);
      myNex.writeNum("home.er3.val", 0);
      myNex.writeNum("home.er4.val", 0);
      myNex.writeNum("home.er5.val", 0);
      myNex.writeNum("home.er6.val", 0);
      myNex.writeNum("home.er7.val", 0);
      myNex.writeNum("home.er8.val", 0);
      myNex.writeStr("home.er.txt", " ");
      break;
    case 1:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er8.val", 1);
      myNex.writeStr("home.er.txt", "External Protection Detected (H), correct fault then clear alarm from menu");

      break;
    case 2:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er1.val", 1);
      myNex.writeStr("home.er.txt", "ANT SWR high (H), correct fault then clear alarm from menu");

      break;
    case 3:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er7.val", 1);
      myNex.writeStr("home.er.txt", "High INPUT power (H), correct fault then clear alarm from menu");

      break;
    case 4:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er2.val", 1);
      myNex.writeStr("home.er.txt", "LPF SWR High (H), correct fault then clear alarm from menu");

      break;
    case 5:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er3.val", 1);
      myNex.writeStr("home.er.txt", "High Temperature (S), correct fault then clear alarm from menu");

      break;
    case 6:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er6.val", 1);
      myNex.writeStr("home.er.txt", "High OUTPUT power (S), correct fault then clear alarm from menu");

      break;
    case 7:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er1.val", 1);
      myNex.writeStr("home.er.txt", "High SWR (S), correct fault then clear alarm from menu");

      break;
    case 8:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er2.val", 1);
      myNex.writeStr("home.er.txt", "LPF mismatch (S), correct fault then clear alarm from menu");

      break;
    case 9:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er6.val", 1);
      myNex.writeStr("home.er.txt", "LPF OUTPUT high (S), correct fault then clear alarm from menu");

      break;
    case 10:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er2.val", 1);
      myNex.writeStr("home.er.txt", "LPF SWR high (S), correct fault then clear alarm from menu");

      break;
    case 11:
      myNex.writeNum("home.ER.val", 1);
      myNex.writeNum("home.er7.val", 1);
      myNex.writeStr("home.er.txt", "High INPUT power (S), correct fault then clear alarm from menu");

      break;
    }
    last_alarm_code = alarm_code;
  }
}

// Display ID/current
void display_ID()
{
  I = ((I_now() * calibration_ID * 10) + 0.5); // Float x 10 and convert to int with rounding

  if ((millis() - last_ID_Refresh) >= ID_Refresh && lastI != I)
  {
    myNex.writeNum("x4.val", I);
    last_ID_Refresh = millis();
    lastI = I;
  }
}

/* === Set FAN speed according to temp and display === */
void fanspeed()
{
  if (temp_digi1 < 25.0)
    PWM = 20;
  else if (temp_digi1 > 25.0 && temp_digi1 < 30.0)
    PWM = 40;
  else if (temp_digi1 > 30.0 && temp_digi1 < 40.0)
    PWM = 60;
  else if (temp_digi1 >= 40.0 && temp_digi1 < 45.0)
    PWM = 70;
  else if (temp_digi1 >= 45.0 && temp_digi1 < 50.0)
    PWM = 80;
  else if (temp_digi1 >= 50.0)
    PWM = 100;

  // only update data when PWM value change
  if (PWM != last_PWM)
  {
    // setPwmDuty(PWM);
    last_PWM = PWM;
    int fan_speed = PWM;
    myNex.writeNum("f1.val", fan_speed);
    analogWrite(FAN_1, (PWM * 255) / 100);
    analogWrite(FAN_2, (PWM * 255) / 100);
  }
}

/* === Tempareturn Monitor  === */
void read_temp()
{
  if (millis() - last_Temp_Refresh >= Temp_Refresh)
  {
    sensors.requestTemperatures();
    if (dtemp1_status == 1)
    {
      temp_digi1 = sensors.getTempCByIndex(0);
      if (dtemp2_status)
        temp_digi2 = sensors.getTempCByIndex(1);
      else
        temp_digi2 = 0.0;
    }
    else
    {
      temp_digi1 = 0.0;
      temp_digi2 = 0.0;
    }

    if (ntc1_status == 1)
      read_ntc1();
    else
      temp_ntc1 = 0.0;
    if (ntc2_status)
      read_ntc2();
    else
      temp_ntc2 = 0.0;
    last_Temp_Refresh = millis();
  }

  if (temp_digi1 >= float(protection_temp) || temp_digi2 >= float(protection_temp) || temp_ntc1 >= float(protection_temp) || temp_ntc2 >= float(protection_temp))
  {
    soft_protection();
    alarm_code = 5;
  }

  if ((millis() - last_Display_Refresh) >= Display_Refresh)
  {
    float graph_limit = (graph_maxTemp / 100.00);

    int Tdisplay1 = (temp_ntc1 * 10) + 0.5; // Float x 10 and convert to int with rounding
    graph_Temp1 = (temp_ntc1 / graph_limit);

    int Tdisplay2 = (temp_ntc2 * 10) + 0.5; // Float x 10 and convert to int with rounding
    graph_Temp2 = (temp_ntc2 / graph_limit);

    int Tdisplay3 = (temp_digi1 * 10) + 0.5; // Float x 10 and convert to int with rounding
    graph_Temp3 = (temp_digi1 / graph_limit);

    int Tdisplay4 = (temp_digi2 * 10) + 0.5; // Float x 10 and convert to int with rounding
    graph_Temp4 = (temp_digi2 / graph_limit);

    if (display_page == 2)
    {
      myNex.writeNum("tn1.val", Tdisplay1);  // n8-9-10-11
      myNex.writeNum("j4.val", graph_Temp1); // j4-7
      myNex.writeNum("tn2.val", Tdisplay2);  // n8-9-10-11
      myNex.writeNum("j5.val", graph_Temp2); // j4-7
      myNex.writeNum("tn3.val", Tdisplay3);  // n8-9-10-11
      myNex.writeNum("j6.val", graph_Temp3);
      myNex.writeNum("tn4.val", Tdisplay4); // n8-9-10-11
      myNex.writeNum("j7.val", graph_Temp4);
    }
    else if (display_page == 4)
    {
      // show NTC output in milivolt
      myNex.writeNum("dn0.val", adc_ntc1 * ref_ADC * 1000);
      myNex.writeNum("dn1.val", adc_ntc2 * ref_ADC * 1000);

      // display Digital sensor output in degree C
      myNex.writeNum("dx0.val", Tdisplay3);
      myNex.writeNum("dx1.val", Tdisplay4);
    }

    last_Display_Refresh = millis();

    last_temp_ntc1 = temp_ntc1;
    last_temp_ntc2 = temp_ntc2;
    last_temp_digi1 = temp_digi1;
    last_temp_digi2 = temp_digi2;
  }
  fanspeed();
}

/* =========== BAND SELECTION ========== */

void CAT_ReqQRG()
{

  Serial2.write(reqQRG, sizeof(reqQRG));
}

int cat2band()
{
  if (cat_MHz >= 1 && cat_MHz < 2)
  {
    // 160m
    return 1;
  }
  else if (cat_MHz >= 2 && cat_MHz < 4)
  {
    // 80m
    return 2;
  }
  else if (cat_MHz >= 4 && cat_MHz < 6)
  {
    // 60m
    return 3;
  }
  else if (cat_MHz >= 6 && cat_MHz < 8)
  {
    // 40m
    return 3;
  }
  else if (cat_MHz >= 8 && cat_MHz < 12)
  {
    // 30m
    return 4;
  }
  else if (cat_MHz >= 12 && cat_MHz < 16)
  {
    // 20m
    return 5;
  }
  else if (cat_MHz >= 16 && cat_MHz < 19)
  {
    // 17m
    return 6;
  }
  else if (cat_MHz >= 19 && cat_MHz < 22)
  {
    // 15m
    return 7;
  }
  else if (cat_MHz >= 22 && cat_MHz < 26)
  {
    // 12m
    return 8;
  }
  else if (cat_MHz >= 26 && cat_MHz < 30)
  {
    // 10m
    return 9;
  }
  else if (cat_MHz >= 30 && cat_MHz < 60)
  {
    // 6m
    return 10;
  }
  else if (cat_MHz >= 60 && cat_MHz < 76)
  {
    // 4m
    return 11;
  }
  return 0;
}

void band_selection()
{
  if (PTT_status != HIGH)
  {
    if (band_rotary() < 50 || rotband_status != 1)
    {
      if (band_mode == 1)
      { // TOUCH
        LCDband_enable();
        current_band = lpf2band(lpf_bank);
        if (current_band != last_band)
        {
          lpf(lpf_bank);
          last_band = current_band;
        }
      }

      else if (band_mode == 2 && bcdband_status)
      { // BCD
        // Serial.println("BCD BAND:");
        LCDband_disable();
        current_band = bcd2band(band_bcd());
        if (current_band != last_band)
        {

          lpf_bank = band2lpf(current_band);
          lpf(lpf_bank);
          last_band = current_band;
        }
      }
      else if (band_mode == 3 && vband_status)
      { // BAND V
        LCDband_disable();
        current_band = bandV2band(band_volt());
        if (current_band != last_band)
        {

          lpf_bank = band2lpf(current_band);
          lpf(lpf_bank);

          last_band = current_band;
        }
      }

      else if (band_mode == 4 && CATband_status)
      { // BAND CAT

        LCDband_disable();
        if (cat_MHz == 0)
        {
          CAT_ReqQRG();
          // newData = true;
        }

        current_band = cat2band();
        if (current_band != last_band)
        {
          lpf_bank = band2lpf(current_band);
          lpf(lpf_bank);

          last_band = current_band;
        }
      }
    }
    else
    {
      // ROTARY
      LCDband_disable();

      current_band = rotary2band(band_rotary());
      if (current_band != last_band)
      {

        lpf_bank = band2lpf(current_band);
        lpf(lpf_bank);
        myNex.writeNum("bt0.val", 0);
        myNex.writeNum("bt1.val", 1);
        myNex.writeNum("bt2.val", 0);
        myNex.writeNum("bt3.val", 0);
        myNex.writeNum("bt4.val", 0);
        last_band = current_band;
      }
    }
  }
}

/* === Trigger from Nextion Display  === */
void trigger1() // x01
{
  band_mode = 1;
  lpf_bank = 1;
  myNex.writeNum("bn1.val", 1);
  myNex.writeNum("bn2.val", 0);
  myNex.writeNum("bn3.val", 0);
  myNex.writeNum("bn4.val", 0);
  myNex.writeNum("bn5.val", 0);
  myNex.writeNum("bn6.val", 0);
  myNex.writeNum("bn7.val", 0);
  myNex.writeNum("bn8.val", 0);
}

void trigger2() // x02
{
  band_mode = 1;
  lpf_bank = 2;
  myNex.writeNum("bn1.val", 0);
  myNex.writeNum("bn2.val", 1);
  myNex.writeNum("bn3.val", 0);
  myNex.writeNum("bn4.val", 0);
  myNex.writeNum("bn5.val", 0);
  myNex.writeNum("bn6.val", 0);
  myNex.writeNum("bn7.val", 0);
  myNex.writeNum("bn8.val", 0);
}

void trigger3() // x03
{
  band_mode = 1;
  lpf_bank = 3;
  myNex.writeNum("bn1.val", 0);
  myNex.writeNum("bn2.val", 0);
  myNex.writeNum("bn3.val", 1);
  myNex.writeNum("bn4.val", 0);
  myNex.writeNum("bn5.val", 0);
  myNex.writeNum("bn6.val", 0);
  myNex.writeNum("bn7.val", 0);
  myNex.writeNum("bn8.val", 0);
}

void trigger4() // x04
{
  band_mode = 1;
  lpf_bank = 4;
  myNex.writeNum("bn1.val", 0);
  myNex.writeNum("bn2.val", 0);
  myNex.writeNum("bn3.val", 0);
  myNex.writeNum("bn4.val", 1);
  myNex.writeNum("bn5.val", 0);
  myNex.writeNum("bn6.val", 0);
  myNex.writeNum("bn7.val", 0);
  myNex.writeNum("bn8.val", 0);
}

void trigger5() // x05
{
  band_mode = 1;
  lpf_bank = 5;
  myNex.writeNum("bn1.val", 0);
  myNex.writeNum("bn2.val", 0);
  myNex.writeNum("bn3.val", 0);
  myNex.writeNum("bn4.val", 0);
  myNex.writeNum("bn5.val", 1);
  myNex.writeNum("bn6.val", 0);
  myNex.writeNum("bn7.val", 0);
  myNex.writeNum("bn8.val", 0);
}

void trigger6() // x06
{
  band_mode = 1;
  lpf_bank = 6;
  myNex.writeNum("bn1.val", 0);
  myNex.writeNum("bn2.val", 0);
  myNex.writeNum("bn3.val", 0);
  myNex.writeNum("bn4.val", 0);
  myNex.writeNum("bn5.val", 0);
  myNex.writeNum("bn6.val", 1);
  myNex.writeNum("bn7.val", 0);
  myNex.writeNum("bn8.val", 0);
}

void trigger7() // x07
{
  band_mode = 1;
  lpf_bank = 7;
  myNex.writeNum("bn1.val", 0);
  myNex.writeNum("bn2.val", 0);
  myNex.writeNum("bn3.val", 0);
  myNex.writeNum("bn4.val", 0);
  myNex.writeNum("bn5.val", 0);
  myNex.writeNum("bn6.val", 0);
  myNex.writeNum("bn7.val", 1);
  myNex.writeNum("bn8.val", 0);
}

void trigger8() // x08
{
  band_mode = 1;
  lpf_bank = 8;
  myNex.writeNum("bn1.val", 0);
  myNex.writeNum("bn2.val", 0);
  myNex.writeNum("bn3.val", 0);
  myNex.writeNum("bn4.val", 0);
  myNex.writeNum("bn5.val", 0);
  myNex.writeNum("bn6.val", 0);
  myNex.writeNum("bn7.val", 0);
  myNex.writeNum("bn8.val", 1);
}

void set_ant()
{
  // Check last antenna ststus with current value
  if (antenna2_status != last_antenna2)
  {
    if (antenna2_status == HIGH)
    {
      digitalWrite(ANT2, HIGH);
      myNex.writeStr("t23.txt", "B");
    }
    else
    {
      digitalWrite(ANT2, LOW);
      myNex.writeStr("t23.txt", "A");
    }
    last_antenna2 = antenna2_status;
  }
}

// Antenna2 switch
void trigger30() // x1E
{

  if (antenna2_status == LOW)
  {
    digitalWrite(ANT2, HIGH);
    myNex.writeStr("t23.txt", "B");
    antenna2_status = HIGH;
  }
  else
  {
    digitalWrite(ANT2, LOW);
    myNex.writeStr("t23.txt", "A");
    antenna2_status = LOW;
  }
  EEPROM.put(76, antenna2_status);
}

void trigger9() // Loading variables data to Setting page
{               // x09
  TX_Enable = false;

  myNex.writeNum("st0.val", ANT.maxGraphWatt);
  myNex.writeNum("st1.val", T_pepHOLD);
  myNex.writeNum("st7.val", graph_maxTemp);
  myNex.writeNum("st8.val", PWM_FREQ_KHZ);

  // Checkboxes
  myNex.writeNum("sc0.val", ntc1_status);
  myNex.writeNum("sc1.val", ntc2_status);
  myNex.writeNum("sc2.val", dtemp1_status);
  myNex.writeNum("sc7.val", band8_status);

  if (dtemp1_status == 1)
    myNex.writeNum("sc3.val", dtemp2_status);

  myNex.writeNum("sc4.val", vband_status);
  myNex.writeNum("sc5.val", bcdband_status);
  myNex.writeNum("sc6.val", rotband_status);
  myNex.writeNum("sc8.val", CATband_status);

  myNex.writeNum("st2.val", protection_temp);
  myNex.writeNum("st3.val", protection_po);
  myNex.writeNum("st4.val", protection_vdd);
  myNex.writeNum("st5.val", protection_id);
  myNex.writeNum("stx0.val", protection_swr_mem);
  myNex.writeNum("st6.val", protection_in);

  display_page = 5;
}

void trigger10() // updating variables data from Setting page and EEPROM
{                // x0A

  ANT.maxGraphWatt = myNex.readNumber("st0.val");
  T_pepHOLD = myNex.readNumber("st1.val");
  graph_maxTemp = myNex.readNumber("st7.val");
  PWM_FREQ_KHZ = myNex.readNumber("st8.val");

  ntc1_status = myNex.readNumber("sc0.val");
  ntc2_status = myNex.readNumber("sc1.val");
  dtemp1_status = myNex.readNumber("sc2.val");

  if (dtemp1_status == 1)
    dtemp2_status = myNex.readNumber("sc3.val");
  else
    dtemp2_status = 0;

  vband_status = myNex.readNumber("sc4.val");
  bcdband_status = myNex.readNumber("sc5.val");
  rotband_status = myNex.readNumber("sc6.val");
  band8_status = myNex.readNumber("sc7.val");
  CATband_status = myNex.readNumber("sc8.val");

  protection_temp = myNex.readNumber("st2.val");
  protection_po = myNex.readNumber("st3.val");
  protection_vdd = myNex.readNumber("st4.val");
  protection_id = myNex.readNumber("st5.val");
  protection_swr_mem = myNex.readNumber("stx0.val");
  protection_in = myNex.readNumber("st6.val");
  protection_swr = (float)protection_swr_mem / 10.0;
  saveEEPROM();
  myNex.writeNum("settings.t9.pco", 2016);
  myNex.writeStr("settings.t9.txt", "SETTINGS SAVED");
  display_page = 5;
}

void trigger11()
{ // x0B HOME SCREEN initialize
  TX_Enable = true;
  myNex.writeNum("home.n8.val", ANT.maxGraphWatt);
  if (band8_status == 0)
    myNex.writeStr("vis bn8,0");
  else
    myNex.writeStr("vis bn8,1");

  if (rotband_status == 0)
    myNex.writeStr("tsw bt1,0");
  else
    myNex.writeStr("tsw bt1,1");

  if (vband_status == 0)
    myNex.writeStr("tsw bt0,0");
  else
    myNex.writeStr("tsw bt0,1");

  if (bcdband_status == 0)
    myNex.writeStr("tsw bt2,0");
  else
    myNex.writeStr("tsw bt2,1");

  if (CATband_status == 0)
  {
    myNex.writeStr("tsw bt4,0");
    myNex.writeNum("x5.pco", 0);
    myNex.writeNum("x6.pco", 0);
  }
  else
  {
    myNex.writeStr("tsw bt4,1");
    myNex.writeNum("x5.pco", 2016);
    myNex.writeNum("x6.pco", 2016);
    CAT_ReqQRG();
  }

  touch_status = true;
  display_page = 2;
  // Serial.print("Band Mode:");
  // Serial.println(band_mode);
  // Serial.print("BCD status:");
  // Serial.println(bcdband_status);
}

void trigger12()
{ // x0C DIAGNOS PAGE
  TX_Enable = true;
  display_page = 4;
}

// Update Band Mode buttons
void display_band_mode()
{
  if (band_mode == 1)
  {
    myNex.writeNum("bt0.val", 0);
    myNex.writeNum("bt1.val", 0);
    myNex.writeNum("bt2.val", 0);
    myNex.writeNum("bt3.val", 1);
    myNex.writeNum("bt4.val", 0);
  }
  else if (band_mode == 2)
  {
    myNex.writeNum("bt0.val", 0);
    myNex.writeNum("bt1.val", 0);
    myNex.writeNum("bt2.val", 1);
    myNex.writeNum("bt3.val", 0);
    myNex.writeNum("bt4.val", 0);
  }
  else if (band_mode == 3)
  {
    myNex.writeNum("bt0.val", 1);
    myNex.writeNum("bt1.val", 0);
    myNex.writeNum("bt2.val", 0);
    myNex.writeNum("bt3.val", 0);
    myNex.writeNum("bt4.val", 0);
  }
  else if (band_mode == 4)
  {
    myNex.writeNum("bt0.val", 0);
    myNex.writeNum("bt1.val", 0);
    myNex.writeNum("bt2.val", 0);
    myNex.writeNum("bt3.val", 0);
    myNex.writeNum("bt4.val", 1);
  }
}

// APPLY button press for ROTARY data in Band Calibration page
void trigger14()
{ // x0E
  b1.rotary = myNex.readNumber("n17.val");
  b2.rotary = myNex.readNumber("n18.val");
  b3.rotary = myNex.readNumber("n19.val");
  b4.rotary = myNex.readNumber("n20.val");
  b5.rotary = myNex.readNumber("n21.val");
  b6.rotary = myNex.readNumber("n22.val");
  b7.rotary = myNex.readNumber("n23.val");
  b8.rotary = myNex.readNumber("n24.val");
  b9.rotary = myNex.readNumber("n0.val");
  b10.rotary = myNex.readNumber("n27.val");
  b11.rotary = myNex.readNumber("n30.val");
  saveEEPROM();
}

// APPLY button press for Band Volt data in Band Calibration page
void trigger15()
{ // x0F
  b1.volt = myNex.readNumber("n1.val");
  b2.volt = myNex.readNumber("n2.val");
  b3.volt = myNex.readNumber("n3.val");
  b4.volt = myNex.readNumber("n4.val");
  b5.volt = myNex.readNumber("n5.val");
  b6.volt = myNex.readNumber("n6.val");
  b7.volt = myNex.readNumber("n7.val");
  b8.volt = myNex.readNumber("n8.val");
  b9.volt = myNex.readNumber("n25.val");
  b10.volt = myNex.readNumber("n28.val");
  b11.volt = myNex.readNumber("n31.val");
  saveEEPROM();
}

// APPLY button press for BCD data in Band Calibration page
void trigger16()
{ // x10
  b1.bcd = myNex.readNumber("n9.val");
  b2.bcd = myNex.readNumber("n10.val");
  b3.bcd = myNex.readNumber("n11.val");
  b4.bcd = myNex.readNumber("n12.val");
  b5.bcd = myNex.readNumber("n13.val");
  b6.bcd = myNex.readNumber("n14.val");
  b7.bcd = myNex.readNumber("n15.val");
  b8.bcd = myNex.readNumber("n16.val");
  b9.bcd = myNex.readNumber("n26.val");
  b10.bcd = myNex.readNumber("n29.val");
  b11.bcd = myNex.readNumber("n32.val");
  saveEEPROM();
}

// APPLY button press for LPF mapping data in Band Calibration page
void trigger17()
{ // x10
  b1.lpf = myNex.readNumber("n43.val");
  b2.lpf = myNex.readNumber("n42.val");
  b3.lpf = myNex.readNumber("n41.val");
  b4.lpf = myNex.readNumber("n40.val");
  b5.lpf = myNex.readNumber("n39.val");
  b6.lpf = myNex.readNumber("n38.val");
  b7.lpf = myNex.readNumber("n37.val");
  b8.lpf = myNex.readNumber("n36.val");
  b9.lpf = myNex.readNumber("n35.val");
  b10.lpf = myNex.readNumber("n34.val");
  b11.lpf = myNex.readNumber("n33.val");
  saveEEPROM();
}

// BAND V button
void trigger18()
{
  // x12
  band_mode = 3;
  EEPROM.put(70, band_mode);
  display_band_mode();
}

// BCD button
void trigger19()
{
  // x13
  band_mode = 2;
  EEPROM.put(70, band_mode);
  display_band_mode();
}

// TOUCH button
void trigger20()
{
  // x14
  band_mode = 1;
  EEPROM.put(70, band_mode);
  display_band_mode();
}

// APPLY Power in calibration
void trigger22()
{
  // x16
  CalibV_in = myNex.readNumber("mcn0.val");
  CalibV_in = CalibV_in / 1000;
  measured_power_in = myNex.readNumber("mcn1.val");
  IN.calibration = CalibV_in * CalibV_in;
  IN.calibration = IN.calibration / measured_power_in;
  saveEEPROM();
}

// APPLY Power out calibration
void trigger23()
{
  // x17
  CalibV_out = myNex.readNumber("mcn3.val");
  CalibV_out = CalibV_out / 1000;
  measured_power_out = myNex.readNumber("mcn4.val");
  ANT.calibration = CalibV_out * CalibV_out;
  ANT.calibration = ANT.calibration / measured_power_out;
  saveEEPROM();
}

// APPLY ID calibration
void trigger24()
{
  // x18
  calibration_ID = myNex.readNumber("mcn5.val");
  calibration_ID = calibration_ID / peak_ID;
  saveEEPROM();
}

// Load Meter Calibration page
void trigger25()
{
  // x19
  TX_Enable = true;
  display_page = 6;
}

// Load remote setup page
void trigger26()
{
  // x1A
  TX_Enable = false;
  display_page = 8;
}

// Load info page
void trigger27()
{
  // x1B
  TX_Enable = false;
  display_page = 9;
}

// Load Meter Calibration page
void trigger28()
{
  // x1C
  TX_Enable = false;
  display_page = 3;
}

void default_write()
{
  current_lpf = 1;
  current_band = 1;
  ANT.calibration = 1;
  IN.calibration = 1;
  // ADC_ANT.calibration = 900;
  // ADC_IN.calibration = 100;
  measured_power_out = 900;
  measured_power_in = 100;
  calibration_ID = 1.0;
  graph_maxWatt = 1000;
  graph_maxTemp = 100;
  protection_po = 1000;
  protection_swr = 3.0;
  protection_temp = 55;
  T_pepHOLD = 250;
  PWM_FREQ_KHZ = 25;
  protection_in = 60;
  protection_vdd = 55;
  protection_id = 20;
  band_mode = 1;

  b1 = {160, 1, 1, 100, 100};
  b2 = {80, 2, 2, 200, 200};
  b3 = {40, 3, 3, 300, 300};
  b4 = {30, 3, 4, 400, 400};
  b5 = {20, 4, 5, 500, 500};
  b6 = {17, 5, 6, 600, 600};
  b7 = {15, 5, 7, 700, 700};
  b8 = {12, 6, 8, 800, 800};
  b9 = {10, 6, 9, 900, 900};
  b10 = {6, 7, 10, 1000, 1000};
  b11 = {4, 8, 11, 1100, 1100};

  band8_status = 1;
  CATband_status = 0;
  CATrig = 1;

  ntc1_status = 0;
  ntc2_status = 0;
  dtemp1_status = 0;
  dtemp2_status = 0;
  vband_status = 0;
  bcdband_status = 0;
  rotband_status = 0;

  saveEEPROM();
  default_value = 21;
  EEPROM.put(250, default_value);
}

void trigger29()
{
  // x1D
  default_write();
}

int x2i(char *s)
{
  int x = 0;
  for (;;)
  {
    char c = *s;
    if (c >= '0' && c <= '9')
    {
      x *= 16;
      x += c - '0';
    }
    else if (c >= 'A' && c <= 'F')
    {
      x *= 16;
      x += (c - 'A') + 10;
    }
    else if (c >= 'a' && c <= 'f')
    {
      x *= 16;
      x += (c - 'a') + 10;
    }
    else
      break;
    s++;
  }
  return x;
}

void testCAT()
{
  char buff[3];

  CATrig = myNex.readNumber("catrig.val");
  CATbaud = myNex.readNumber("n40.val");

  myNex.readStr("t13.txt").toCharArray(buff, 3);
  CATaddress = x2i(buff);

  RIGprofile();
  CAT_ReqQRG();
}

// Load CAT page
void trigger31()
{
  // x1F
  CAT_ReqQRG();
  TX_Enable = false;

  String CATaddress_hex = String(CATaddress, HEX);
  myNex.writeStr("t13.txt", CATaddress_hex);
  myNex.writeNum("n40.val", CATbaud);
  myNex.writeNum("catrig.val", CATrig);

  myNex.writeStr("t1.txt", rig1);
  myNex.writeStr("t2.txt", rig2);
  myNex.writeStr("t3.txt", rig3);
  myNex.writeStr("t4.txt", rig4);
  myNex.writeStr("t5.txt", rig5);
  myNex.writeStr("t6.txt", rig6);
  myNex.writeStr("t7.txt", rig7);
  myNex.writeStr("t8.txt", rig8);
  myNex.writeStr("t9.txt", rig9);
  myNex.writeStr("t10.txt", rig10);
  myNex.writeStr("t11.txt", rig11);
  myNex.writeStr("t12.txt", rig12);
  display_page = 10;
}

// TEST CAT BUTTON
void trigger32()
{
  // x20
  testCAT();
}

// CAT SETTINGS SAVE
void trigger34()
{
  // x22

  char buff[3];

  CATrig = myNex.readNumber("catrig.val");
  CATbaud = myNex.readNumber("n40.val");

  myNex.readStr("t13.txt").toCharArray(buff, 3);
  CATaddress = x2i(buff);
  EEPROM.put(159, CATrig);
  EEPROM.put(160, CATbaud);
  EEPROM.put(162, CATaddress);
  Serial2.end();
  Serial2.begin(CATbaud);
  RIGprofile();
}

// BAND SELECTION "CAT"
void trigger35()
{
  // x23
  band_mode = 4;
  EEPROM.put(70, band_mode);
  display_band_mode();
  CAT_ReqQRG();
}

void alarm_clear()
{

  digitalWrite(RF_IN, LOW);
  digitalWrite(BIAS_ON, LOW);
  digitalWrite(ANT_RXTX, LOW);
  digitalWrite(PROT_OUT, LOW);

  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
  PROT_OUT_status = LOW;
  extprot_status = LOW;
  inpo_status = LOW;
  lpfswr_status = LOW;
  antswr_status = LOW;

  alarm_code = 0;
  // display_error();
  myNex.writeNum("home.ER.val", 0);
  myNex.writeStr("vis b6,0");
  myNex.writeStr("vis t5,0");
  delay(500);
  display_error();
  SCB_AIRCR = 0x05FA0004; // reboot controller core.
}

// ALARM CLEAR
void trigger21()
{
  // x15
  alarm_clear();
}

/* ************************* CAT SECTION ****************** */

// Convert "Binary coded decimal" HEX value to Integer
int bcd2int(byte bcd_hex)
{
  int cat_temp;
  cat_temp = bcd_hex / 16;
  cat_temp = ((cat_temp * 10) + (bcd_hex - (cat_temp * 16)));
  return cat_temp;
}

// Receive CAT data from Serial port Serial2
void listenCAT()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;

  while (Serial2.available() > 0 && newData == false)
  {
    cat_byte = Serial2.read();

    if (recvInProgress == true)
    {
      if (cat_byte != endMarker)
      {
        // cat_data[ndx] = bcd2int(cat_byte);
        cat_data[ndx] = cat_byte;
        ndx++;
        if (ndx >= cat_length)
        {
          ndx = cat_length - 1;
        }
      }
      else
      {
        cat_data[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (cat_byte == startMarker)
    {
      recvInProgress = true;
    }
  }
}

void ShowCatData()
{
  if (newData == true)
  {

    if ((cat_data[Cn_indx - 1] == Cn_byte) || (cat_data[Cn_indx - 1] == Cn_returnQRG)) // Only take serial data which has frequency data, ignore other data
    {
      if (cat_data[address_indx - 1] == CATaddress) // Check if the data is coming from selected Radio
      {
        cat_MHz = bcd2int(cat_data[data5_indx - 1]) * 100;
        cat_MHz = cat_MHz + bcd2int(cat_data[data4_indx - 1]);

        cat_10KHz = cat_MHz * 100;
        cat_10KHz = cat_10KHz + bcd2int(cat_data[data3_indx - 1]);
        cat_10Hz = bcd2int(cat_data[data2_indx - 1]) * 100;
        cat_10Hz = cat_10Hz + bcd2int(cat_data[data1_indx - 1]);
        myNex.writeNum("x5.val", cat_10KHz);
        myNex.writeNum("x6.val", cat_10Hz);
        newData = false;
      }
      else
        newData = false;
    }
    else
      newData = false;
  }

  else
    newData = false;
}

void cat_page()
{
  listenCAT();
  ShowCatData();
}

/* ========= INTURRUPT ROUTINES =========*/

// Inturupt routine for External triggered protection signal.
FASTRUN void int_extprot()
{
  digitalWriteFast(RF_IN, LOW);
  digitalWriteFast(BIAS_ON, LOW);
  digitalWriteFast(ANT_RXTX, LOW);
  if (alarm_code == 0)
    alarm_code = 1;
  extprot_status = HIGH;
  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
}

// Inturupt routine for high ANT SWR
FASTRUN void int_antswr()
{
  digitalWriteFast(RF_IN, LOW);
  digitalWriteFast(BIAS_ON, LOW);
  digitalWriteFast(ANT_RXTX, LOW);
  digitalWriteFast(PROT_OUT, HIGH);
  alarm_code = 2;
  antswr_status = HIGH;
  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
  PROT_OUT_status = HIGH;
}

// Inturupt routine for high input power
FASTRUN void int_inpo()
{
  digitalWriteFast(RF_IN, LOW);
  digitalWriteFast(BIAS_ON, LOW);
  digitalWriteFast(ANT_RXTX, LOW);
  digitalWriteFast(PROT_OUT, HIGH);
  alarm_code = 3;
  inpo_status = HIGH;
  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
  PROT_OUT_status = HIGH;
}

// Inturupt routine for high LPF SWR
FASTRUN void int_lpfswr()
{
  digitalWriteFast(RF_IN, LOW);
  digitalWriteFast(BIAS_ON, LOW);
  digitalWriteFast(ANT_RXTX, LOW);
  digitalWriteFast(PROT_OUT, HIGH);
  alarm_code = 4;
  lpfswr_status = HIGH;
  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
  PROT_OUT_status = HIGH;
}

// Inturupt routine to monitor PTT status
void int_PTT_CHANGE()
{
  if (digitalRead(PTT_SENSE))
    PTT_status = false;
  else
    PTT_status = true;
  display_ptt();
}

void tx_sequence()
{
  if (PTT_status && alarm_code == 0 && TX_Enable)
  {
    digitalWriteFast(BIAS_ON, HIGH);
    digitalWriteFast(ANT_RXTX, HIGH);
    digitalWriteFast(RF_IN, HIGH);
    BIAS_ON_status = HIGH;
    ANT_RXTX_status = HIGH;
    RF_IN_status = HIGH;
  }
  else
  {
    digitalWriteFast(RF_IN, LOW);
    digitalWriteFast(BIAS_ON, LOW);
    digitalWriteFast(ANT_RXTX, LOW);
    RF_IN_status = LOW;
    BIAS_ON_status = LOW;
    ANT_RXTX_status = LOW;
  }
}

void homepage_normalrun()
{
  display_error();
  band_selection();
  display_band(current_band);
  display_lpf(current_lpf);
  set_ant();
  display_volt(); // Read Volt and Display
  display_ID();   // Read current
  read_temp();    // Read Temp and display
  fanspeed();     // Set fan speed
  tx_sequence();
  if (PTT_status)
  {
    read_ANTpower();     // Read Po and SWR of Antenna SWR Bridge
    read_LPFpower();     // Read Po and SWR of LPF SWR Bridge
    read_INpower();      // Read Po and SWR of input SWR Bridge
    display_power(true); // Display power and swr data
  }
  else
    display_power(false); // Display zero power

  ShowCatData();
}

void homepage_alarmrun()
{
  display_error();
  band_selection();
  display_band(current_band);
  display_lpf(current_lpf);
  display_volt(); // Read Volt and Display
  display_ID();   // Read current
  read_temp();    // Read Temp and display
  fanspeed();     // Set fan speed

  ShowCatData();
}

void menu_page()
{
  // nothing to do in this page
  // calibpage_load = false;
}

void diagnos_page()
{
  band_selection();
  tx_sequence();
  if ((millis() - last_Diag_Refresh) >= Diag_Refresh)
  {

    read_temp();     // Read and display temperature of all available sensors.
    read_ANTpower(); // Read Po and SWR of Antenna SWR Bridge
    read_LPFpower(); // Read Po and SWR of LPF SWR Bridge
    read_INpower();  // Read Po and SWR of input SWR Bridge

    // Show in mV at controller input,
    myNex.writeNum("dn2.val", (ANT.rawFWD) * 1000);
    myNex.writeNum("dn3.val", (ANT.rawREF) * 1000);
    myNex.writeNum("dn4.val", (LPF.rawFWD) * 1000);
    myNex.writeNum("dn5.val", (LPF.rawREF) * 1000);
    myNex.writeNum("dn6.val", (IN.rawFWD) * 1000);
    myNex.writeNum("dn7.val", (IN.rawREF) * 1000);

    fanspeed(); // Set fan speed and display
    myNex.writeNum("dn9.val", V_now() * 1000 * ResV);
    myNex.writeNum("dn10.val", I_now() * 1000);

    // Serial.print("I:");
    // Serial.println(I_now());

    myNex.writeNum("dn11.val", band_rotary() * ref_ADC * 1000); // show Band voltage of rotary swith
    myNex.writeNum("dn12.val", band_volt() * ref_ADC * 1000);   // show auto Band Voltage in mv
    myNex.writeNum("dn13.val", band_bcd());                     // Show BCD input in decimal
    myNex.writeNum("dn8.val", lpf_bank);                        // Show current band selected

    if (PTT_status)
      myNex.writeNum("dn14.val", 1); // Show PTT status
    else
      myNex.writeNum("dn14.val", 0);

    if (inpo_status)
      myNex.writeNum("dn15.val", 1); // Show HI IN status
    else
      myNex.writeNum("dn15.val", 0);

    if (lpfswr_status)
      myNex.writeNum("dn16.val", 1); // Show LPF SWR status
    else
      myNex.writeNum("dn16.val", 0);

    if (antswr_status)
      myNex.writeNum("dn17.val", 1); // Show ANT SWR status
    else
      myNex.writeNum("dn17.val", 0);

    if (extprot_status)
      myNex.writeNum("dn18.val", 1); // Show External Protection status
    else
      myNex.writeNum("dn18.val", 0);

    if (ANT_RXTX_status)
      myNex.writeNum("dn19.val", 1); // Show RX/TX relay status
    else
      myNex.writeNum("dn19.val", 0);

    if (RF_IN_status)
      myNex.writeNum("dn20.val", 1); // Show RF IN relay status
    else
      myNex.writeNum("dn20.val", 0);

    if (BIAS_ON_status)
      myNex.writeNum("dn21.val", 1); // Show BIAS OUT status
    else
      myNex.writeNum("dn21.val", 0);

    if (PROT_OUT_status)
      myNex.writeNum("dn22.val", 1); // Show PROTECTION OUT status
    else
      myNex.writeNum("dn22.val", 0);

    myNex.writeNum("dn23.val", alarm_code); // Show Eroor code

    last_Diag_Refresh = millis();
  }
}

void settings_page()
{
}

void metercalib_page()
{
  if ((millis() - last_metercalib_Refresh) >= metercalib_Refresh)
  {

    if (calibpage_load == false)
    {
      myNex.writeNum("mcn3.val", CalibV_out * 1000);
      myNex.writeNum("mcn4.val", measured_power_out);

      myNex.writeNum("mcn0.val", CalibV_in * 1000);
      myNex.writeNum("mcn1.val", measured_power_in);
      calibpage_load = true;
    }

    band_selection();
    if (last_calib_band != current_band)
    {
      myNex.writeNum("n12.val", current_band);
      myNex.writeNum("n6.val", current_lpf);

      last_calib_band = current_band;
    }

    tx_sequence();

    if (PTT_status)
    {

      CalibV_out = myNex.readNumber("mcn3.val");
      CalibV_out = CalibV_out / 1000;
      CalibV_in = myNex.readNumber("mcn0.val");
      CalibV_in = CalibV_in / 1000;

      maxCalibV_out = ant_fwd_now();
      maxCalibV_in = in_fwd_now();

      if (maxCalibV_out > CalibV_out)
        CalibV_out = maxCalibV_out;

      if (maxCalibV_in > CalibV_in)
        CalibV_in = maxCalibV_in;

      if (last_calib_out != CalibV_out)
      {
        myNex.writeNum("mcn3.val", CalibV_out * 1000);
        last_calib_out = CalibV_out;
      }

      if (last_calib_in != CalibV_in)
      {
        myNex.writeNum("mcn0.val", CalibV_in * 1000);
        last_calib_in = CalibV_in;
      }

      peak_ID = myNex.readNumber("mcn6.val");

      calib_ID = I_now() * 1000;
      if (calib_ID > peak_ID)
        peak_ID = calib_ID;

      if (last_peak_ID != peak_ID)
      {
        myNex.writeNum("mcn6.val", peak_ID);
        last_peak_ID = peak_ID;
      }
    }

    last_metercalib_Refresh = millis();

    // delay(50);
  }
}
/* ======================= Band Calibration page data load ======================= */
void bandcalib_load()
{
  // Load Rotary values from variable
  myNex.writeNum("n17.val", b1.rotary);
  myNex.writeNum("n18.val", b2.rotary);
  myNex.writeNum("n19.val", b3.rotary);
  myNex.writeNum("n20.val", b4.rotary);
  myNex.writeNum("n21.val", b5.rotary);
  myNex.writeNum("n22.val", b6.rotary);
  myNex.writeNum("n23.val", b7.rotary);
  myNex.writeNum("n24.val", b8.rotary);
  myNex.writeNum("n0.val", b9.rotary);
  myNex.writeNum("n27.val", b10.rotary);
  myNex.writeNum("n30.val", b11.rotary);

  // Load band volt values from variable
  myNex.writeNum("n1.val", b1.volt);
  myNex.writeNum("n2.val", b2.volt);
  myNex.writeNum("n3.val", b3.volt);
  myNex.writeNum("n4.val", b4.volt);
  myNex.writeNum("n5.val", b5.volt);
  myNex.writeNum("n6.val", b6.volt);
  myNex.writeNum("n7.val", b7.volt);
  myNex.writeNum("n8.val", b8.volt);
  myNex.writeNum("n25.val", b9.volt);
  myNex.writeNum("n28.val", b10.volt);
  myNex.writeNum("n31.val", b11.volt);

  // Load bcd values from variable
  myNex.writeNum("n9.val", b1.bcd);
  myNex.writeNum("n10.val", b2.bcd);
  myNex.writeNum("n11.val", b3.bcd);
  myNex.writeNum("n12.val", b4.bcd);
  myNex.writeNum("n13.val", b5.bcd);
  myNex.writeNum("n14.val", b6.bcd);
  myNex.writeNum("n15.val", b7.bcd);
  myNex.writeNum("n16.val", b8.bcd);
  myNex.writeNum("n26.val", b9.bcd);
  myNex.writeNum("n29.val", b10.bcd);
  myNex.writeNum("n32.val", b11.bcd);

  // Load lpf selection values from variable
  myNex.writeNum("n43.val", b1.lpf);
  myNex.writeNum("n42.val", b2.lpf);
  myNex.writeNum("n41.val", b3.lpf);
  myNex.writeNum("n40.val", b4.lpf);
  myNex.writeNum("n39.val", b5.lpf);
  myNex.writeNum("n38.val", b6.lpf);
  myNex.writeNum("n37.val", b7.lpf);
  myNex.writeNum("n36.val", b8.lpf);
  myNex.writeNum("n35.val", b9.lpf);
  myNex.writeNum("n34.val", b10.lpf);
  myNex.writeNum("n33.val", b11.lpf);
}

void bandcalib_page()
{
  if ((millis() - last_bandcalib_Refresh) >= bandcalib_Refresh)
  {
    myNex.writeNum("rot.val", band_rotary());
    myNex.writeNum("volt.val", band_volt());
    myNex.writeNum("bcd.val", band_bcd());
    last_bandcalib_Refresh = millis();
  }
}

void trigger13()
{ // x0D
  TX_Enable = false;
  bandcalib_load();
  display_page = 7;
}

void remote_page()
{
}

void info_page()
{
}

void setup()
{
  Wire2.begin();
  delay(1000);
  Serial.begin(9600);

  // Serial.println("*** SETUP START ***");
  myNex.begin(921600); // start Nextion Display

  MCP.begin();
  MCP.pinMode8(0x00);

  analogReadAveraging(4);

  // PWMsetup();
  analogWriteFrequency(FAN_1, PWM_FREQ_KHZ * 1000); // Teensy pin changes to 25 kHz
  analogWriteFrequency(FAN_2, PWM_FREQ_KHZ * 1000); // Teensy pin changes to 25 kHz

  // TEENSY INTARRUPT

  // Attach inturupt for external protection detection
  pinMode(INT_PROT, INPUT_PULLUP);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(INT_PROT), int_extprot, FALLING);

  // Attach inturupt for high swr
  pinMode(INT_ANT, INPUT_PULLUP);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(INT_ANT), int_antswr, RISING);

  // Attach inturupt for high input
  pinMode(INT_IN, INPUT_PULLUP);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(INT_IN), int_inpo, RISING);

  // Attach inturupt for high LPF swr error
  pinMode(INT_LPF, INPUT_PULLUP);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(INT_LPF), int_lpfswr, RISING);

  // Attach inturupt for PTT
  pinMode(PTT_SENSE, INPUT_PULLUP);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(PTT_SENSE), int_PTT_CHANGE, CHANGE);

  pinMode(NTC1, INPUT_DISABLE);
  pinMode(NTC2, INPUT_DISABLE);

  pinMode(VCC, INPUT_DISABLE);
  pinMode(ID, INPUT_DISABLE);

  pinMode(IN_FWD, INPUT_DISABLE);
  pinMode(IN_REF, INPUT_DISABLE);

  pinMode(LPF_FWD, INPUT_DISABLE);
  pinMode(LPF_REF, INPUT_DISABLE);

  pinMode(ANT_FWD, INPUT_DISABLE);
  pinMode(ANT_REF, INPUT_DISABLE);

  pinMode(BAND_V, INPUT_DISABLE);
  pinMode(ROTARY_POS, INPUT_DISABLE);
  pinMode(BAND_A, INPUT_PULLUP);
  pinMode(BAND_B, INPUT_PULLUP);
  pinMode(BAND_C, INPUT_PULLUP);
  pinMode(BAND_D, INPUT_PULLUP);

  pinMode(BIAS_ON, OUTPUT);
  pinMode(GPIO_O9, OUTPUT);
  pinMode(GPIO_13, OUTPUT);
  pinMode(RF_IN, OUTPUT);
  pinMode(ANT_RXTX, OUTPUT);
  pinMode(ANT2, OUTPUT);
  pinMode(Relay_1, OUTPUT);
  pinMode(PROT_OUT, OUTPUT);
  pinMode(BIAS_ON, OUTPUT);

  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_13 |= IOMUXC_PAD_SPEED(1) | IOMUXC_PAD_DSE(4) | IOMUXC_PAD_SRE; // 34
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_02 |= IOMUXC_PAD_SPEED(1) | IOMUXC_PAD_DSE(4) | IOMUXC_PAD_SRE; // 36

  digitalWrite(BIAS_ON, LOW);
  digitalWrite(RF_IN, LOW);
  digitalWrite(ANT_RXTX, LOW);
  digitalWrite(PROT_OUT, LOW);

  EEPROM.get(250, default_value);
  if (default_value != 21)
    default_write();
  // default_write();

  // Load variable values and setting parameters from EEPROM
  loadEEPROM();

  // Start CAT
  Serial2.begin(CATbaud);
  myNex.writeNum("cat.catrig.val", CATrig);
  // ==== Temp sensor setup ====

  if (dtemp1_status)
  {
    sensors.begin();
    sensors.getAddress(tempDeviceAddress, 0);
    sensors.setResolution(tempDeviceAddress, resolution);
    if (dtemp2_status)
    {
      sensors.getAddress(tempDeviceAddress, 1);
      sensors.setResolution(tempDeviceAddress, resolution);
    }

    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
  }
  Temp_Refresh = 750 / (1 << (12 - resolution));
  last_Temp_Refresh = millis();

  trigger11();
  myNex.writeStr("Er1.txt", "");
  // Serial.println("*** SETUP END ***");

  ANT.maxgraphSWR = 3;

  // Serial.print("band:");
  // Serial.println(current_band);
  // Serial.print("LPF:");
  // Serial.println(current_lpf);

  display_band_mode();
  display_band(current_band);
  display_lpf(current_lpf);
  set_lpf(current_lpf);
  set_ant();

  display_volt(); // Read Volt and Display
  display_ID();   // Read current
  read_temp();    // Read Temp and display
  fanspeed();
  RIGprofile(); // Load Radio CAT profile
  CAT_ReqQRG(); // Request radio to send frequency
}

void loop()
{
  while (1)
  {
    listenCAT();

    //  Receive from display if not in TX
    if (PTT_status != HIGH)
    {
      myNex.NextionListen();
    }

    // Home page tasks
    if (display_page == 2)
    {
      if (alarm_code == 0)
      {
        // Serial.println("*** HOME NORMAL RUN ***");
        homepage_normalrun();
      }

      else
      {
        // Serial.println("*** HOME ALARM RUN ***");
        homepage_alarmrun();
      }
    }

    else if (display_page == 3)
    {
      // Serial.println("*** MENU PAGE ***");
      menu_page();
    }
    else if (display_page == 4)
    {
      // Serial.println("*** DIAG PAGE ***");
      diagnos_page();
    }
    else if (display_page == 5)
    {
      // Serial.println("*** SETTING PAGE ***");
      settings_page();
    }
    else if (display_page == 6)
    {
      // Serial.println("*** METER CALIB PAGE ***");

      metercalib_page();
    }
    else if (display_page == 7)
    {
      // Serial.println("*** BAND CALIB PAGE ***");
      bandcalib_page();
    }
    else if (display_page == 8)
    {
      //  Serial.println("*** REMOTE PAGE ***");
      remote_page();
    }
    else if (display_page == 9)
    {
      // Serial.println("*** INFO PAGE ***");
      info_page();
    }
    else if (display_page == 10)
    {
      // Serial.println("*** CAT PAGE ***");
      cat_page();
    }
  }
}
