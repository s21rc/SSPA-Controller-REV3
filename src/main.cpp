/*                                                                                          */
/*  Arduino Code (Version 3.0) for S21RC SSPA Controller REV3.0 PCB.                       */
/*                                                                                          */
/*  Created by Fazlay Rabby S21RC, JUNE, 2023.                                              */
/*  Released into the public domain.                                                        */
/*                                                                                          */
/*  Feel free to use/change the Gerber file for the PCB,                                    */
/*  display file(nextion TFT) for the display and all codes here.                           */
/*                                                                                          */
/*  If you need any clarification or help please issue a ticket in github.                  */
/*                                                                                          */

/* For PCB Version REV3.0, CODE V 3.0, Display: Nextion 5inch (800x480)                     */

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*    THE CODE IS STILL IN TESTING PHASE, WORK IN PROGRESS   */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include "Arduino.h"
#include <EEPROM.h>
#include "math.h"
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>  //https://github.com/milesburton/Arduino-Temperature-Control-Library
#include "EasyNextionLibrary.h" // https://github.com/Seithan/EasyNextionLibrary
#include <PCF8574.h>            //https://registry.platformio.org/libraries/robtillaart/PCF8574
#include <pinout.h>

bool debug = true;

// PCF8574 pcf8574(0x38);
PCF8574 PCF(0x38, &Wire2);

EasyNex myNex(UART_DISP);

/* ======= Tempareture variables ======== */
// Digital Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
int resolution = 9;
unsigned long last_Temp_Refresh = 0;

float temp_digi1 = 0.0;
float temp_digi2 = 0.0;
float last_temp_digi1 = 0.0;
float last_temp_digi2 = 0.0;

// NTC temp
#define ntcbeta 3950 // for 10K 3950 NTC

float adc_ntc1 = 0.0;
float adc_ntc2 = 0.0;
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
uint8_t protection_swr = 3;
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

/* USER DEFINED VARIABLES, PUT AS PER YOUR BOARD */

uint8_t graph_maxSwr = 3;             // = 3; // Scale of the SWR for display
unsigned long Temp_Refresh = 0;       //
unsigned long Display_Refresh = 1000; // refresh Temp display every N mili sec, set as per your liking
unsigned long Volt_Refresh = 2000;    // refresh V display every N mili sec, set as per your liking
unsigned long ID_Refresh = 1000;      // refresh I display every N mili sec, set as per your liking
unsigned long Effi_Refresh = 1000;    // refresh Efficiency display every N mili sec, set as per your liking
unsigned long Power_Refresh = 250;
unsigned long Diag_Refresh = 200;
float ResV1 = 10000.00; // Set R1 of voltage devider
float ResV2 = 470.00;   // Set R2 of voltage devider
float ResP1 = 1800.00;  // Set R1 of voltage devider for power/swr input
float ResP2 = 470.00;   // Set R2 of voltage devider for power/swr input
int RL = 2530;          // Load Resistor at Current sensor

uint8_t band_margin = 30; // around 90mV margin for change of voltage reference between SSPA and Radio

float ref_ADC = (3.3096 / 1024); // Refrence is 3.3v for Teensy ADC

// Calibration factors
uint16_t measured_power_out;
uint16_t measured_power_in;

float VFdiode = 0.3; //
float VFdiode_ADC = VFdiode / ref_ADC;

int calibration_out;
int calibration_in;

uint16_t ADC_calibration_out;
uint16_t ADC_calibration_in;
uint16_t ADC_maxCalibV_out;
uint16_t ADC_maxCalibV_in;

bool calibpage_load = false;
uint8_t last_calib_band = 0;
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
bool rotary_status = false;

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

char error_text[20] = "";
uint8_t default_value = 0;

volatile uint8_t alarm_code = 0;
uint8_t last_alarm_code = 0;
bool last_reset_alarm = LOW;

uint8_t V = 0;
uint8_t lastV = 0;
float ResV = ResV2 / (ResV1 + ResV2);
uint8_t I;
uint8_t lastI = 0;

uint8_t graph_Watt = 0;
uint8_t graph_Temp1 = 0;
uint8_t graph_Temp2 = 0;
uint8_t graph_Temp3 = 0;
uint8_t graph_Temp4 = 0;

uint8_t graph_Swr = 0;
uint8_t graph_Swr_ANT = 0;
uint8_t graph_Swr_IN = 0;

/* ======== power and swr related variable ======== */

float ResP = ResP2 / (ResP1 + ResP2); // Voltage devider for SWR/POWER/LPF
unsigned int lastANTpep = 0;          // update PEP display
unsigned int lastLPFpep = 0;          // update PEP display LPF
unsigned int lastINpep = 0;           // update PEP display LPF

unsigned int ant_power_fwd = 0; // ANT power forward (watts)
unsigned int ant_power_ref = 0; // ANT power reflected (watts)
float ant_raw_fwd = 0;          // ANT power forward for SWR calculation
float ant_raw_ref = 0;          // ANT power reflected for SWR calculation
uint16_t ant_power_fwd_max = 0; // power forward max (for peak hold)
uint16_t ant_power_ref_max = 0; // power reflected max (for peak hold)

unsigned int lpf_power_fwd = 0; // LPF power forward (watts)
unsigned int lpf_power_ref = 0; // LPF power reflected (watts)
float lpf_raw_fwd = 0;          // LPF power forward for SWR calculation
float lpf_raw_ref = 0;          // LPF power reflected for SWR calculation
uint16_t lpf_power_fwd_max = 0; // power forward max (for peak hold)
uint16_t lpf_power_ref_max = 0; // power reflected max (for peak hold)

unsigned int in_power_fwd = 0; // LPF power forward (watts)
unsigned int in_power_ref = 0; // LPF power reflected (watts)
float in_raw_fwd = 0;          // LPF power forward for SWR calculation
float in_raw_ref = 0;          // LPF power reflected for SWR calculation
uint16_t in_power_fwd_max = 0; // power forward max (for peak hold)
uint16_t in_power_ref_max = 0; // power reflected max (for peak hold)

float ANT_SWR = 0;        // ANT_SWR
float LPF_SWR = 0;        // LPF_SWR
float IN_SWR = 0;         // Input_SWR
float power_ratio = 0;    // Power ratio P forward / P refl
uint8_t swr_display = 10; // swr x 10 showing in display

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
  uint16_t band_volt_adc = 0;

  for (uint8_t i = 0; i < 10; i++)
  {
    band_volt_adc += analogRead(BAND_V);
  }
  band_volt_adc /= 10;

  return band_volt_adc;
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
  uint16_t band_rotary_adc = 0;
  for (uint8_t i = 0; i < 10; i++)
  {
    band_rotary_adc += analogRead(ROTARY_POS);
  }
  band_rotary_adc /= 10;
  return band_rotary_adc;
}

// Band Mapping from ROTARY to Band#
uint8_t rotary2band(uint8_t band_rotary_adc)
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
  EEPROM.get(1, ADC_calibration_out);
  EEPROM.get(3, ADC_calibration_in);
  EEPROM.get(5, measured_power_out);
  EEPROM.get(7, measured_power_in);
  EEPROM.get(9, calibration_ID);
  EEPROM.get(13, calibration_out);
  EEPROM.get(17, calibration_in);

  EEPROM.get(21, graph_maxWatt);
  EEPROM.get(25, graph_maxTemp);
  EEPROM.get(30, protection_po);
  EEPROM.get(35, protection_swr);
  EEPROM.get(40, protection_temp);
  EEPROM.get(45, T_pepHOLD);
  EEPROM.get(50, PWM_FREQ_KHZ);
  EEPROM.get(55, protection_in);
  EEPROM.get(60, protection_vdd);
  EEPROM.get(65, protection_id);

  EEPROM.get(70, band_mode);
  EEPROM.get(72, current_band);
  EEPROM.get(74, current_lpf);

  EEPROM.get(150, ntc1_status);
  EEPROM.get(151, ntc2_status);
  EEPROM.get(152, dtemp1_status);
  EEPROM.get(153, dtemp2_status);
  EEPROM.get(154, vband_status);
  EEPROM.get(155, bcdband_status);
  EEPROM.get(156, rotband_status);
  EEPROM.get(157, band8_status);

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
}

void saveEEPROM()
{

  EEPROM.put(1, ADC_calibration_out);
  EEPROM.put(3, ADC_calibration_in);
  EEPROM.put(5, measured_power_out);
  EEPROM.put(7, measured_power_in);
  EEPROM.put(9, calibration_ID);
  EEPROM.put(13, calibration_out);
  EEPROM.put(17, calibration_in);

  EEPROM.put(21, graph_maxWatt);
  EEPROM.put(25, graph_maxTemp);
  EEPROM.put(30, protection_po);
  EEPROM.put(35, protection_swr);
  EEPROM.put(40, protection_temp);
  EEPROM.put(45, T_pepHOLD);
  EEPROM.put(50, PWM_FREQ_KHZ);
  EEPROM.put(55, protection_in);
  EEPROM.put(60, protection_vdd);
  EEPROM.put(65, protection_id);

  EEPROM.put(70, band_mode);
  EEPROM.put(72, current_band);
  EEPROM.put(74, current_lpf);

  EEPROM.put(150, ntc1_status);
  EEPROM.put(151, ntc2_status);
  EEPROM.put(152, dtemp1_status);
  EEPROM.put(153, dtemp2_status);
  EEPROM.put(154, vband_status);
  EEPROM.put(155, bcdband_status);
  EEPROM.put(156, rotband_status);
  EEPROM.put(157, band8_status);

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
    // myNex.writeStr("tsw b8,1");
    // myNex.writeStr("tsw b9,1");
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
  adc_ntc1 = 0;

  for (uint8_t i = 0; i < 10; i++)
  {
    adc_ntc1 += analogRead(NTC1);
  }
  adc_ntc1 = adc_ntc1 / 10;
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
  adc_ntc2 = 0;

  for (uint8_t i = 0; i < 10; i++)
  {
    adc_ntc2 += analogRead(NTC2);
  }
  adc_ntc2 = adc_ntc2 / 10;

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

  for (uint8_t i = 0; i < 10; i++)
  {
    currentI += analogRead(ID);
  }
  currentI /= 10;
  currentI *= ref_ADC;
  if (display_page == 2)
  {
    currentI /= RL;
    currentI *= 13000;
  }

  return currentI;
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
  for (uint8_t i = 0; i < 10; i++)
  {
    currentV += analogRead(VCC);
  }
  currentV = currentV / 10;
  currentV = (currentV * ref_ADC);
  currentV /= ResV;
  return currentV;
}

/* *=== RF Power measurement Function === */

float ant_fwd_now()
{
  float ANTFWD = 0.0;
  for (uint8_t i = 0; i < 10; i++)
  {
    ANTFWD += analogRead(ANT_FWD);
  }
  ANTFWD = ANTFWD / 10;
  ANTFWD = ((ANTFWD * ref_ADC) / ResP) + VFdiode;
  return ANTFWD;
}

float ant_ref_now()
{
  float ANTREF = 0.0;
  for (uint8_t i = 0; i < 10; i++)
  {
    ANTREF += analogRead(ANT_REF);
  }
  ANTREF = ANTREF / 10;
  ANTREF = ((ANTREF * ref_ADC) / ResP) + VFdiode;
  return ANTREF;
}

float lpf_fwd_now()
{
  float LPFFWD = 0.0;
  for (uint8_t i = 0; i < 10; i++)
  {
    LPFFWD += analogRead(LPF_FWD);
  }
  LPFFWD = LPFFWD / 10;
  LPFFWD = ((LPFFWD * ref_ADC) / ResP) + VFdiode;
  if (debug)
  {
    Serial.println("LPFFWD:");
    Serial.println(LPFFWD);
  }
  return LPFFWD;
}

float lpf_ref_now()
{
  float LPFREF = 0.0;
  for (uint8_t i = 0; i < 10; i++)
  {
    LPFREF += analogRead(LPF_REF);
  }
  LPFREF = LPFREF / 10;
  LPFREF = ((LPFREF * ref_ADC) / ResP) + VFdiode;
  return LPFREF;
}

float in_fwd_now()
{
  float INFWD = 0.0;
  for (uint8_t i = 0; i < 10; i++)
  {
    INFWD += analogRead(IN_FWD);
  }
  INFWD = INFWD / 10;
  INFWD = ((INFWD * ref_ADC) / ResP) + VFdiode;
  return INFWD;
}

float in_ref_now()
{
  float INREF = 0.0;
  for (uint8_t i = 0; i < 10; i++)
  {
    INREF += analogRead(IN_REF);
  }
  INREF = INREF / 10;
  INREF = ((INREF * ref_ADC) / ResP) + VFdiode;
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

  ant_raw_fwd = ant_fwd_now();
  ant_raw_ref = ant_ref_now();

  // Calculate Forward power
  if (ant_raw_fwd > VFdiode)
  { // only correct for diode voltage when more than zero
    ant_power_fwd = (ant_raw_fwd * ant_raw_fwd) / calibration_out;
  }
  else
    ant_power_fwd = 0;

  // Calculate Reflected power
  if (ant_raw_ref > VFdiode)
  {
    ant_power_ref = (ant_power_ref * ant_power_ref) / calibration_out;
  }
  else
    ant_power_ref = 0;

  // Calculate SWR
  if (ant_power_ref > 0 && !ant_power_fwd == 0)
  {
    power_ratio = ant_power_fwd / ant_power_ref;
    ANT_SWR = abs((1 + sqrt(power_ratio)) / (1 - sqrt(power_ratio)));
  }
  else
    ANT_SWR = 1;

  /* === Check if OUTPUT is higher than set value  === */
  if (ant_power_fwd >= protection_po)
  {
    soft_protection();
    alarm_code = 6;
  }

  /* === Check if SWR is higher than set value  === */
  if (ANT_SWR >= protection_swr)
  {
    soft_protection();
    alarm_code = 7;
  }

  /* === Check current vs Output Power to see if LPF mismatch  === */
  if (Iprot >= 10.0 && ant_power_fwd <= 100)
  {
    soft_protection();
    alarm_code = 8;
  }

  // hold peak
  if (ant_power_fwd >= ant_power_fwd_max)
  {
    lastANTpep = millis();
    ant_power_fwd_max = ant_power_fwd;
  }

  if (millis() > (lastANTpep + T_pepHOLD))
    ant_power_fwd_max = ant_power_fwd; // clear the peak after hold time
  // Efficiency calculation
  if (Veff != 0 && Iprot != 0)
    Eff = 100 * (ant_power_fwd / (Veff * Iprot));
}

void read_LPFpower()
{

  lpf_raw_fwd = lpf_fwd_now();
  lpf_raw_ref = lpf_ref_now();

  // Calculate Forward power
  if (lpf_raw_fwd > VFdiode)
  { // only correct for diode voltage when more than zero
    lpf_power_fwd = (lpf_raw_fwd * lpf_raw_fwd) / calibration_out;
  }
  else
    lpf_power_fwd = 0;

  // Calculate Reflected power
  if (lpf_raw_ref > VFdiode)
  { // only correct for diode voltage when more than zero
    lpf_power_ref = (lpf_power_ref * lpf_power_ref) / calibration_out;
  }
  else
    lpf_power_ref = 0;

  // Calculate SWR
  if (lpf_power_ref > 0 && !lpf_power_fwd == 0)
  {
    power_ratio = lpf_power_fwd / lpf_power_ref;
    LPF_SWR = abs((1 + sqrt(power_ratio)) / (1 - sqrt(power_ratio)));
  }
  else
    LPF_SWR = 1;

  /* === Check if OUTPUT is higher than set value  === */
  if (lpf_power_fwd >= protection_po)
  {
    if (debug)
    {
      Serial.println("Alarm 9 Prot PO");
      Serial.println(calibration_out);
      Serial.println(lpf_power_fwd);
      Serial.println(protection_po);
    }
    soft_protection();
    alarm_code = 9;
  }

  /* === Check if SWR is higher than set value  === */
  if (LPF_SWR >= protection_swr)
  {
    soft_protection();
    alarm_code = 10;
  }

  // hold peak
  if (lpf_power_fwd >= lpf_power_fwd_max)
  {
    lastLPFpep = millis();
    lpf_power_fwd_max = lpf_power_fwd;
  }

  if (millis() > (lastLPFpep + T_pepHOLD))
    lpf_power_fwd_max = lpf_power_fwd; // clear the peak after hold time
}

void read_INpower()
{

  in_raw_fwd = in_fwd_now();
  in_raw_ref = in_ref_now();

  // Calculate Forward power
  if (in_raw_fwd > VFdiode)
  { // only correct for diode voltage when more than zero
    in_power_fwd = (in_raw_fwd * in_raw_fwd) / calibration_in;
  }
  else
    in_power_fwd = 0;

  // Calculate Reflected power
  if (in_raw_ref > VFdiode)
  {
    in_power_ref = (in_power_ref * in_power_ref) / calibration_in;
  }
  else
    in_power_ref = 0;

  // Calculate SWR
  if (in_power_ref > 0 && !in_power_fwd == 0)
  {
    power_ratio = in_power_fwd / in_power_ref;
    IN_SWR = abs((1 + sqrt(power_ratio)) / (1 - sqrt(power_ratio)));
  }
  else
    IN_SWR = 1;

  /* === Check if OUTPUT is higher than set value  === */

  if (in_power_fwd >= protection_in)
  {
    soft_protection();
    alarm_code = 11;
  }
  // hold peak
  if (in_power_fwd >= in_power_fwd_max)
  {
    lastINpep = millis();
    in_power_fwd_max = in_power_fwd;
  }

  if (millis() > (lastINpep + T_pepHOLD))
    in_power_fwd_max = in_power_fwd; // clear the peak after hold time
}

/* === SWR/Po display Function === */
// Refresh screen every N miliseconds (N = PdelayInMillis)
void display_power(bool active)
{

  if (active)
  {
    if ((millis() - last_Power_Refresh) >= Power_Refresh)
    {

      swr_display = (ANT_SWR * 10) + 0.5; //// Float x 10 and convert to int with rounding
      if (swr_display < 10)
        swr_display = 10; // SWR cannot be lower than 1.0

      float graph_limit_watt = (graph_maxWatt / 100.00);
      graph_Watt = (ant_power_fwd_max / graph_limit_watt);

      float graph_limit_swr = ((graph_maxSwr - 1) / 100.00);
      float swr_forgraph = ANT_SWR - 1;
      graph_Swr = swr_forgraph / graph_limit_swr;

      myNex.writeNum("n1.val", ant_power_fwd);
      myNex.writeNum("n0.val", ant_power_fwd_max);
      myNex.writeNum("j0.val", graph_Watt);
      myNex.writeNum("x1.val", swr_display);
      myNex.writeNum("j1.val", graph_Swr);
      myNex.writeNum("n3.val", ant_power_fwd);
      myNex.writeNum("n4.val", ant_power_ref);
      myNex.writeNum("n5.val", lpf_power_fwd);
      myNex.writeNum("n6.val", lpf_power_ref);
      myNex.writeNum("n7.val", Eff);

      ant_power_fwd_max = 0;
      graph_Watt = 0;
      swr_display = 10;
      graph_Swr = 0;
      Eff = 0;

      last_Power_Refresh = millis();
    }
  }
  else
  {
    if ((millis() - last_Power_Refresh) >= Power_Refresh)
    {
      ant_power_fwd_max = 0;
      graph_Watt = 0;
      swr_display = 10;
      graph_Swr = 0;
      Eff = 0;

      myNex.writeNum("n0.val", ant_power_fwd_max);
      myNex.writeNum("j0.val", graph_Watt);
      myNex.writeNum("x1.val", swr_display);
      myNex.writeNum("j1.val", graph_Swr);
      myNex.writeNum("n7.val", Eff);
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

void lpf(int Nrelay)
{
  if (last_lpf_relay != Nrelay)
  {
    EEPROM.put(72, current_band); // Update band info to Memory when LPF chnages
    EEPROM.put(74, Nrelay);       // Update band info to Memory when LPF chnages
    PCF.selectN(Nrelay);
    Serial.print("Nrelay:");
    Serial.println(Nrelay);
    Serial.print("Current Band:");
    Serial.println(current_band);

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
      myNex.writeNum("er1.val", 0);
      myNex.writeNum("er2.val", 0);
      myNex.writeNum("er3.val", 0);
      myNex.writeNum("er4.val", 0);
      myNex.writeNum("er5.val", 0);
      myNex.writeNum("er6.val", 0);
      myNex.writeNum("er7.val", 0);
      myNex.writeNum("er8.val", 0);
      myNex.writeStr("er.txt", " ");
      break;
    case 1:
      myNex.writeNum("er8.val", 1);
      myNex.writeStr("er.txt", "External Protection Detected (H), correct and clear alarm");
      break;
    case 2:
      myNex.writeNum("er1.val", 1);
      myNex.writeStr("er.txt", "ANT SWR high (H), correct and clear alarm");
      break;
    case 3:
      myNex.writeNum("er7.val", 1);
      myNex.writeStr("er.txt", "High INPUT power (H), correct and clear alarm");
      break;
    case 4:
      myNex.writeNum("er2.val", 1);
      myNex.writeStr("er.txt", "LPF SWR High (H), correct and clear alarm");
      break;
    case 5:
      myNex.writeNum("er3.val", 1);
      myNex.writeStr("er.txt", "High Temperature (S), correct and clear alarm");
      break;
    case 6:
      myNex.writeNum("er6.val", 1);
      myNex.writeStr("er.txt", "High OUTPUT power (S), correct and clear alarm");
      break;
    case 7:
      myNex.writeNum("er1.val", 1);
      myNex.writeStr("er.txt", "High SWR (S), correct and clear alarm");
      break;
    case 8:
      myNex.writeNum("er2.val", 1);
      myNex.writeStr("er.txt", "LPF mismatch (S), correct and clear alarm");
      break;
    case 9:
      myNex.writeNum("er6.val", 1);
      myNex.writeStr("er.txt", "LPF OUTPUT high (S), correct and clear alarm");
      break;
    case 10:
      myNex.writeNum("er2.val", 1);
      myNex.writeStr("er.txt", "LPF SWR high (S), correct and clear alarm");
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
    myNex.writeNum("Ca.val", I);
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

      // show NTC output in degree C
      /*
      myNex.writeNum("n0.val", Tdisplay1);
      myNex.writeNum("n1.val", Tdisplay2);
      */

      // show NTC output in milivolt
      myNex.writeNum("dn0.val", adc_ntc1 * ref_ADC);
      myNex.writeNum("dn1.val", adc_ntc2 * ref_ADC);

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

void band_selection()
{
  if (PTT_status != HIGH)
  {
    if (band_rotary() < 10)
    {
      if (band_mode == 1)
      {
        LCDband_enable();
        // TOUCH
        current_band = lpf2band(lpf_bank);
        lpf(lpf_bank);
        myNex.writeNum("bt0.val", 0);
        myNex.writeNum("bt1.val", 0);
        myNex.writeNum("bt2.val", 0);
        myNex.writeNum("bt3.val", 1);
      }

      else if (band_mode == 2)
      {
        // BCD
        LCDband_disable();
        current_band = bcd2band(band_bcd());
        lpf_bank = band2lpf(current_band);
        lpf(lpf_bank);
        myNex.writeNum("bt0.val", 0);
        myNex.writeNum("bt1.val", 0);
        myNex.writeNum("bt2.val", 1);
        myNex.writeNum("bt3.val", 0);
      }
      else if (band_mode == 3)
      {
        // BAND V
        LCDband_disable();
        current_band = bandV2band(band_volt());
        lpf_bank = band2lpf(current_band);
        lpf(lpf_bank);
        myNex.writeNum("bt0.val", 1);
        myNex.writeNum("bt1.val", 0);
        myNex.writeNum("bt2.val", 0);
        myNex.writeNum("bt3.val", 0);
      }
    }
    else
    {
      // ROTARY
      if (current_band != last_band)
      {

        LCDband_disable();
        current_band = rotary2band(band_rotary());
        lpf_bank = band2lpf(current_band);
        lpf(lpf_bank);
        last_band = current_band;
        myNex.writeNum("bt0.val", 0);
        myNex.writeNum("bt1.val", 1);
        myNex.writeNum("bt2.val", 0);
        myNex.writeNum("bt3.val", 0);
      }
    }
  }
}

/* === Trigger from Nextion Display  === */
void trigger1()
{ // x01
  band_mode = 1;
  lpf_bank = 1;
}

void trigger2()
{ // x02
  band_mode = 1;
  lpf_bank = 2;
}

void trigger3()
{ // x03
  band_mode = 1;
  lpf_bank = 3;
}

void trigger4()
{ // x04
  band_mode = 1;
  lpf_bank = 4;
}

void trigger5()
{ // x05
  band_mode = 1;
  lpf_bank = 5;
}

void trigger6()
{ // x06
  band_mode = 1;
  lpf_bank = 6;
}

void trigger7()
{ // x07
  band_mode = 1;
  lpf_bank = 7;
}

void trigger8()
{ // x08
  band_mode = 1;
  lpf_bank = 8;
}

/*
// Antenna2 switch
void trigger8()
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
} */

void trigger9() // Loading variables data to Setting page
{               // x09
  TX_Enable = false;

  myNex.writeNum("st0.val", graph_maxWatt);
  myNex.writeNum("st1.val", T_pepHOLD);
  myNex.writeNum("st7.val", graph_maxTemp);
  myNex.writeNum("st8.val", PWM_FREQ_KHZ);

  // Checkboxes
  myNex.writeNum("sc0.val", ntc1_status);
  myNex.writeNum("sc1.val", ntc2_status);
  myNex.writeNum("sc2.val", dtemp1_status);

  if (dtemp1_status == 1)
    myNex.writeNum("sc3.val", dtemp2_status);

  myNex.writeNum("sc4.val", vband_status);
  myNex.writeNum("sc5.val", bcdband_status);
  myNex.writeNum("sc6.val", rotband_status);

  myNex.writeNum("st2.val", protection_temp);
  myNex.writeNum("st3.val", protection_po);
  myNex.writeNum("st4.val", protection_vdd);
  myNex.writeNum("st5.val", protection_id);
  myNex.writeNum("stx0.val", protection_swr);
  myNex.writeNum("st6.val", protection_in);

  display_page = 5;
}

void trigger10() // updating variables data from Setting page and EEPROM
{                // x0A

  graph_maxWatt = myNex.readNumber("st0.val");
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

  protection_temp = myNex.readNumber("st2.val");
  protection_po = myNex.readNumber("st3.val");
  protection_vdd = myNex.readNumber("st4.val");
  protection_id = myNex.readNumber("st5.val");
  protection_swr = myNex.readNumber("stx0.val");
  protection_in = myNex.readNumber("st6.val");

  saveEEPROM();
  display_page = 5;
}

void trigger11()
{ // x0B HOME SCREEN
  TX_Enable = true;

  // Disable Band 8 (4m button if not selected by setting page)
  if (band8_status == 0)
  {
    myNex.writeStr("vis bn8,0");
  }
  else
    myNex.writeStr("vis bn8,1");

  display_page = 2;
}

void trigger12()
{ // x0C DIAGNOS PAGE
  TX_Enable = true;
  display_page = 4;
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
  ;
  b10.lpf = myNex.readNumber("n34.val");
  b11.lpf = myNex.readNumber("n33.val");
  saveEEPROM();
}

// BAND V button
void trigger18()
{
  // x12
  band_mode = 3;
}

// BCD button
void trigger19()
{
  // x13
  band_mode = 2;
}

// TOUCH button
void trigger20()
{
  // x14
  band_mode = 1;
}

// APPLY Power in calibration
void trigger22()
{
  // x16
  measured_power_in = myNex.readNumber("mcn1.val");
  calibration_in = ADC_calibration_in + VFdiode_ADC;
  calibration_in = calibration_in * calibration_in;
  calibration_in = calibration_in / measured_power_in;
  saveEEPROM();
}

// APPLY Power out calibration
void trigger23()
{
  // x17
  measured_power_out = myNex.readNumber("mcn4.val");
  calibration_out = ADC_calibration_out + VFdiode_ADC;
  calibration_out = calibration_out * calibration_out;
  calibration_out = calibration_out / measured_power_out;
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
  calibration_out = 1;
  calibration_in = 1;
  ADC_calibration_out = 900;
  ADC_calibration_in = 100;
  measured_power_out = 900;
  measured_power_in = 100;
  calibration_ID = 1.0;
  graph_maxWatt = 1500;
  graph_maxTemp = 100;
  protection_po = 1500;
  protection_swr = 3;
  protection_temp = 55;
  T_pepHOLD = 200;
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

void alarm_clear()
{
  alarm_code = 0;
  digitalWrite(RF_IN, LOW);
  digitalWrite(BIAS_ON, LOW);
  digitalWrite(ANT_RXTX, LOW);
  digitalWrite(PROT_OUT, LOW);
  antswr_status = LOW;
  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
  PROT_OUT_status = LOW;
  myNex.writeStr("vis b0,0");
  last_reset_alarm = LOW;
}

// ALARM CLEAR
void trigger21()
{
  // x15
  alarm_clear();
  display_error();
}

/* ========= INTURRUPT ROUTINES =========*/

// Inturupt routine for External triggered protection signal.
void int_extprot()
{
  digitalWrite(RF_IN, LOW);
  digitalWrite(BIAS_ON, LOW);
  digitalWrite(ANT_RXTX, LOW);
  if (alarm_code == 0)
    alarm_code = 1;
  extprot_status = HIGH;
  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
}

// Inturupt routine for high ANT SWR
void int_antswr()
{
  digitalWrite(RF_IN, LOW);
  digitalWrite(BIAS_ON, LOW);
  digitalWrite(ANT_RXTX, LOW);
  digitalWrite(PROT_OUT, HIGH);
  alarm_code = 2;
  antswr_status = HIGH;
  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
  PROT_OUT_status = HIGH;
}

// Inturupt routine for high input power
void int_inpo()
{
  digitalWrite(RF_IN, LOW);
  digitalWrite(BIAS_ON, LOW);
  digitalWrite(ANT_RXTX, LOW);
  digitalWrite(PROT_OUT, HIGH);
  alarm_code = 3;
  inpo_status = HIGH;
  RF_IN_status = LOW;
  BIAS_ON_status = LOW;
  ANT_RXTX_status = LOW;
  PROT_OUT_status = HIGH;
}

// Inturupt routine for high LPF SWR
void int_lpfswr()
{
  digitalWrite(RF_IN, LOW);
  digitalWrite(BIAS_ON, LOW);
  digitalWrite(ANT_RXTX, LOW);
  digitalWrite(PROT_OUT, HIGH);
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
    digitalWrite(BIAS_ON, HIGH);
    digitalWrite(ANT_RXTX, HIGH);
    digitalWrite(RF_IN, HIGH);
    BIAS_ON_status = HIGH;
    ANT_RXTX_status = HIGH;
    RF_IN_status = HIGH;
  }
  else
  {
    digitalWrite(RF_IN, LOW);
    digitalWrite(BIAS_ON, LOW);
    digitalWrite(ANT_RXTX, LOW);
    RF_IN_status = LOW;
    BIAS_ON_status = LOW;
    ANT_RXTX_status = LOW;
  }
}

void reset_alarm_button()
{
  if (last_reset_alarm == LOW)
  {
    myNex.writeStr("vis b0,1");
    last_reset_alarm = HIGH;
  }
}

void homepage_normalrun()
{
  display_error();
  band_selection();
  display_band(current_band);
  display_lpf(current_lpf);
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
  reset_alarm_button();
}

void menu_page()
{
  // nothing to do in this page
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

    // Show in mV at controller input, to see ADC values please comment this section and use next one below
    myNex.writeNum("dn2.val", ant_raw_fwd * ref_ADC * 1000);
    myNex.writeNum("dn3.val", ant_raw_ref * ref_ADC * 1000);
    myNex.writeNum("dn4.val", lpf_raw_fwd * ref_ADC * 1000);
    myNex.writeNum("dn5.val", lpf_raw_ref * ref_ADC * 1000);
    myNex.writeNum("dn6.val", in_raw_fwd * ref_ADC * 1000);
    myNex.writeNum("dn7.val", in_raw_ref * ref_ADC * 1000);

    /*
    // Show ADC values of controller input
    myNex.writeNum("dn2.val", ant_raw_fwd);
    myNex.writeNum("dn3.val", ant_raw_ref);
    myNex.writeNum("dn4.val", lpf_raw_fwd);
    myNex.writeNum("dn5.val", lpf_raw_ref);
    myNex.writeNum("dn6.val", in_raw_fwd);
    myNex.writeNum("dn7.val", in_raw_ref);
    */

    fanspeed(); // Set fan speed and display
    myNex.writeNum("dn9.val", V_now() * ResV);
    myNex.writeNum("dn9.val", I_now());

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
  if (calibpage_load == false)
  {
    myNex.writeNum("msn3.val", ADC_calibration_out);
    myNex.writeNum("msn4.val", measured_power_out);

    myNex.writeNum("msn0.val", ADC_calibration_in);
    myNex.writeNum("msn1.val", measured_power_in);
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
    ADC_maxCalibV_out = ant_fwd_now() / ref_ADC;
    ADC_maxCalibV_in = in_fwd_now() / ref_ADC;

    if (ADC_maxCalibV_out > ADC_calibration_out)
      ADC_calibration_out = ADC_maxCalibV_out;

    if (ADC_maxCalibV_in > ADC_calibration_in)
      ADC_calibration_in = ADC_maxCalibV_in;

    if (last_calib_out != ADC_calibration_out)
    {
      myNex.writeNum("mcn3.val", ADC_calibration_out);
      last_calib_out = ADC_calibration_out;
    }
    if (last_calib_in != ADC_calibration_in)
    {
      myNex.writeNum("mcn0.val", ADC_calibration_in);
      last_calib_in = ADC_calibration_in;
    }
  }

  calib_ID = I_now() * 1000;
  if (calib_ID > peak_ID)
    peak_ID = calib_ID;

  if (last_peak_ID != peak_ID)
  {
    myNex.writeNum("mcn6.val", peak_ID);
    last_peak_ID = peak_ID;
  }

  delay(50);
}

/* ======================= Band Calibration page data load ======================= */
void bandcalib_page()
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

void trigger13()
{ // x0D
  TX_Enable = false;
  bandcalib_page();
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
  delay(100);
  // Serial.begin(9600);
  // if(debug) Serial.println("*** SETUP START ***");
  myNex.begin(115200); // start Nextion Display

  PCF.begin();
  Serial.println(PCF.isConnected());
  Serial.println();

  // PWMsetup();
  analogWriteFrequency(FAN_1, PWM_FREQ_KHZ * 1000); // Teensy pin changes to 25 kHz
  analogWriteFrequency(FAN_2, PWM_FREQ_KHZ * 1000); // Teensy pin changes to 25 kHz

  // TEENSY INTARRUPT

  // Attach inturupt for external protection detection
  pinMode(INT_PROT, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(INT_PROT), int_extprot, RISING);

  // Attach inturupt for high swr
  pinMode(INT_ANT, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(INT_ANT), int_antswr, HIGH);

  // Attach inturupt for high input
  pinMode(INT_IN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(INT_IN), int_inpo, RISING);

  // Attach inturupt for high LPF swr error
  pinMode(INT_LPF, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(INT_LPF), int_lpfswr, RISING);

  // Attach inturupt for PTT
  pinMode(PTT_SENSE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PTT_SENSE), int_PTT_CHANGE, CHANGE);

  pinMode(BAND_A, INPUT_PULLUP);
  pinMode(BAND_B, INPUT_PULLUP);
  pinMode(BAND_C, INPUT_PULLUP);
  pinMode(BAND_D, INPUT_PULLUP);

  pinMode(BIAS_ON, OUTPUT);

  EEPROM.get(250, default_value);
  if (default_value != 21)
    default_write();
  // default_write();

  loadEEPROM();

  /* ==== Temp sensor setup ==== */
  if (dtemp1_status)
  {
    sensors.begin(); // Temp sensor
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

  // myNex.writeStr("vis d1,0");
  digitalWrite(BIAS_ON, LOW);
  myNex.writeStr("Er1.txt", error_text);
  // if(debug) Serial.println("*** SETUP END ***");

  //delay(2000);

  // band_selection();

  Serial.print("band:");
  Serial.println(current_band);
  Serial.print("LPF:");
  Serial.println(current_lpf);
  display_band(current_band);
  display_lpf(current_lpf);

  display_volt(); // Read Volt and Display
  display_ID();   // Read current
  read_temp();    // Read Temp and display
  fanspeed();
}

void loop()
{
  // if(debug) Serial.println("*** LOOP START ***");
  //  Receive from display if not in TX
  if (PTT_status != HIGH)
  {
    myNex.NextionListen();
    // if(debug) Serial.println("*** NEX LISTEN ***");
  }

  // Home page tasks
  if (display_page == 2)
  {
    if (alarm_code == 0)
    {
      // if(debug) Serial.println("*** HOME NORMAL RUN ***");
      homepage_normalrun();
    }

    else
    {
      // if(debug) Serial.println("*** HOME ALARM RUN ***");
      homepage_alarmrun();
    }
  }

  else if (display_page == 3)
  {
    // if(debug) Serial.println("*** MENU PAGE ***");
    menu_page();
  }
  else if (display_page == 4)
  {
    // if(debug) Serial.println("*** DIAG PAGE ***");
    diagnos_page();
  }
  else if (display_page == 5)
  {
    // if(debug) Serial.println("*** SETTING PAGE ***");
    settings_page();
  }
  else if (display_page == 6)
  {
    // if(debug) Serial.println("*** METER CALIB PAGE ***");
    metercalib_page();
  }
  else if (display_page == 7)
  {
    // if(debug) Serial.println("*** BAND CALIB PAGE ***");
    // bandcalib_page();
  }
  else if (display_page == 8)
  {
    // if(debug) Serial.println("*** REMOTE PAGE ***");
    remote_page();
  }
  else if (display_page == 9)
  {
    // if(debug) Serial.println("*** INFO PAGE ***");
    info_page();
  }
  // delay(1000);
}