/*
 * LiPo Balancer Top balance BMS control software:
 *  - Every 8 seconds wake up from sleep and take ADC readings from each channel of the ADS1115 16 bit ADCs.
 *  - Keep a moving average of the last few readings and calculate the cell voltages
 *  - Top balance the cells:
 *    # If any cell is > 0.02 volts from the minimum cell and the pack is above 24v then discharge the higher cells
 *  
 *  - If any 1 cell leaves the 3.00 - 4.25volt range then do a slightly different flash pattern using the LED
 *  
 *  The BMS draws about 500uA average current. It flashes an LED every 8 seconds to indicate that it is alive.
 */

// **** INCLUDES *****
#include "LowPower.h"
#include "Average.h"
#include <Wire.h>
#include <Adafruit_ADS1015.h>

// **** DEFINES ******
#define DC_CELL_0_PIN                     A3
#define DC_CELL_1_PIN                     A2
#define DC_CELL_2_PIN                     A1
#define DC_CELL_3_PIN                     A0
#define DC_CELL_4_PIN                     13
#define DC_CELL_5_PIN                     12
#define LED_PIN                           9

#define CELL_0_V_CONST                    0.99069         // These are calibration constants which I calculated by apply voltages to pins and seeing what ADC readings they gave
#define CELL_1_V_CONST                    0.99790  
#define CELL_2_V_CONST                    0.99840 
#define CELL_3_V_CONST                    1.00463  
#define CELL_4_V_CONST                    1.00171  
#define CELL_5_V_CONST                    1.00249  

#define MIN_CELL_VOLTAGE                  3000            // The Warning LED flash starts when any cell dips below this voltage
#define MAX_CELL_VOLTAGE                  4250            // Likewise if a cell goes above this

#define BALANCE_PACK_VOLTAGE_KICK_IN_MV   24000           // Don't do any balancing unless the pack is above this voltage. We aim to top balance
#define START_BALANCING_THRESHOLD_MV      20              // Only balance if cells are mismatched more than this voltage

#define FILTER_LENGTH                     12              // Length of the moving average filter in which cell_voltages are filtered. 12 @ 8secs/reading = 96 second refresh

//#define DEBUG                                           // If this is commented out then the BMS uses a slightly smaller amount of power. 

// **** Function declarations *****
void Read_Voltages();
void Check_Cell_ranges();
void Stop_Discharges();
void Set_Discharges();
void Print_Cell_Voltages();
void Initialise_Avg_Filters();

// **** Variables *****
int Pack_mVoltage;
int Cell_mVoltage[6];
float Cell_ADC_Readings[6];

byte Cell_Warning_Flags;
byte Cell_Discharge_Flags;

Adafruit_ADS1115 ads1(0x48);
Adafruit_ADS1115 ads2(0x49);

Average          Average0(FILTER_LENGTH);
Average          Average1(FILTER_LENGTH);
Average          Average2(FILTER_LENGTH);
Average          Average3(FILTER_LENGTH);
Average          Average4(FILTER_LENGTH);
Average          Average5(FILTER_LENGTH);
Average          Average6(FILTER_LENGTH);                                                                                // This one is for the pack voltage

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void setup(void) 
{
  #ifdef DEBUG
    // Start USART
    Serial.begin(115200);
    Serial.println("Starting Up...");
    delay(50);
  #endif

  //Configure pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(DC_CELL_0_PIN, OUTPUT);
  pinMode(DC_CELL_1_PIN, OUTPUT);
  pinMode(DC_CELL_2_PIN, OUTPUT);
  pinMode(DC_CELL_3_PIN, OUTPUT);
  pinMode(DC_CELL_4_PIN, OUTPUT);
  pinMode(DC_CELL_5_PIN, OUTPUT);
    
  digitalWrite(LED_PIN, HIGH);                                                                                          // LED is active LOW
  Stop_Discharges();  

  // Start the ADC drivers
  ads1.setGain(GAIN_ONE);
  ads1.begin();

  ads2.setGain(GAIN_ONE);
  ads2.begin();

  Initialise_Avg_Filters();
}

void loop(void) 
{
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);                                                                        // This enters a very low power mode

  Stop_Discharges();                                                                                                    // Don't want cell discharging to effect ADC readings                                   
  delay(1);                                                                                                             // Allow things to settle before reading ADCs
  
  Read_Voltages();                                                                                                      // Read all the ADC channels once
  digitalWrite(LED_PIN, LOW);                                                                                           // Flash indicator LED to show system is active
  Check_Cell_ranges();                                                                                                  // If any cell out of range, warn of that  
  Set_Discharges();                                                                                                     // Start any cells >4.15v discharging 

  #ifdef DEBUG
    Print_Cell_Voltages();
    delay(60);                                                                                                          // Data needs time to export
  #else
    LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  #endif 
    
  digitalWrite(LED_PIN, HIGH);                                                                                          // Turn LED off before going low power again

  if(Cell_Warning_Flags)  {
    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
    digitalWrite(LED_PIN, LOW);
    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
    digitalWrite(LED_PIN, HIGH);
  }  
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Read_Voltages()
{
  // Read the ADCs to calculate the cell voltages. Use an average filter so that the CellVoltages are averaged over the last 10 readings.
    
  Cell_ADC_Readings[0] = (float)ads1.readADC_SingleEnded(3) * CELL_0_V_CONST;
  Cell_ADC_Readings[1] = (float)ads1.readADC_SingleEnded(2) * CELL_1_V_CONST;
  Cell_ADC_Readings[2] = (float)ads2.readADC_SingleEnded(3) * CELL_2_V_CONST;
  Cell_ADC_Readings[3] = (float)ads2.readADC_SingleEnded(2) * CELL_3_V_CONST;
  Cell_ADC_Readings[4] = (float)ads2.readADC_SingleEnded(1) * CELL_4_V_CONST;
  Cell_ADC_Readings[5] = (float)ads2.readADC_SingleEnded(0) * CELL_5_V_CONST; 

  Pack_mVoltage = Average6.Rolling_Average(Cell_ADC_Readings[5]);
  
  Cell_mVoltage[0] = (int)Cell_ADC_Readings[0];
  Cell_mVoltage[1] = (int)(Cell_ADC_Readings[1] - Cell_ADC_Readings[0]);
  Cell_mVoltage[2] = (int)(Cell_ADC_Readings[2] - Cell_ADC_Readings[1]);
  Cell_mVoltage[3] = (int)(Cell_ADC_Readings[3] - Cell_ADC_Readings[2]);
  Cell_mVoltage[4] = (int)(Cell_ADC_Readings[4] - Cell_ADC_Readings[3]);
  Cell_mVoltage[5] = (int)(Cell_ADC_Readings[5] - Cell_ADC_Readings[4]);

  Cell_mVoltage[0] = Average0.Rolling_Average(Cell_mVoltage[0]);
  Cell_mVoltage[1] = Average1.Rolling_Average(Cell_mVoltage[1]);
  Cell_mVoltage[2] = Average2.Rolling_Average(Cell_mVoltage[2]);
  Cell_mVoltage[3] = Average3.Rolling_Average(Cell_mVoltage[3]);
  Cell_mVoltage[4] = Average4.Rolling_Average(Cell_mVoltage[4]);
  Cell_mVoltage[5] = Average5.Rolling_Average(Cell_mVoltage[5]);
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Check_Cell_ranges()
{
  // If any cell deviates from the 3.00v - 4.21v range then indicate this using the LED
  Cell_Warning_Flags = 0;
  
  if(Cell_mVoltage[0] > MAX_CELL_VOLTAGE || Cell_mVoltage[0] < MIN_CELL_VOLTAGE) {
    Cell_Warning_Flags |= 0b10000001;
  }
  if(Cell_mVoltage[1] > MAX_CELL_VOLTAGE || Cell_mVoltage[1] < MIN_CELL_VOLTAGE) {
    Cell_Warning_Flags |= 0b10000010;
  }
  if(Cell_mVoltage[2] > MAX_CELL_VOLTAGE || Cell_mVoltage[2] < MIN_CELL_VOLTAGE) {
    Cell_Warning_Flags |= 0b10000100;
  }
  if(Cell_mVoltage[3] > MAX_CELL_VOLTAGE || Cell_mVoltage[3] < MIN_CELL_VOLTAGE) {
    Cell_Warning_Flags |= 0b10001000;
  }
  if(Cell_mVoltage[4] > MAX_CELL_VOLTAGE || Cell_mVoltage[4] < MIN_CELL_VOLTAGE) {
    Cell_Warning_Flags |= 0b10010000;
  }
  if(Cell_mVoltage[5] > MAX_CELL_VOLTAGE || Cell_mVoltage[5] < MIN_CELL_VOLTAGE) {
    Cell_Warning_Flags |= 0b10100000;
  }  
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Set_Discharges()
{
  Cell_Discharge_Flags = 0;
  
  // 1) identify the cell with the lowest voltage
  int Minimum_Cell_mV = Cell_mVoltage[0];
  int Minimum_Cell_Index = 0;
  
  for (int i=1; i<6; i++) 
  {
     if (Cell_mVoltage[i] < Minimum_Cell_mV)
     {
         Minimum_Cell_mV = Cell_mVoltage[i]; 
         Minimum_Cell_Index = i;
      } 
  }

  // 2) identify the cell with the highest voltage
  int Maximum_Cell_mV = Cell_mVoltage[0];
  int Maximum_Cell_Index = 0;
  
  for (int i=1; i<6; i++) 
  {
     if (Cell_mVoltage[i] > Maximum_Cell_mV)
     {
         Maximum_Cell_mV = Cell_mVoltage[i]; 
         Maximum_Cell_Index = i;
      } 
  }

  // 3) if the difference between min and max cell voltages is < than START_BALANCING_THRESHOLD_MV then do nothing
  if(Maximum_Cell_mV - Minimum_Cell_mV < START_BALANCING_THRESHOLD_MV)  {
    return;
  }

  // 4) Otherwise if battery > BALANCE_PACK_VOLTAGE_KICK_IN_MV then discharge all cells which are > START_BALANCING_THRESHOLD_MV from the lowest cell
  if(Pack_mVoltage < BALANCE_PACK_VOLTAGE_KICK_IN_MV) {
    return;
  }  

  if(Cell_mVoltage[0] - Minimum_Cell_mV > START_BALANCING_THRESHOLD_MV) {
    digitalWrite(DC_CELL_0_PIN, HIGH);
    Cell_Discharge_Flags |= 0b10000001;
  }
  if(Cell_mVoltage[1] - Minimum_Cell_mV > START_BALANCING_THRESHOLD_MV) {
    digitalWrite(DC_CELL_1_PIN, HIGH);
    Cell_Discharge_Flags |= 0b10000010;
  }
  if(Cell_mVoltage[2] - Minimum_Cell_mV > START_BALANCING_THRESHOLD_MV) {
    digitalWrite(DC_CELL_2_PIN, HIGH);
    Cell_Discharge_Flags |= 0b10000100;
  }
  if(Cell_mVoltage[3] - Minimum_Cell_mV > START_BALANCING_THRESHOLD_MV) {
    digitalWrite(DC_CELL_3_PIN, HIGH);
    Cell_Discharge_Flags |= 0b10001000;
  }
  if(Cell_mVoltage[4] - Minimum_Cell_mV > START_BALANCING_THRESHOLD_MV) {
    digitalWrite(DC_CELL_4_PIN, HIGH);
    Cell_Discharge_Flags |= 0b10010000;
  }
  if(Cell_mVoltage[5] - Minimum_Cell_mV > START_BALANCING_THRESHOLD_MV) {
    digitalWrite(DC_CELL_5_PIN, HIGH);
    Cell_Discharge_Flags |= 0b10100000;
  }  
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Stop_Discharges()
{
  // Ensure no cell discharging 
  digitalWrite(DC_CELL_0_PIN, LOW);
  digitalWrite(DC_CELL_1_PIN, LOW);
  digitalWrite(DC_CELL_2_PIN, LOW);
  digitalWrite(DC_CELL_3_PIN, LOW);
  digitalWrite(DC_CELL_4_PIN, LOW);
  digitalWrite(DC_CELL_5_PIN, LOW);
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Print_Cell_Voltages()
{
  // Display to Debug   
  Serial.print("C1: "); Serial.print(Cell_mVoltage[0]);
  Serial.print(", C2: "); Serial.print(Cell_mVoltage[1]);
  Serial.print(", C3: "); Serial.print(Cell_mVoltage[2]);
  Serial.print(", C4: "); Serial.print(Cell_mVoltage[3]);
  Serial.print(", C5: "); Serial.print(Cell_mVoltage[4]);
  Serial.print(", C6: "); Serial.print(Cell_mVoltage[5]);
  Serial.print(", Pack: "); Serial.println(Pack_mVoltage);

  if(Cell_Warning_Flags)  {
    Serial.print("WARNING - CELL OUT OF RANGE: "); Serial.println(Cell_Warning_Flags, BIN);      
  }

  if(Cell_Discharge_Flags)  {
    Serial.print("D/C Cells: "); Serial.println(Cell_Discharge_Flags, BIN);      
  }  
}

// ######################################################################################################################
// ----------------------------------------------------------------------------------------------------------------------
// ######################################################################################################################
void Initialise_Avg_Filters()
{
  for(int i = 0; i < FILTER_LENGTH; i++) {
    Average0.Rolling_Average(4000);
    Average1.Rolling_Average(4000);
    Average2.Rolling_Average(4000);
    Average3.Rolling_Average(4000);
    Average4.Rolling_Average(4000);
    Average5.Rolling_Average(4000); 

    Average6.Rolling_Average(24000);
  }
}




