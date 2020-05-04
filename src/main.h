#include "Arduino.h"
#include "EEPROM.h"

// I/O
#define ECA_INPUT                 A6
#define ETHANOL_OUTPUT            10

// EEPROM
#define EEPROM_O2_PWM_ADDRESS     0

// constants
#define PWM_RESOLUTION            255.0f
#define OUTPUT_REF_VOLTAGE        5.0f
#define FREQUENCY_ALPHA           0.01f
#define OUTPUT_ALPHA              0.01f
#define MAX_FREQUENCY             200.f   // Hz

#define E0_FREQUENCY              50.f    // Hz
#define E100_FREQUENCY            150.f   // Hz
#define DEFAULT_FREQUENCY         60      // Hz (E10 is typical in the US for pump gas)
#define ETHANOL_FREQUENCY_SCALER  (E100_FREQUENCY - E0_FREQUENCY) / 100.0f

#define E0_VOLTAGE                0.5f    // volts
#define E100_VOLTAGE              4.5f    // volts
#define ETHANOL_VOLTAGE_SCALER    (E100_VOLTAGE - E0_VOLTAGE) / 100.0f

#define DEFAULT_OUTPUT_PWM        45.9f   // E10 is typical in the US for pump gas
#define E0_PWM                    25.5f   // duty cycle
#define E100_PWM                  229.5f  // duty cycle

#define SAVE_INTERVAL_MS          60000   // 1 minutes in milliseconds

#define LOG_FREQUENCY
#define LOG_ETHANOL
#define LOG_OUTPUT
#define LOG_RAW_ECA

#if defined(LOG_ETHANOL) or defined(LOG_OUTPUT) or defined(LOG_FREQUENCY) or defined(LOG_RAW_ECA)
#define LOGGING_ENABLED
#endif

// DAC ref used for negative input in the comparator
// 0.8 * 1.5V = 1.2
// scaled to 256 => 204
#define DACREF_VALUE (0.8 * 256 / 1.5)

/*
  AC Comparator Pins
  A2/D16 = PD1 = AIN1 = P3
  A1/D15 = PD2 = AIN2 = P0
  A0/D14 = PD3 = AIN3 = N0
  A6/D20 = PD4 = AIN4 = P1
  A7/D21 = PD5 = AIN5 = N1
  PD6 = AIN6 = P2
  AREF = PD7 = AIN7 = N2

  we're using A6 (AC Positive Input 1)
*/

bool previouslyStored = true;
float tempOutputVoltage = 0.5, outputVoltage = 0.5, ethanol = 0;
float outputPWM = DEFAULT_OUTPUT_PWM;

// frequency readings
float frequency = 0.f, dutyCycle = 0.f;
bool increment = false;

const float frequencyScaler = ETHANOL_FREQUENCY_SCALER;
const float voltageScaler = ETHANOL_VOLTAGE_SCALER;
uint32_t lastSave = millis();

volatile uint16_t ticks;

float loadVoltagePWM();
void saveVoltagePWM(float value);

bool calculateFrequency();
void frequencyToEthanolContent(float frequency, float scaler);
void calculateOutput();

void setupTimer();
uint32_t period = 0;
bool newData = false;