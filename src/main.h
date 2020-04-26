#include "Arduino.h"
#include "EEPROM.h"

// I/O
#define ECA_INPUT                 A4
#define O2_OUTPUT                 10

// EEPROM
#define EEPROM_O2_PWM_ADDRESS     0

// constants
#define PWM_RESOLUTION            255.0f
#define OUTPUT_REF_VOLTAGE        5.0f
#define FILTER_ALPHA              0.01f
#define FREQUENCY_ALPHA           0.01f
#define MIN_PERIOD                5.0f    // ms (200 Hz)

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

#define SAVE_INTERVAL_MS          300000  // 5 minutes in milliseconds

#define LOG_ETHANOL
#define LOG_OUTPUT

bool previouslyStored = true;
float tempOutputVoltage = 0.5, outputVoltage = 0.5, ethanol = 0;
float outputPWM = DEFAULT_OUTPUT_PWM;

// frequency readings
float frequency = 0.f, dutyCycle = 0.f;
double sum = 0;
uint8_t count = 0;
bool increment = false;

uint32_t pulseHigh, pulseLow, pulseTotal;

const float frequencyScaler = ETHANOL_FREQUENCY_SCALER;
const float voltageScaler = ETHANOL_VOLTAGE_SCALER;
uint32_t lastSave = millis();

volatile uint16_t ticks;

float loadVoltagePWM();
void saveVoltagePWM(float value);

void frequencyToEthanolContent(float frequency, float scaler);
void calculateOutput();

void onRise();
void onFall();

uint32_t fall = 0, lastFall = 0, rise = 0;