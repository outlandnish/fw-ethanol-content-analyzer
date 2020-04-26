#include "main.h"

void setup()
{
  #ifdef LOGGING_ENABLED
    Serial.begin(115200);
    while (!Serial) { yield; }
  #endif

  pinMode(ETHANOL_OUTPUT, OUTPUT);
  pinMode(ECA_INPUT, INPUT);

  // load last stored PWM value for output
  outputPWM = loadVoltagePWM();
  analogWrite(ETHANOL_OUTPUT, outputPWM);

  attachInterrupt(ECA_INPUT, onRise, RISING);
}

void loop()
{
  // get ethanol content from sensor frequency
  bool valid = calculateFrequency();
  if (valid)
    frequencyToEthanolContent(frequency, frequencyScaler);

  // calculate the output voltage and PWM values
  calculateOutput();

  // update our output to the O2 sensors
  analogWrite(ETHANOL_OUTPUT, (int)outputPWM);

  // periodically store last output pwm
  uint32_t now = millis();
  if (!previouslyStored || now - lastSave > SAVE_INTERVAL_MS)
    saveVoltagePWM(outputPWM); 

  #ifdef LOG_FREQUENCY
    Serial.print("Frequency: ");
    Serial.print(frequency);
  #endif

  #ifdef LOG_ETHANOL
    Serial.print("\tEthanol: ");
    Serial.print(ethanol);
    Serial.print("%");
  #endif

  #ifdef LOG_OUTPUT
    Serial.print("\tOutput (v): ");
    Serial.print(tempOutputVoltage);
    Serial.print("\tFiltered Output (v): ");
    Serial.print(outputVoltage);
    Serial.print("\tPWM: ");
    Serial.print(outputPWM);
  #endif

  #ifdef LOGGING_ENABLED
    Serial.println("");
  #endif
}

// Interrupt Service Routine (ISR) to store the last time we saw the signal rise from low to high
void onRise() {
  rise = millis();
  attachInterrupt(ECA_INPUT, onFall, FALLING);
}

// Interrupt Service Routine (ISR) to store the last time we saw the signal fall from high to low
void onFall() {
  lastFall = fall;
  fall = millis();
  attachInterrupt(ECA_INPUT, onRise, RISING);
}

// Loads the last output PWM value from EEPROM
float loadVoltagePWM() {
  float pwm;
  EEPROM.get(EEPROM_O2_PWM_ADDRESS, pwm);

  // use default if invalid value in EEPROM
  if (isnan(pwm)) {
    previouslyStored = false;
    Serial.println("Invalid PWM in EEPROM. Using default value");
    pwm = DEFAULT_OUTPUT_PWM;
  }

  Serial.print("Loaded PWM value: ");
  Serial.println(pwm);

  return pwm;  
}

/** Saves an output PWM to EEPROM
* @param value - PWM duty cycle to store
**/
void saveVoltagePWM(float value) {
  EEPROM.put(EEPROM_O2_PWM_ADDRESS, value);
  previouslyStored = true;
  Serial.print("Saved PWM value: ");
  Serial.println(value);
}

/**
 * Calculates frequency by measuring time between falling edges of the
 * ethanol sensor input
 * @return boolean - true if valid frequency
**/
bool calculateFrequency() {
  if (fall < rise || lastFall == 0)
    return false;
  
  // ensure the period is valid
  // > 200 Hz is invalid    
  float period = fall - lastFall;
  if (period < MIN_PERIOD)
    return false;

  float tempFrequency = 1.0f / (period / 1000.0f);

  // if we haven't calculated frequency, use the current frequency. 
  // Otherwise, run it through an exponential filter to smooth readings
  if (frequency == 0)
    frequency = tempFrequency;
  else
    frequency = (1 - FREQUENCY_ALPHA) * frequency + FREQUENCY_ALPHA * tempFrequency;

  return true;
}

/** 
 * Converts a sensor frequency to ethanol percentage 
 * Ethanol % = Frequency (in Hz) - 50.0.
 * A value of 180 Hz - 190 Hz indicates contaminated fuel.
 * @param frequency - Input frequency (1 / period)
 * @param scaler - Value by which interpolate frequencies between E0 and E100
**/
void frequencyToEthanolContent(float frequency, float scaler) {
  ethanol = (frequency - E0_FREQUENCY) / scaler;

  // bound ethanol content by a max of 0 to 100
  ethanol = max(0.0f, min(100.0f, ethanol));
}

/**
 *  Calculates output voltage and PWM duty cycle based on ethanol content
**/
void calculateOutput() {
  // calculate current voltage output
  tempOutputVoltage = max(E0_VOLTAGE, min((voltageScaler * ethanol) + E0_VOLTAGE, E100_VOLTAGE));

  // run exponential filter to smooth output
  outputVoltage = (1.0f - OUTPUT_ALPHA) * outputVoltage  + OUTPUT_ALPHA * tempOutputVoltage;

  // set output pwm value
  outputPWM = (int)ceil(outputVoltage * (PWM_RESOLUTION / OUTPUT_REF_VOLTAGE));
}