#include "main.h"

void setup()
{
  Serial.begin(115200);
  while (!Serial) { yield; }
  pinMode(O2_OUTPUT, OUTPUT);
  pinMode(ECA_INPUT, INPUT);

  // load last stored PWM value for output
  outputPWM = loadVoltagePWM();
  analogWrite(O2_OUTPUT, outputPWM);

  attachInterrupt(ECA_INPUT, onRise, RISING);
}

void loop()
{
  if (fall > rise && lastFall != 0) {
    float period = fall - lastFall;

    // ensure we're getting a valid pulse reading from the ethanol sensor
    // a reading of 180 - 190 Hz = contaminated fuel
    // between 50 Hz - 150 Hz -> ethanol = frequency - 50
    // additionally, you can get the fuel temperature from the duty cycle of the signal
    if (period >=  MIN_PERIOD) {
      Serial.print("Frequency: ");
      Serial.print(1.0f / (period / 1000.0f));

      if (frequency == 0)
        frequency = 1.0f / (period / 1000.0f);
      else
        frequency = (1 - FREQUENCY_ALPHA) * frequency + FREQUENCY_ALPHA * (1.0f / (period / 1000.0f));

      // get ethanol content from sensor frequency
      frequencyToEthanolContent(frequency, frequencyScaler);

      // calculate the output voltage and PWM values
      calculateOutput();

      // update our output to the O2 sensors
      analogWrite(O2_OUTPUT, (int)outputPWM);

      #if defined(LOG_ETHANOL) or defined(LOG_OUTPUT)
        Serial.println("");
      #endif
    }

    // periodically store last output pwm
    uint32_t now = millis();
    if (!previouslyStored || now - lastSave > SAVE_INTERVAL_MS)
      saveVoltagePWM(outputPWM);
  }
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

#ifdef LOG_ETHANOL
  Serial.print("\tEthanol: ");
  Serial.print(ethanol);
  Serial.print("%");
#endif
}

/**
 *  Calculates output voltage and PWM duty cycle based on ethanol content
**/
void calculateOutput() {
  // calculate current voltage output
  tempOutputVoltage = max(E0_VOLTAGE, min((voltageScaler * ethanol) + E0_VOLTAGE, E100_VOLTAGE));

  // run exponential filter to smooth output
  outputVoltage = (1.0f - FILTER_ALPHA) * outputVoltage  + FILTER_ALPHA * tempOutputVoltage;

  // set output pwm value
  outputPWM = (int)ceil(outputVoltage * (PWM_RESOLUTION / OUTPUT_REF_VOLTAGE));

#ifdef LOG_OUTPUT
  Serial.print("\tOutput (v): ");
  Serial.print(tempOutputVoltage);
  Serial.print("\tFiltered Output (v): ");
  Serial.print(outputVoltage);
  Serial.print("\tPWM: ");
  Serial.print(outputPWM);
#endif
}