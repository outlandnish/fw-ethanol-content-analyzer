#include "main.h"

void setup()
{
  #ifdef LOGGING_ENABLED
    Serial.begin(115200);
    while (!Serial) { yield; }
  #endif

  pinMode(ETHANOL_OUTPUT, OUTPUT);
  // pinMode(ECA_INPUT, INPUT);

  // load last stored PWM value for output
  outputPWM = loadVoltagePWM();
  analogWrite(ETHANOL_OUTPUT, outputPWM);

  // attachInterrupt(ECA_INPUT, onRise, RISING);
  setupTimer();

  sei();
}

void loop()
{
  // get ethanol content from sensor frequency
  if (newData && calculateFrequency()) {
    frequencyToEthanolContent(frequency, frequencyScaler);
    newData = false;
  }

  // calculate the output voltage and PWM values
  calculateOutput();

  // update our output to the O2 sensors
  analogWrite(ETHANOL_OUTPUT, (int)outputPWM);

  // periodically store last output pwm
  uint32_t now = millis();
  if (!previouslyStored || now - lastSave > SAVE_INTERVAL_MS)
    saveVoltagePWM(outputPWM); 

  #ifdef LOG_RAW_ECA
    Serial.print("Period:");
    Serial.print(period);
  #endif

  #ifdef LOG_FREQUENCY
    Serial.print("\tFrequency: ");
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

/*
  Feeds the data from the Ethanol Content Analyzer into the Analog Comparator
  on the Atmega4809. The signal is compared to a voltage reference from the internal
  DAC to see when the signal is high / low. The output from the Analog Comparator is fed
  into timer TCB0 which counts the time between the positive and negative edges of the signal
*/
void setupTimer() {
  // PD4
  PORTD.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;

  // Use internal voltage reference @ 1.5V
  VREF.CTRLA = VREF_AC0REFSEL_1V5_gc;
  // AC0 DACREF reference enable
  VREF.CTRLB = VREF_AC0REFEN_bm;
  
  // set DAC voltage reference
  AC0.DACREF = DACREF_VALUE;
  // AC Pin 1 as positive input, DAC Voltage Reference as negative input
  AC0.MUXCTRLA = AC_MUXPOS_PIN1_gc | AC_MUXNEG_DACREF_gc;
  // Enable analog comaparator, output buffer enabled
  AC0.CTRLA = AC_ENABLE_bm | AC_OUTEN_bm;
  // Analog Comparator 0 Interrupt Disabled
  AC0.INTCTRL = 0;

  // setup AC output as event generator for channel 0
  EVSYS.CHANNEL0 = EVSYS_GENERATOR_AC0_OUT_gc;
  // Connect user to event channel 0
  EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;

  // Configure TCB in Input Capture Frequency Mode
  TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;
  // Enable Capture or Timeout interrupt
  TCB0.INTCTRL = TCB_CAPT_bm;
  // Enable Input Capture event
  TCB0.EVCTRL = TCB_CAPTEI_bm;

  // enable TCB and use clock from TCA
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm | TCB_RUNSTDBY_bm;
}

// Loads the last output PWM value from EEPROM
float loadVoltagePWM() {
  float pwm;
  EEPROM.get(EEPROM_O2_PWM_ADDRESS, pwm);

  // use default if invalid value in EEPROM
  if (isnan(pwm)) {
    previouslyStored = false;
    pwm = DEFAULT_OUTPUT_PWM;

#ifdef LOGGING_ENABLED
    Serial.println("Invalid PWM in EEPROM. Using default value");
#endif
  }

#ifdef LOGGING_ENABLED
  Serial.print("Loaded PWM value: ");
  Serial.println(pwm);
#endif

  return pwm;  
}

/** Saves an output PWM to EEPROM
* @param value - PWM duty cycle to store
**/
void saveVoltagePWM(float value) {
  EEPROM.put(EEPROM_O2_PWM_ADDRESS, value);
  previouslyStored = true;

#ifdef LOGGING_ENABLED
  Serial.print("Saved PWM value: ");
  Serial.println(value);
#endif

  lastSave = millis();
}

/**
 * Calculates frequency using the output from the AC Comparator + TCB Counter
 * ethanol sensor input
 * @return boolean - true if valid frequency
**/
bool calculateFrequency() {
  float tempFrequency = 1.f / ((float) period * (1.f / 256000.f));

  if (tempFrequency < 0 || tempFrequency > MAX_FREQUENCY)
    return false;

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

ISR(TCB0_INT_vect) {
  period = TCB0.CCMP;
  newData = true;
}