/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/Paul/Documents/ParticleWS/MyProjects/Vehicle_Sensor/src/Vehicle_Sensor.ino"
/*
 * Project Vehicle_Sensor
 * Description:
 * Author:
 * Date:
 */

#include <Adafruit_FXOS8700.h>
#include "math.h"

void int1_isr(void);
void enableFXOS8700Interrupts();
void displaySensorDetails(void);
void displayEventData(bool magOnly);
void setup(void);
void loop(void);
#line 11 "c:/Users/Paul/Documents/ParticleWS/MyProjects/Vehicle_Sensor/src/Vehicle_Sensor.ino"
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler(LOG_LEVEL_NONE, {   // Logging level for system messages (LOG_LEVEL_ALL for all OS messages)
    { "app", LOG_LEVEL_INFO },                  // Logging level for application messages
    { "app.fxs", LOG_LEVEL_INFO },           // Logging level for application messages
});


/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// Global counters to track the number of unused interrupts fired and total interrupts fired
volatile uint32_t int1_fired, int1_total;

sensors_event_t aevent, mevent;

float acc_mvec, mag_mvec;

// Set threshold. 400 x 0.1uT = 40.0uT
uint16_t mag_threshold = 400;

// Initialize reference vector
int16_t refMagX = 0;
int16_t refMagY = 0;
int16_t refMagZ = 0;

// Initialize min/max for hard iron estimate
int16_t magXMax = int16_t( 0x8000 ), magYMax = int16_t( 0x8000 ), magZMax = int16_t( 0x8000 );
int16_t magXMin = int16_t( 0x7FFF ), magYMin = int16_t( 0x7FFF ), magZMin = int16_t( 0x7FFF );

uint32_t rtime;

int int1_prev;

/** Interrupt service routine for FXOS8700 INT1 events. */
void int1_isr(void)
{
  static int led = 0;

  int1_fired++;
  int1_total++;
    
  digitalWrite(D7, led);
  led = (led)?0:1;
}


#define FXOS8700_INT_PIN  D4

void enableFXOS8700Interrupts() {
  // Attach interrupt inputs on the MCU
  // Assume active HIGH interrupt
  attachInterrupt(FXOS8700_INT_PIN, int1_isr, FALLING); // INT1 is active HIGH, Push-pull by default
}



void displaySensorDetails(void) {
  sensor_t accel, mag;

  accelmag.getSensor(&accel, &mag);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(accel.name);
  Serial.print("Driver Ver:   ");
  Serial.println(accel.version);
  Serial.print("Unique ID:    0x");
  Serial.println(accel.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(accel.max_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(accel.min_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(accel.resolution, 8);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(mag.name);
  Serial.print("Driver Ver:   ");
  Serial.println(mag.version);
  Serial.print("Unique ID:    0x");
  Serial.println(mag.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(mag.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(mag.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(mag.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void displayEventData(bool magOnly) {

  if (!magOnly) {
    // Display the accel results (acceleration is measured in m/s^2)
    Log.info("-------------- X ------ Y ------- Z --------- MV ----");
    Log.info("| ACCEL | %8.4f  %8.4f  %8.4f  | %8.4f  |", aevent.acceleration.x, aevent.acceleration.y, aevent.acceleration.z, acc_mvec);
  }
  // Display the mag results (mag data is in uTesla)
  Log.info("-------------- X ------ Y ------- Z --------- MV ----");
  Log.info("| MAG:  | %8.1f  %8.1f  %8.1f  | %8.1f  |", mevent.magnetic.x, mevent.magnetic.y, mevent.magnetic.z, mag_mvec);
  Log.info("-----------------------------------------------------");
 }


void setup(void) {
  Serial.begin(9600);

  /* Wait for the Serial Monitor */
 // while (!Serial) {
 //   delay(1);
 // }

  pinMode(D7, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A4, OUTPUT);

  pinMode(D8, OUTPUT);
  digitalWrite(D8,HIGH);
  delay(2000);

  pinMode(FXOS8700_INT_PIN, INPUT);   // INT1 is already pulled up with a 4.7K externally

  Log.info("---> FXOS8700 Test <---");

  /* Initialise the sensor */
  if (!accelmag.begin()) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Log.info("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1) delay(100);
  }

  //accelmag.dumpFXRegisters("Dump Register BEFORE", false);

  uint32_t startupTime = micros();

  enableFXOS8700Interrupts();

  /* Default initialization values when initialize(true) is used
     - ACCEL_RANGE_2G
     - Reduced noise mode (CTRL_REG1)
     - High resolution (CTRL_REG2)
     - HYBRID mode
     - ODR_100HZ
     - Over-sampling rate MAG_OSR_7 (Over Sampling Rate = 16 @ 100Hz ODR)
  */
  accelmag.initialize(false);

  // Configure M_VECM_THS to set magnetometer vector-magnitude threshold to ~40uT (value = 0x190) and set MSB bit 7 (debounce counter mode
  // to 1, cleared whenever the current vector-magnitude result is below the threshold set in M_VECM_THS
  accelmag.setMagVectorMagnitudeThreshold(mag_threshold);

  // Configure the M_VECM_THS to set magnetometer vector-magnitude count tso 20ms (1 x 20ms)
  accelmag.setMagVectorMagnitudeDbounceCount(2);

  // Set Mag Vec Magnitude reference to initial values (0, 0, 0)
  accelmag.setMagVectorMagnitudeReferenceValue(refMagX, refMagY, refMagZ);

  /*  Configure M_VECM_CFG (register 0x69) to:
      m_vecm_init_cfg = 1, Magnetic vector-magnitude interrupt configuration to INT1
      m_vecm_init_en = 1, Magnetic vector-magnitude interrupt enabled
      m_vecm_wake_en = 0, Magnetic vector-magnitude wake disabled
      m_vecm_en = 1, Enable magnetometer vector-magnitude detection function,
      m_vecm_updm = 1, do NOT update initial reference
      m_vecm_initm = 1, Set initial ref to the data stored in M_VECM_X/Y/Z_INIT,
      m_vecm_ele = 1, Enable event LATCHED interrupt output
  */
  accelmag.setMagVectorMagnitudeConfig(MAG_M_VECM_INT_CFG | MAG_M_VECM_INT_EN | MAG_M_VECM_EN | MAG_M_VECM_UPDM |
              /*MAG_M_VECM_INITM |*/ MAG__VECM_ELE);

  //Configure CTRL_REG4 to enable data ready event interrupt
  accelmag.enableInterrupts(INT_EN_DRDY);

  // Configure CTRL_REG5 to route data ready event interrupt to INT pin
  // NOTE: by default, all interrupts are routed to INT2
  accelmag.setInterruptMapToINT1(INT_CFG_DRDY);

  /* Set in hybrid mode, with hyb_autoinc_mode = 1, reads jump to reg 0x33 after reading 0x06 */
  accelmag.setSensorMode(HYBRID_MODE);

  /* Set the output data rate to 50Hz, default */
  accelmag.setOutputDataRate(ODR_25HZ); // ODR_50HZ

  /* Highest Over Sampling Ratio = 7 > (Over Sampling Rate = 32 @ 100Hz ODR)*/
  accelmag.setMagOversamplingRatio(MAG_OSR_7);

    /* Low Noise & High accelerometer OSR resolution */
  //  setReducedNoiseMode(true);
  //  setAccelWakeModeOSR(HIGH_RESOLUTION);

  // Configure CTRL_REG3 to set interrupt polarity to active high and push-pull configuration
//  accelmag.setInterruptPOL_PP_Config(INT_ACTIVE_HIGH_PP);
//  accelmag.setInterruptPOL_PP_Config(INT_ACTIVE_LOW_OD);

  /* Set accelerometer range (optional, default is 2G) */
  // accelmag.setAccelRange(ACCEL_RANGE_8G);

  /* Set the sensor mode (optional, default is hybrid mode) */
  // accelmag.setSensorMode(ACCEL_ONLY_MODE);

  /* Set the magnetometer's oversampling ratio (optional, default is 7) */
  // accelmag.setMagOversamplingRatio(MAG_OSR_7);

  /* Set the output data rate (optional, default is 100Hz) */
  // accelmag.setOutputDataRate(ODR_400HZ);

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  // accelmag.dumpFXRegisters("Dump Register AFTER", false);

  Log.info("Startup time: %lu", micros()-startupTime);
}


void loop(void) {

  static int mvmLED = 0, drdyLED = 0;

  uint8_t magIntSrc, intSrc;

  if (int1_fired) {

    // Read magnetometer interrupt source register M_INT_SOURCE (0x5E)
    intSrc = accelmag.readInterruptSource();
    magIntSrc = accelmag.readMagInterruptSource();

  //  Log.info("INT1 source: INT_SOURCE:0x%02X, MAG_SOURCE:0x%02X", intSrc, magIntSrc);
  //  Log.info("INT1 MAG_SOURCE:0x%02X, int1_fired:%lu", magIntSrc, int1_fired);

    if (magIntSrc & MAG_INT_SRC_M_VECM) {

      digitalWrite(A5, mvmLED);
      mvmLED = (mvmLED)?0:1;

#if 0
      // This a Mag Vec Magnitude interrupt
      // Reset hard iron estimate
		  magXMax = magYMax = magZMax = int16_t( 0x8000 );
		  magXMin = magYMin = magZMin = int16_t( 0x7FFF );

		  Log.info( "Mag Vec Magnitude interrupt - reset hard iron estimate");
#endif
    }
    // Read interrupt source register INT_SOURCE (0x0C)
    if (intSrc & INT_SRC_DRDY) {
      // This a Data Ready interrupt
      accelmag.getEvent(&aevent, &mevent);
      mag_mvec = sqrt(sq(mevent.magnetic.x - refMagX) + sq(mevent.magnetic.y - refMagY) + sq(mevent.magnetic.z - refMagZ));

      // Display only the mag data
//      displayEventData(true);
      
      if ((uint16_t)mag_mvec > mag_threshold) {
        digitalWrite(A4, drdyLED);
        drdyLED = (drdyLED)?0:1;

        //Log.info("Mag Vec Mag interrupt should have triggered (magIntSrc: 0x%02X)", magIntSrc);
      }
#if 0
      // Finally calculate new hard iron estimate and put in reference registers
      if (mevent.magnetic.x < magXMin) magXMin = mevent.magnetic.x;
      if (mevent.magnetic.y < magYMin) magYMin = mevent.magnetic.y;
      if (mevent.magnetic.z < magZMin) magZMin = mevent.magnetic.z;
      if (mevent.magnetic.x > magXMax) magXMax = mevent.magnetic.x;
      if (mevent.magnetic.y > magYMax) magYMax = mevent.magnetic.y;
      if (mevent.magnetic.z > magZMax) magZMax = mevent.magnetic.z;

      refMagX = (magXMax + magXMin) / 2;
      refMagY = (magYMax + magYMin) / 2;
      refMagZ = (magZMax + magZMin) / 2;

  	  //Log.info("NEW refMagX = %d, refMagY = %d, refMagZ = %d", refMagX, refMagY, refMagZ);

      accelmag.setMagVectorMagnitudeReferenceValue(refMagX, refMagY, refMagZ);
#endif
    }
    // Log.info("INT1 source: INT_SOURCE:0x%02X, MAG_SOURCE:0x%02X", intSrc, magIntSrc);

    /* Decrement the unhandled int counter. */
    int1_fired--;
  }
}
