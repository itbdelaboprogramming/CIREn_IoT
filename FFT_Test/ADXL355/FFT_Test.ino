// Complete FFT + Multi-Sensor + Event Detection Framework with Tagged Batches + I2C Sensors + CSV/JSON Export

#include <Arduino.h>
#include <Wire.h>

#include <arduinoFFT.h>
#include <DFRobot_ADXL345.h>
#include <DFRobot_BNO055.h>
#include <DFRobot_BMP280.h>
#include "PL_ADXL355.h"

/*
  ============================
    Table of Contents (TOC)
  ============================
  [1] Constants        : Configuration constants variables.
  [2] Global Variables : Object instances and sensor data storage.
  [3] Function Declarations : Prototypes for user-defined functions.
  [4] Setup            : Initialization routine run once at boot.
  [5] Loop             : Main execution loop using non-blocking logic.
  [6] Function Definitions : Implementations of declared functions.
*/

// [1] ========================= CONSTANTS =========================
#define SCL_PIN 22 // custom I2C SCL pin
#define SDA_PIN 21 // custom I2C SDA pin

#define NUM_SENSORS 0 // Adjust as needed
#define NUM_ADXL345 0 // Extra channels for I2C sensors (3 accel)
#define NUM_ADXL355 1 // Extra channels for I2C sensors (3 accel)
#define NUM__BNO055 0 // Extra channels for I2C sensors (3 accel + 3 gyro)
#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure

using namespace PL;

// Timing Constants
const unsigned long mainTime = 10;
const unsigned long restTime = 10; // ms between samples

// ADC Configuration
const float ADC_vol = 3.3; // 3.3V for ESP32's ADC
const uint8_t ADC_bit = 10; // 12 by default
const unsigned int ADC_max = (1U << ADC_bit) - 1;
const float ADC_mag = (ADC_vol / (float)(ADC_max)); // (Reference Voltage / Maximum ADC Value)

// FFT Configuration
const uint16_t SAMPLES = 256;
const float samplingFrequency = 1000.0;  // Reduced for I2C sensors
#if (NUM_SENSORS <= 0)
  const uint8_t sensorPins[1] = {25};
#else
  const uint8_t sensorPins[NUM_SENSORS] = {25};
#endif
const uint8_t NUM_FFT = (NUM_SENSORS * 1) + ((NUM_ADXL345 + NUM_ADXL355) * 3) + (NUM__BNO055 * 6);

// [2] ============== GLOBAL VARIABLES ==============
struct Vector3 {
  float x;
  float y;
  float z;
};

// I2C Configuration
typedef DFRobot_BMP280_IIC    BMP;
typedef DFRobot_BNO055_IIC    BNO;

BNO bno(&Wire, 0x28);
BMP bmp(&Wire, BMP::eSdoLow);
DFRobot_ADXL345_I2C adxl345(&Wire, 0x53);
ADXL355 adxl355 = ADXL355(); // default 0x1D or 0x53
const uint8_t adxl355_i2c_address = 0x53;

// Timing Variables
unsigned long lastMainTime = 0;
unsigned long lastSampleTime = 0;
uint32_t batchCounter = 0;   // To tag each batch with an ID

// FFT Variables
float vReal[NUM_FFT][SAMPLES];
float vImag[NUM_FFT][SAMPLES];
ArduinoFFT<float> FFT[NUM_FFT] = {
  ArduinoFFT<float>(vReal[0], vImag[0], SAMPLES, samplingFrequency),
  ArduinoFFT<float>(vReal[1], vImag[1], SAMPLES, samplingFrequency),
  ArduinoFFT<float>(vReal[2], vImag[2], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[3], vImag[3], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[4], vImag[4], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[5], vImag[5], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[6], vImag[6], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[7], vImag[7], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[8], vImag[8], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[9], vImag[9], SAMPLES, samplingFrequency)
};

bool BNO_init = false;
bool BMP_init = false;
bool ADXL355_init = false;

// Data I/O Variables
float temp = 0.0;
float alti = 0.0;
uint32_t press = 0;

// Configurable Variables
bool enableEventDetection = false; // Toggle event detection
bool exportCSV = true;
bool exportJSON = false;


// [3] ============== FUNCTION DECLARATIONS ==============
// Status Checking Functions
bool detectEvent(float peakFreq, float peakAmplitude, float minFreq = 10.0, float maxFreq = 200.0, float minAmplitude = 5.0);
void BNOprintLastOperateStatus(BNO::eStatus_t eStatus);
void BMPprintLastOperateStatus(BMP::eStatus_t eStatus);

// Initialization Functions
bool BNOSetup(uint16_t interval = 2000, int iteration = -1);
bool BMPSetup(uint16_t interval = 2000, int iteration = -1);
bool ADXL355Setup(uint16_t interval = 2000, int iteration = -1);

// Functions to Read or Write Value to I/O
void sampleAnalogSensor(uint8_t index, bool removeDC = true);
void sampleADXL345(uint8_t baseIndex);
void sampleADXL355(uint8_t baseIndex);
void sampleBNO055(uint8_t baseIndex);
Vector3 rotateByQuaternion(Vector3 v, float qw, float qx, float qy, float qz);

// Data Pocessing Functions
void processFFT(uint8_t index, const char* label = "", uint8_t unit = 0, uint8_t decimal = 3);
void FFT_Task(void *pvParameters);

// [4] ========================= SETUP =========================
void setup() {
  Serial.begin(115200);
  Wire.setPins(SDA_PIN, SCL_PIN);
  //Wire.begin();
  if (NUM_SENSORS > 0) analogReadResolution(ADC_bit); // For ESP32/ESP8266

  if (NUM_ADXL345 > 0) {
    adxl345.begin();
    adxl345.powerOn();
    adxl345.setRangeSetting(4); // (2 for +- 2g, 4 for 4g, 8 for 8g, 16 for 16g)
  }

  if (NUM_ADXL355 > 0) {
    ADXL355_init = ADXL355Setup();
  }

  if (NUM__BNO055 > 0) {
    BNO_init = BNOSetup();
    BMP_init = BMPSetup();
  }

  // Create sensor task
  xTaskCreatePinnedToCore(
    FFT_Task,   // Task function
    "SensorTask",            // Name
    8192,                   // Stack size
    NULL,                    // Parameters
    1,                       // Priority
    NULL,                    // Task handle
    0                        // Run on core 0 (ESP32)
  );
}

// [5] ========================= LOOP =========================
void loop() {
  if (millis() - lastMainTime >= mainTime) {
    lastMainTime = millis();

    //Serial.println("Main Loop Here!");
  }
}

// [6] ============== FUNCTION DEFINITIONS ==============
bool detectEvent(float peakFreq, float peakAmplitude, float minFreq, float maxFreq, float minAmplitude) {return (peakFreq >= minFreq && peakFreq <= maxFreq && peakAmplitude >= minAmplitude);}

// Analog Sensor Section code (Piezo are included)
void sampleAnalogSensor(uint8_t index, bool removeDC) {
  uint8_t pin = sensorPins[index];
  for (uint16_t i = 0; i < SAMPLES; i++) {
    vReal[index][i] = analogRead(pin) * ADC_mag;
    vImag[index][i] = 0.0;
    delayMicroseconds((1.0 / samplingFrequency) * 1e6);
  }
  if (removeDC) {
    float avg = 0;
    for (uint16_t i = 0; i < SAMPLES; i++) avg += vReal[index][i];
    avg /= SAMPLES;
    for (uint16_t i = 0; i < SAMPLES; i++) vReal[index][i] -= avg;
  }
}
// ==================================

// ADXL345 Section code (I2C or SPI)
void sampleADXL345(uint8_t baseIndex) {
  int accval[3];
  for (uint16_t i = 0; i < SAMPLES; i++) {
    adxl345.readAccel(accval);
    vReal[baseIndex + 0][i] = accval[0] * 9.81 / 1000.0;
    vReal[baseIndex + 1][i] = accval[1] * 9.81 / 1000.0;
    vReal[baseIndex + 2][i] = accval[2] * 9.81 / 1000.0;
    for (uint8_t j = 0; j < 3; j++) vImag[baseIndex + j][i] = 0;
    delayMicroseconds((1.0 / samplingFrequency) * 1e6);
  }
}
// ==================================

/*
// ADXL355 Section code (I2C or SPI)
bool ADXL355Setup(uint16_t interval, int iteration) {
  int tried = -2;
  bool status = adxl355.begin();
  while(!status && tried < iteration) {
    Serial.printf("ADXL355 Status: %d\n", status);
    if (iteration >= 0) {
      if (tried < 0) {tried = 0;}
      else {tried += 1;}
    }
    Serial.println("ADXL355 begin failed");
    delay(interval);
    status = adxl355.begin();
  }
  if (status) {
    int res = adxl355.StopTempSensor();
    if (!res) {res = adxl355.Stop();}
    if (!res) {res = adxl355.SetRange(ADXL355::RANGE_VALUES::RANGE_8G);}
    if (!res) {res = adxl355.SetOdrLpf(ADXL355::ODR_LPF::ODR_4000_AND_1000);}
    if (!res) {res = adxl355.Start();}
    if (!res) {res = adxl355.StartTempSensor();}
    if (!res)
      return true;
  }
  return false;
}

void sampleADXL355(uint8_t baseIndex) {
  float accval[3];
  for (uint16_t i = 0; i < SAMPLES; i++) {
    adxl355.GetAccAxes(&accval[0], &accval[1], &accval[2]);
    vReal[baseIndex + 0][i] = accval[0]; // Already in m/s2
    vReal[baseIndex + 1][i] = accval[1]; // Already in m/s2
    vReal[baseIndex + 2][i] = accval[2]; // Already in m/s2
    for (uint8_t j = 0; j < 3; j++) vImag[baseIndex + j][i] = 0;
    delayMicroseconds((1.0 / samplingFrequency) * 1e6);
  }
}
// ==================================
*/

// === ADXL355 Section code (I2C or SPI) ===
bool ADXL355Setup(uint16_t interval, int iteration) {
  int tried = -2;
  adxl355.reset();
  adxl355.beginI2C(adxl355_i2c_address);
  delay(interval);
  ADXL355_Status adxlStatus = adxl355.getStatus();
  uint8_t status = (uint8_t)(adxlStatus);
  Serial.printf("ADXL355 Status: %d, %d\n", adxlStatus, status);
  if (adxlStatus == ADXL355_Status::dataReady || adxlStatus == ADXL355_Status::fifoFull || adxlStatus == ADXL355_Status::fifoOverrun || adxlStatus == ADXL355_Status::activity || adxlStatus == ADXL355_Status::nvmBusy) {
    status = (uint8_t)(adxlStatus);
  }

  while(!status && tried < iteration) {
    Serial.printf("ADXL355 Status: %d, %d\n", adxlStatus, status);
    if (iteration >= 0) {
      if (tried < 0) {tried = 0;}
      else {tried += 1;}
    }
    Serial.println("ADXL355 begin failed");
    adxl355.beginI2C(adxl355_i2c_address);
    delay(interval);
    adxlStatus = adxl355.getStatus();
    if (adxlStatus == ADXL355_Status::dataReady || adxlStatus == ADXL355_Status::fifoFull || adxlStatus == ADXL355_Status::fifoOverrun || adxlStatus == ADXL355_Status::activity || adxlStatus == ADXL355_Status::nvmBusy) {
      status = (uint8_t)(adxlStatus);
    }
  }

  if (status) {
    adxl355.disableTemperature();
    adxl355.disableMeasurement();
    adxl355.setRange(ADXL355_Range::range8g);
    adxl355.setOutputDataRate(ADXL355_OutputDataRate::odr4000);
    adxl355.enableMeasurement();
    adxl355.enableTemperature();
    return true;
  }
  return false;
}

void sampleADXL355(uint8_t baseIndex) {
  ADXL355_Accelerations Accels;
  for (uint16_t i = 0; i < SAMPLES; i++) {
    Accels = adxl355.getAccelerations();
    vReal[baseIndex + 0][i] = Accels.x; // Already in m/s2
    vReal[baseIndex + 1][i] = Accels.y; // Already in m/s2
    vReal[baseIndex + 2][i] = Accels.z; // Already in m/s2
    for (uint8_t j = 0; j < 3; j++) vImag[baseIndex + j][i] = 0;
    delayMicroseconds((1.0 / samplingFrequency) * 1e6);
  }
}
// ==================================

// BNO055 Section code (I2C)
void BNOprintLastOperateStatus(BNO::eStatus_t eStatus) {
  switch(eStatus) {
  case BNO::eStatusOK:    Serial.println("everything ok"); break;
  case BNO::eStatusErr:   Serial.println("unknow error"); break;
  case BNO::eStatusErrDeviceNotDetect:    Serial.println("device not detected"); break;
  case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("device ready time out"); break;
  case BNO::eStatusErrDeviceStatus:       Serial.println("device internal status error"); break;
  default: Serial.println("unknow status"); break;
  }
}

// timeout in millisecond & iteration = -1 means unlimited
bool BNOSetup(uint16_t interval, int iteration) {
  int tried = -2;
  bno.reset();
  BNO::eStatus_t status = bno.begin();
  while(status != BNO::eStatusOK && tried < iteration) {
    if (iteration >= 0) {
      if (tried < 0) {tried = 0;}
      else {tried += 1;}
    }
    Serial.println("bno begin failed");
    BNOprintLastOperateStatus(bno.lastOperateStatus);
    delay(interval);
    status = bno.begin();
  }
  if (status == BNO::eStatusOK) {
    bno.setPowerMode(BNO::ePowerModeNormal);    // set to normal power mode
    bno.setOprMode(BNO::eOprModeConfig);    // must set sensor to config-mode before configure

    // accelerometer normal configure
    bno.setAccRange(BNO::eAccRange_4G);   // set range to 4g
    bno.setAccBandWidth(BNO::eAccBandWidth_62_5);   // set band width 62.5HZ
    bno.setAccPowerMode(BNO::eAccPowerModeNormal);  // set accelerometer power mode

    // gyroscope normal configure
    bno.setGyrRange(BNO::eGyrRange_2000);   // set range
    bno.setGyrBandWidth(BNO::eGyrBandWidth_32);   // set band width
    bno.setGyrPowerMode(BNO::eGyrPowerModeNormal);    // set power mode

    // magnetometer normal configure
    bno.setMagDataRate(BNO::eMagDataRate_20);   // set output data rate 20HZ
    bno.setMagPowerMode(BNO::eMagPowerModeForce);   // set power mode
    bno.setMagOprMode(BNO::eMagOprModeRegular); // set operate mode
  
    BNO::sAxisAnalog_t    sOffsetAcc;   // unit mg, members can't out of acc range
    BNO::sAxisAnalog_t    sOffsetGyr;   // unit dps, members can't out of gyr range
    BNO::sAxisAnalog_t    sOffsetMag;   // unit ut, members can't out of mag range
    sOffsetAcc.x = 1;
    sOffsetAcc.y = 1;
    sOffsetAcc.z = 1;
    sOffsetGyr.x = 1;
    sOffsetGyr.y = 1;
    sOffsetGyr.z = 1;
    sOffsetMag.x = 1;
    sOffsetMag.y = 1;
    sOffsetMag.z = 1;
    bno.setAxisOffset(BNO::eAxisAcc, sOffsetAcc);   // set offset
    bno.setAxisOffset(BNO::eAxisGyr, sOffsetGyr);
    bno.setAxisOffset(BNO::eAxisMag, sOffsetMag);

    bno.setOprMode(BNO::eOprModeNdof);   // shift to other operate mode, reference datasheet for more detail
    return true;
  }
  return false;
}

Vector3 rotateByQuaternion(Vector3 v, float qw, float qx, float qy, float qz) {
  // Quaternion multiplication: q * v * q_conjugate
  float ix =  qw * v.x + qy * v.z - qz * v.y;
  float iy =  qw * v.y + qz * v.x - qx * v.z;
  float iz =  qw * v.z + qx * v.y - qy * v.x;
  float iw = -qx * v.x - qy * v.y - qz * v.z;

  Vector3 result;
  result.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
  result.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
  result.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;

  return result;
}

void sampleBNO055(uint8_t baseIndex) {
  BNO::sAxisAnalog_t   sGyrAnalog, sLiaAnalog; //, sAccAnalog, sMagAnalog, sGrvAnalog;
  //BNO::sEulAnalog_t    sEulAnalog;
  BNO::sQuaAnalog_t    sQuaAnalog;
  float norm, norm_w, norm_x, norm_y, norm_z;
  for (uint16_t i = 0; i < SAMPLES; i++) {
    sQuaAnalog = bno.getQua();                // Sensor coordniate to global coordinate
    sGyrAnalog = bno.getAxis(BNO::eAxisGyr);  // angular velocity Sensor coordinate
    sLiaAnalog = bno.getAxis(BNO::eAxisLia);  // Linear acceleration Sensor coordinate (use this for get clean acceleration without gravity)
    //sAccAnalog = bno.getAxis(BNO::eAxisAcc);  // raw acceleration Sensor coordinate
    //sMagAnalog = bno.getAxis(BNO::eAxisMag);  // raw geomagnetic
    //sGrvAnalog = bno.getAxis(BNO::eAxisGrv);  // gravity vector
    //sEulAnalog = bno.getEul();                // oritentation tracker sensor coordinate
    Vector3 accVec = {sLiaAnalog.x, sLiaAnalog.y, sLiaAnalog.z};
    //Vector3 accVec = {sAccAnalog.x, sAccAnalog.y, sAccAnalog.z};
    Vector3 gyrVec = {sGyrAnalog.x, sGyrAnalog.y, sGyrAnalog.z};

    // Normalize quaternion
    norm = sqrt(sQuaAnalog.w * sQuaAnalog.w + sQuaAnalog.x * sQuaAnalog.x + sQuaAnalog.y * sQuaAnalog.y + sQuaAnalog.z * sQuaAnalog.z);
    norm_w = (sQuaAnalog.w / norm); norm_x = (sQuaAnalog.x / norm); norm_y = (sQuaAnalog.y / norm); norm_z = (sQuaAnalog.z / norm);

    Vector3 accGlobal = rotateByQuaternion(accVec, norm_w, norm_x, norm_y, norm_z);
    Vector3 gyrGlobal = rotateByQuaternion(gyrVec, norm_w, norm_x, norm_y, norm_z);
    
    // change to accVec or directly use sAccAnalog/sLiaAnalog for sensor coordinate and use accGlobal for global coordinate
    vReal[baseIndex + 0][i] = accGlobal.x; // * 9.81 / 1000.0;
    vReal[baseIndex + 1][i] = accGlobal.y; // * 9.81 / 1000.0;
    vReal[baseIndex + 2][i] = accGlobal.z; // * 9.81 / 1000.0;
    // change to gyrVec or directly use sGyrAnalog for sensor coordinate and use gyrGlobal for global coordinate
    vReal[baseIndex + 3][i] = gyrGlobal.x;
    vReal[baseIndex + 4][i] = gyrGlobal.y;
    vReal[baseIndex + 5][i] = gyrGlobal.z;
    for (uint8_t j = 0; j < 6; j++) vImag[baseIndex + j][i] = 0;
    delayMicroseconds((1.0 / samplingFrequency) * 1e6);
  }
}
// ==================================

// BMP280 section code (I2C)
void BMPprintLastOperateStatus(BMP::eStatus_t eStatus) {
  switch(eStatus) {
  case BMP::eStatusOK:    Serial.println("everything ok"); break;
  case BMP::eStatusErr:   Serial.println("unknow error"); break;
  case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}

bool BMPSetup(uint16_t interval, int iteration) {
  int tried = -2;
  bmp.reset();
  BMP::eStatus_t status = bmp.begin();
  while(status != BMP::eStatusOK && tried < iteration) {
    if (iteration >= 0) {
      if (tried < 0) {tried = 0;}
      else {tried += 1;}
    }
    Serial.println("bmp begin failed");
    BMPprintLastOperateStatus(bmp.lastOperateStatus);
    delay(interval);
    status = bmp.begin();
  }
  if (status == BMP::eStatusOK) {
    bmp.setConfigFilter(BMP::eConfigFilter_off);        // set config filter
    bmp.setConfigTStandby(BMP::eConfigTStandby_125);    // set standby time
    bmp.setCtrlMeasSamplingTemp(BMP::eSampling_X8);     // set temperature over sampling
    bmp.setCtrlMeasSamplingPress(BMP::eSampling_X8);    // set pressure over sampling
    bmp.setCtrlMeasMode(BMP::eCtrlMeasModeNormal);      // set control measurement mode to make these settings effective
    return true;
  }
  return false;
}
// ==================================

// Data processing section code
void processFFT(uint8_t index, const char* label, uint8_t unit, uint8_t decimal) {
  FFT[index].windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT[index].compute(FFTDirection::Forward);
  FFT[index].complexToMagnitude();

  float peakFreq, peakAmplitude, scaleFactor, peakDiv;
  FFT[index].majorPeak(&peakFreq, &peakAmplitude);
  scaleFactor = 1.0 / peakAmplitude;
  //if (peakFreq < 1) {peakDiv = 1;}
  //else {peakDiv = peakFreq;}
  float delTime = (1.0 / samplingFrequency);
  if (unit == 2) {scaleFactor = 1 / (2 * delTime * delTime);}
  else if (unit == 1) {scaleFactor = delTime;}
  else {scaleFactor = 1;}
  if (enableEventDetection && detectEvent(peakFreq, peakAmplitude)) {
    Serial.print("Sensor "); Serial.print(label); Serial.println(" 🔔 EVENT DETECTED!");
    Serial.print("  Peak Frequency: "); Serial.print(peakFreq); Serial.println(" Hz");
    Serial.print("  Amplitude: "); Serial.println(peakAmplitude * scaleFactor, decimal);
  }

  // CSV Export
  if (exportCSV) {
    Serial.print("$SENSOR,");
    Serial.println(label);
    for (uint16_t i = 1; i < SAMPLES / 2; i++) {
      float freq = (i * 1.0 * samplingFrequency) / SAMPLES;
      float div = freq;
      scaleFactor = 1.0 / peakAmplitude;
      //if (freq <= 1) {div = 1;}
      if (unit == 2) {scaleFactor = 1 / (2 * delTime * delTime);}
      else if (unit == 1) {scaleFactor = delTime;}
      else {scaleFactor = 1;}
      float value = vReal[index][i] * scaleFactor;
      Serial.print(freq, 3); Serial.print(",");
      Serial.println(value, decimal);
    }
    Serial.println("");
  }

  // JSON Export (basic format)
  if (exportJSON) {
    Serial.print("{\"label\":\""); Serial.print(label); Serial.print("\",\"fft\":[");
    for (uint16_t i = 1; i < SAMPLES / 2; i++) {
      float freq = (i * 1.0 * samplingFrequency) / SAMPLES;
      float div = freq;
      scaleFactor = 1.0 / peakAmplitude;
      //if (freq <= 1) {div = 1;}
      if (unit == 2) {scaleFactor = 1 / (2 * delTime * delTime);}
      else if (unit == 1) {scaleFactor = delTime;}
      else {scaleFactor = 1;}
      float value = vReal[index][i] * scaleFactor;
      if (i > 0) Serial.print(",");
      Serial.print("["), Serial.print(freq, 3);
      Serial.print(",");
      Serial.print(value, decimal);
      Serial.print("]");
    }
    Serial.println("]}");
    Serial.println("");
  }
}

void FFT_Task(void *pvParameters) {
  while (true) {
    lastSampleTime = millis();
    
    if (BMP_init) {
      temp = bmp.getTemperature();
      press = bmp.getPressure();
      alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);
    }

    Serial.print("#START,");
    Serial.print(batchCounter++);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(temp,2);
    Serial.print(",");
    Serial.print(press);
    Serial.print(",");
    Serial.println(alti,2);

    if (NUM_SENSORS > 0) {
      // Analog sensors
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        sampleAnalogSensor(i);
        String label = "PIEZO_" + String(i+1);
        processFFT(i, label.c_str(), 0, 3);
      }
    }

    if (NUM_ADXL345 > 0) {
      // ADXL345
      uint8_t fft_adxl;
      for (uint8_t i = 0; i < NUM_ADXL345; i++) {
        fft_adxl = NUM_SENSORS + (i * 3);
        sampleADXL345(fft_adxl);
        for (uint8_t j = 0; j < 3; j++) {
          String label = "ADXL345_";
          if (j == 0) {label += "X_" + String(i+1);}
          else if (j == 1) {label += "Y_" + String(i+1);}
          else if (j == 2) {label += "Z_" + String(i+1);}

          processFFT((fft_adxl + j), label.c_str(), 2, 4);
        }
      }
    }

    if (NUM_ADXL355 > 0 && ADXL355_init) {
      // ADXL355
      uint8_t fft_adxl;
      for (uint8_t i = 0; i < NUM_ADXL355; i++) {
        fft_adxl = NUM_SENSORS + ((NUM_ADXL345 + i) * 3);
        sampleADXL355(fft_adxl);
        for (uint8_t j = 0; j < 3; j++) {
          String label = "ADXL355_";
          if (j == 0) {label += "X_" + String(i+1);}
          else if (j == 1) {label += "Y_" + String(i+1);}
          else if (j == 2) {label += "Z_" + String(i+1);}

          processFFT((fft_adxl + j), label.c_str(), 2, 4);
        }
      }
    }

    if (NUM__BNO055 > 0 && BNO_init) {
      // BNO055 Accel + Gyro
      uint8_t fft_bno;
      for (uint8_t i = 0; i < NUM__BNO055; i++) {
        fft_bno = NUM_SENSORS + ((NUM_ADXL345 + NUM_ADXL355) * 3) + (i * 6);
        sampleBNO055(fft_bno);
        for (uint8_t j = 0; j < 6; j++) {
          String label = "BNO055_";
          if (j == 0) {label += "AX_" + String(i+1);}
          else if (j == 1) {label += "AY_" + String(i+1);}
          else if (j == 2) {label += "AZ_" + String(i+1);}
          else if (j == 3) {label += "GX_" + String(i+1);}
          else if (j == 4) {label += "GY_" + String(i+1);}
          else if (j == 5) {label += "GZ_" + String(i+1);}

          if (j < 3) {processFFT((fft_bno + j), label.c_str(), 0, 4);}
          else {processFFT((fft_bno + j), label.c_str(), 0, 2);}
        }
      }
    }

    Serial.println("#END\n");

    vTaskDelay(pdMS_TO_TICKS(restTime)); // While loop use
  }
}
