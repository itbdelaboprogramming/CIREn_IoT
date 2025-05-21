// Complete FFT + Multi-Sensor + Event Detection Framework with Tagged Batches + I2C Sensors + CSV/JSON Export

#include <arduinoFFT.h>
#include <Wire.h>
#include <DFRobot_ADXL345.h>
#include <DFRobot_BNO055.h>
#include <DFRobot_BMP280.h>

#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure
#define NUM_SENSORS 0 // Adjust as needed
#define NUM_ADXL345 0 // Extra channels for I2C sensors (3 accel)
#define NUM__BNO055 1 // Extra channels for I2C sensors (3 accel + 3 gyro)
#define SCL_PIN 22 // custom I2C SCL pin
#define SDA_PIN 21 // custom I2C SDA pin

typedef DFRobot_BMP280_IIC    BMP;
// BNO055 Section code (I2C)
typedef DFRobot_BNO055_IIC    BNO;

struct Vector3 {
  float x;
  float y;
  float z;
};
// =============

const float ADC_vol = 3.3; // 3.3V for ESP32's ADC
const uint8_t ADC_bit = 10; // 12 by default
const unsigned int ADC_max = (1U << ADC_bit) - 1;

bool exportCSV = true;
bool exportJSON = false;
bool BNO_init = false;
bool BMP_init = false;

const uint16_t SAMPLES = 128;
float samplingFrequency = 500.0;  // Reduced for I2C sensors
#if (NUM_SENSORS <= 0)
  const uint8_t sensorPins[1] = {27};
#else
  const uint8_t sensorPins[NUM_SENSORS] = {27};
#endif
const uint8_t NUM_FFT = (NUM_SENSORS * 1) + (NUM_ADXL345 * 3) + (NUM__BNO055 * 6);
float vReal[NUM_FFT][SAMPLES];
float vImag[NUM_FFT][SAMPLES];

ArduinoFFT<float> FFT[NUM_FFT] = {
  ArduinoFFT<float>(vReal[0], vImag[0], SAMPLES, samplingFrequency),
  ArduinoFFT<float>(vReal[1], vImag[1], SAMPLES, samplingFrequency),
  ArduinoFFT<float>(vReal[2], vImag[2], SAMPLES, samplingFrequency),
  ArduinoFFT<float>(vReal[3], vImag[3], SAMPLES, samplingFrequency),
  ArduinoFFT<float>(vReal[4], vImag[4], SAMPLES, samplingFrequency),
  ArduinoFFT<float>(vReal[5], vImag[5], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[6], vImag[6], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[7], vImag[7], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[8], vImag[8], SAMPLES, samplingFrequency),
  //ArduinoFFT<float>(vReal[9], vImag[9], SAMPLES, samplingFrequency)
};

DFRobot_ADXL345_I2C adxl(&Wire, 0x53);
BNO bno(&Wire, 0x28);
BMP bmp(&Wire, BMP::eSdoLow);

float temp = 0.0;
float alti = 0.0;
uint32_t press = 0;

unsigned long lastSampleTime = 0;
unsigned long restTime = 100; // ms between samples
uint32_t batchCounter = 0;   // To tag each batch with an ID

bool enableEventDetection = false; // Toggle event detection

bool detectEvent(
  float peakFreq,
  float peakAmplitude,
  float minFreq = 10.0,
  float maxFreq = 200.0,
  float minAmplitude = 5.0
) {
  return (peakFreq >= minFreq && peakFreq <= maxFreq && peakAmplitude >= minAmplitude);
}

// Analog Sensor Section code (Piezo are included)
void sampleAnalogSensor(uint8_t index, bool removeDC = true) {
  uint8_t pin = sensorPins[index];
  for (uint16_t i = 0; i < SAMPLES; i++) {
    vReal[index][i] = analogRead(pin);
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
    adxl.readAccel(accval);
    vReal[baseIndex + 0][i] = accval[0] * 9.81 / 1000.0;
    vReal[baseIndex + 1][i] = accval[1] * 9.81 / 1000.0;
    vReal[baseIndex + 2][i] = accval[2] * 9.81 / 1000.0;
    for (uint8_t j = 0; j < 3; j++) vImag[baseIndex + j][i] = 0;
    delayMicroseconds((1.0 / samplingFrequency) * 1e6);
  }
}
// ==================================

// BNO055 Section code (I2C)
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
    vReal[baseIndex + 0][i] = accGlobal.x * 9.81 / 1000.0;
    vReal[baseIndex + 1][i] = accGlobal.y * 9.81 / 1000.0;
    vReal[baseIndex + 2][i] = accGlobal.z * 9.81 / 1000.0;
    // change to gyrVec or directly use sGyrAnalog for sensor coordinate and use gyrGlobal for global coordinate
    vReal[baseIndex + 3][i] = gyrGlobal.x;
    vReal[baseIndex + 4][i] = gyrGlobal.y;
    vReal[baseIndex + 5][i] = gyrGlobal.z;
    for (uint8_t j = 0; j < 6; j++) vImag[baseIndex + j][i] = 0;
    delayMicroseconds((1.0 / samplingFrequency) * 1e6);
  }
}

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
bool BNOSetup(uint16_t interval = 2000, int iteration = -1) {
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

bool BMPSetup(uint16_t interval = 2000, int iteration = -1) {
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

void processFFT(uint8_t index, const char* label = "", uint8_t unit = 0, uint8_t decimal = 3) {
  FFT[index].windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT[index].compute(FFTDirection::Forward);
  FFT[index].complexToMagnitude();

  float peakFreq, peakAmplitude, scaleFactor, peakDiv;
  FFT[index].majorPeak(&peakFreq, &peakAmplitude);
  scaleFactor = 1.0 / peakAmplitude;
  //scaleFactor = (ADC_vol / (float)(ADC_max)); // (Reference Voltage / Maximum ADC Value)
  //if (peakFreq < 1) {peakDiv = 1;}
  //else {peakDiv = peakFreq;}
  //if (unit == 2) {scaleFactor = 1 / (4 * PI * PI * peakFreq * peakFreq);}
  //else if (unit == 1) {scaleFactor = 1 / (2 * PI * peakFreq);}
  //else {scaleFactor = 1;}
  if (enableEventDetection && detectEvent(peakFreq, peakAmplitude)) {
    Serial.print("Sensor "); Serial.print(label); Serial.println(" 🔔 EVENT DETECTED!");
    Serial.print("  Peak Frequency: "); Serial.print(peakFreq); Serial.println(" Hz");
    Serial.print("  Amplitude: "); Serial.println(peakAmplitude * scaleFactor, decimal);
  }

  // CSV Export
  if (exportCSV) {
    Serial.print("$SENSOR,");
    Serial.println(label);
    for (uint16_t i = 0; i < SAMPLES / 2; i++) {
      float freq = (i * 1.0 * samplingFrequency) / SAMPLES;
      float div = freq;
      scaleFactor = 1.0 / peakAmplitude;
      //if (freq <= 1) {div = 1;}
      //if (unit == 2) {scaleFactor = 1 / (4 * PI * PI * div * div);}
      //else if (unit == 1) {scaleFactor = 1 / (2 * PI * div);}
      //else {scaleFactor = 1;}
      float value = vReal[index][i] * scaleFactor;
      Serial.print(freq, 3); Serial.print(",");
      Serial.println(value, decimal);
    }
    Serial.println("");
  }

  // JSON Export (basic format)
  if (exportJSON) {
    Serial.print("{\"label\":\""); Serial.print(label); Serial.print("\",\"fft\":[");
    for (uint16_t i = 0; i < SAMPLES / 2; i++) {
      float freq = (i * 1.0 * samplingFrequency) / SAMPLES;
      float div = freq;
      scaleFactor = 1.0 / peakAmplitude;
      //if (freq <= 1) {div = 1;}
      //if (unit == 2) {scaleFactor = 1 / (4 * PI * PI * div * div);}
      //else if (unit == 1) {scaleFactor = 1 / (2 * PI * div);}
      //else {scaleFactor = 1;}
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

void setup() {
  Serial.begin(115200);
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  if (NUM_SENSORS > 0) analogReadResolution(ADC_bit); // For ESP32/ESP8266

  if (NUM_ADXL345 > 0) {
    adxl.begin();
    adxl.powerOn();
    adxl.setRangeSetting(4); // (2 for +- 2g, 4 for 4g, 8 for 8g, 16 for 16g)
  }

  if (NUM__BNO055 > 0) {
    BNO_init = BNOSetup();
    BMP_init = BMPSetup();
  }
}

void loop() {
  if (millis() - lastSampleTime >= restTime) {
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

    if (NUM__BNO055 > 0 && BNO_init) {
      // BNO055 Accel + Gyro
      uint8_t fft_bno;
      for (uint8_t i = 0; i < NUM__BNO055; i++) {
        fft_bno = NUM_SENSORS + (NUM_ADXL345 * 3) + (i * 6);
        sampleBNO055(fft_bno);
        for (uint8_t j = 0; j < 6; j++) {
          String label = "BNO055_";
          if (j == 0) {label += "AX_" + String(i+1);}
          else if (j == 1) {label += "AY_" + String(i+1);}
          else if (j == 2) {label += "AZ_" + String(i+1);}
          else if (j == 3) {label += "GX_" + String(i+1);}
          else if (j == 4) {label += "GY_" + String(i+1);}
          else if (j == 5) {label += "GZ_" + String(i+1);}

          if (j < 3) {processFFT((fft_bno + j), label.c_str(), 2, 4);}
          else {processFFT((fft_bno + j), label.c_str(), 1, 2);}
        }
      }
    }

    Serial.println("#END\n");
  }
}
