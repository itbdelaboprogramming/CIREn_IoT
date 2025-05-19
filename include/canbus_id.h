enum sensorId {
  DHT22 = 0,
  ADXL345 = 1,
  BM3680 = 2,
  BMP280 = 3,
  MAX6675 = 4,
};

enum dhtDataId {
  HUMIDITY = 0,
  TEMPERATURE = 1
};

enum adxlDataId {
    ACCEL_X = 0,
    ACCEL_Y = 1,
    ACCEL_Z = 2,
};

enum bme680DataId {
    TEMP = 0, // degree (C)
    PRESSURE = 1, // pascal (Pa)
    HUMIDITY = 2, // rh
    ALTITUDE = 3, // meter (m)
    GAS_RESISTANCE = 4, // ohm 
    SEA_LEVEL = 5 // meter (m)
};

enum bmp280DataId {
    TEMP = 0, // celcius (c)
    PRESSURE = 1, // UINT32 - pa
};

enum max6675DataId {
    TEMP = 0, // celcius (c)
}







