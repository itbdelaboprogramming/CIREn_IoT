#include <driver/spi_master.h>
#include "sdkconfig.h"
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
////////////////////////////////////
//
//#define MAGTYPE  true
//#define XGTYPE   false
//////////////////////////
///////////////////////////
//
////////////////////////////
 uint8_t GetLowBits();
 int8_t GetHighBits();
 int fReadSPIdata16bits( spi_device_handle_t &h, int address );
 int fWriteSPIdata8bits( spi_device_handle_t &h, int address, int sendData );
 int fInitializeSPI_Devices( spi_host_device_t SPI_Host, spi_device_handle_t &h, int csPin);
// spi_device_handle_t fInitializeSPI_Devices( int csPin);
int fInitializeSPI_Channel( int spiCLK, int spiMOSI, int spiMISO, spi_host_device_t SPI_Host, bool EnableDMA);