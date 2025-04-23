#pragma
#include <Arduino.h>

/**
 * @brief Driver class for the TCRT5000 reflective optical sensor.
 * 
 * This sensor can be used in two wiring configurations:
 * - With only the receiver pin (passive reading).
 * - With both receiver and LED emitter pins (active reading).
 */
class TCRT5000Sensor {
public:
    /**
     * @brief Constructor for passive mode (no LED control).
     * @param receiverPin The input pin connected to the sensor's output.
     */
    TCRT5000Sensor(uint8_t receiverPin);

    /**
     * @brief Constructor for active mode (controls the IR LED).
     * @param receiverPin The input pin connected to the sensor's output.
     * @param ledPin The output pin connected to the sensor's LED anode.
     */
    TCRT5000Sensor(uint8_t receiverPin, uint8_t ledPin);

    /**
     * @brief Checks if an object is detected (close).
     * @return true if object is detected; false otherwise.
     */
    bool isClose();

private:
    uint8_t _receiverPin;
    int8_t _ledPin; // -1 if not used (passive mode)
};
