#include <Arduino.h>
#include <FastLED.h>

// Define what pin hardware is attached to.
const uint8_t LED_DATA_PIN = 15;
const uint8_t BUTTON_PIN_NO = 2;

// Create the required ping body, which we use to broadcast our identitication.
const uint8_t VENDOR_ID[4]  = { 0x00, 0x00, 0x00, 0x01 };
const uint8_t PRODUCT_ID[4] = { 0x00, 0x00, 0x00, 0x01 };
const uint8_t SERIAL_NUM[4] = { 0x00, 0x00, 0x00, 0x01 };

// Timings / Delays.
const uint32_t PING_INTERVAL_SHORT = 2000;  // 2 seconds
const uint32_t PING_INTERVAL_LONG  = 10000; // 10 seconds
const int      RESPONSE_TIMEOUT    = 30000; // 30 seconds
const int      IDLE_TIMEOUT        = 60000; // 1 minute
const uint8_t  DEBOUNCE_DELAY      = 50;    // If a change is seen, delay by this much and let things settle.

// All start bytes are followed by 0x01 SOH
const uint8_t REQUEST_START_BYTE_PING_COMMAND = 0x07;           // BEL: For making a request or sending out a ping.
const uint8_t RESPONSE_START_BYTE             = 0x06;           // ACK: Acknowledge.
const uint8_t FAILED_START_BYTE               = 0x15;           // NAK: Negative Ackowledge
const uint8_t STOP_BYTES[2]                   = { 0x17, 0x04 }; // ETB EOT: End of Transmission Block,
                                                                // End Of Transmission.
const uint8_t READ_COMMAND                    = 0x05;           // ENQ - Enquiry
const uint8_t WRITE_COMMAND                   = 0x1A;           // SUB - Substitute
const uint8_t START_BYTE_2                    = 0x01;           // SOH: Second start byte of every frame.
const uint8_t FRAME_OVERHEAD                  = 7;              // 2 start + command + variable + length + 2 stop.
const uint8_t MAX_DATA_LENGTH                 = 32;
const uint8_t BUFFER_LENGTH                   = FRAME_OVERHEAD + MAX_DATA_LENGTH; // 39 bytes
const uint8_t INVERT_MUTE_CMD[8]              = { 0x07, 0x01, 0x1A, 0x00, 0x01, 0x02, 0x17, 0x04 };

// LED brightness constants: Be mindful of power usage.
const uint8_t MIN_BRIGHTNESS = 8;
const uint8_t MAX_BRIGHTNESS = 64;
// End LED brightness constants.

uint8_t  receiveBuffer[BUFFER_LENGTH];               // Buffer for our serial data.
uint32_t currentTime           = 0;                  // Save the running time in milliseconds.
uint32_t lastCommunicationTime = 0;                  // Save the last time we had a successful incoming communication.
uint32_t lastPingTime          = 0;                  // Save the last time we sent a ping.
uint32_t pingInterval          = PING_INTERVAL_LONG; // Current ping interval to use.

// Create a bitfield to hold booleans and super small values.
struct Flags {
    uint8_t hasData               : 1;
    uint8_t hasError              : 1;
    uint8_t isStartupMode         : 1;
    uint8_t previousButtonState   : 1;
    uint8_t currentButtonState    : 1;
    uint8_t previousLedState      : 2;
    uint8_t currentLedState       : 2;
} flags;

// Define the array of leds, we only have 1.
CRGB leds[1];

// Prototype function.
void doPing();
bool isStartByte(uint8_t value);
void processButton();
void processData();
bool processFailedResponseFrame();
bool processRequestFrame();
bool processResponseFrame();
void receiveSerialData();
void requestMuteStatus();
void sendPingRequest();
void setLed();

// Initial setup, called once.
void setup() {
    // Set up the serial port, so we can talk and list to the host.
    Serial.begin(115200);
    while (!Serial)
        ; // wait for serial port to connect. Needed for native USB

    pinMode(BUTTON_PIN_NO, INPUT_PULLUP); // Set the button pin to be an input with pullup, which will have less noise.
    pinMode(LED_DATA_PIN,  OUTPUT);       // Set the LED pin to be an output of course.

    // Set up the LED.
    FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, 1);
    FastLED.setBrightness(MIN_BRIGHTNESS);
    FastLED.show();
    FastLED.clear();
    leds[0] = CRGB::Blue;
    FastLED.setBrightness(MAX_BRIGHTNESS);
    FastLED.show();

    // Set a couple bits that need to be 0x1 / true / HIGH
    // These are equal to 0x1, but help differentiate between their usage.
    flags.isStartupMode       = true;
    flags.previousButtonState = HIGH;
    flags.currentButtonState  = HIGH;
    flags.previousLedState    = B01;
    flags.currentLedState     = B01;
}

// Main loop, called every time.
void loop() {
    // Save the current run time in milliseconds.
    currentTime = millis();

    // Deal with possible millis() overflow: https://forum.arduino.cc/t/resetting-millis-to-zero-reset-clock/180147
    if (lastPingTime > currentTime) {
        lastPingTime = 0;
    }

    if (lastCommunicationTime > currentTime) {
        lastCommunicationTime = 0;
    }

    // Do all the processes.
    receiveSerialData();
    processData();
    doPing();
    setLed();
    processButton();
}

// "Broadcast" our identification in the hopes that the host will hear us.
void doPing() {
    // Compare time difference between last communication and now.
    // Determine if we should have a long interval or short interval between pings.
    // Maybe we don't need to send a ping yet?
    unsigned long lastCommTimeDiff = currentTime - lastCommunicationTime;
    bool          fast             = lastCommTimeDiff >= IDLE_TIMEOUT || flags.isStartupMode;
    bool          slow             = !fast && lastCommTimeDiff >= RESPONSE_TIMEOUT;

    if (slow || fast) {
        // Check for idle timeout
        if (fast && pingInterval != PING_INTERVAL_SHORT) {
            pingInterval = PING_INTERVAL_SHORT;
        } else if (slow && pingInterval == PING_INTERVAL_SHORT) {
            pingInterval = PING_INTERVAL_LONG;
        }

        if (currentTime - lastPingTime >= pingInterval) {
            if (fast) {
                // If it's a short interval, that means it's been a long time since we had a successful communication.
                // We should show this in some sort of visual form to the button pusher.
                flags.currentLedState = B01;
            }

            // Send a ping request, and update the last ping time.
            sendPingRequest();
            lastPingTime = currentTime;
        }
    }
}


/**
 * @brief Process the button state.
 */
void processButton() {
    flags.currentButtonState = digitalRead(BUTTON_PIN_NO);

    if (flags.previousButtonState != flags.currentButtonState) {
        // Poor man's debounce: Change was seen, delay by N.
        delay(DEBOUNCE_DELAY);
        // Read the value again and see if we can continue on.
        if (flags.currentButtonState != digitalRead(BUTTON_PIN_NO)) {
            // Not the same, so return.
            return;
        }
    }

    if (LOW == flags.currentButtonState && HIGH == flags.previousButtonState) {
        flags.previousButtonState = LOW;
        Serial.write((uint8_t *)&INVERT_MUTE_CMD, 8);
    } else if (HIGH == flags.currentButtonState && LOW == flags.previousButtonState) {
        flags.previousButtonState = HIGH;
    }
}

/**
 * @brief Process the incoming serial data, if there is any.
 */
void processData() {
    flags.hasError = false;

    if (flags.hasData) {
        // Dispatch on the frame's own type byte. The host can send a request (e.g. a mute state change)
        // while we are waiting on a ping / read response, so we can't assume the next frame is the response.
        switch (receiveBuffer[0]) {
            case REQUEST_START_BYTE_PING_COMMAND:
                flags.hasError = processRequestFrame();
                break;

            case RESPONSE_START_BYTE:
                flags.hasError = processResponseFrame();
                break;

            case FAILED_START_BYTE:
                flags.hasError = processFailedResponseFrame();
                break;

            default:
                // receiveSerialData() only accepts the start bytes above, so we shouldn't get here.
                flags.hasError = true;
        }

        if (!flags.hasError) {
            // Update last communication time.
            lastCommunicationTime = currentTime;
            if (flags.isStartupMode) {
                flags.isStartupMode = false;
            }
        }
    }
}

/**
 * @brief Process a failed response frame. I this case we don't care and we just continue on.
 *
 * @return bool
 */
bool processFailedResponseFrame() {
    return false;
}

/**
 * @brief Proccess a request frame.
 *
 * @return bool
 */
bool processRequestFrame() {
    // Command
    switch (receiveBuffer[2]) {
        case 0x05:
            // Read Request
            // We have nothing to read.
            break;

        case 0x1A:
            // Write Request
            if (receiveBuffer[3] == 0x00) {
                if (receiveBuffer[5] == 0x00) {
                    // Setting LED to indicate unmuted state.
                    flags.currentLedState = B10;
                } else if (receiveBuffer[5] == 0x01) {
                    // Setting LED to indicate muted state.
                    flags.currentLedState = B11;
                }
            }
            break;

        case 0x07:
        default:
            // Device shouldn't be recieving a ping requests...
            return true;
    }

    return false;
}

/**
 * @brief Process a response frame.
 *
 * @return bool
 */
bool processResponseFrame() {
    // Command
    switch (receiveBuffer[2]) {
        case 0x05:
            // Response from a read request.
            // This is the for the mute status.
            if (receiveBuffer[3] == 0x00) {
                if (receiveBuffer[5] == 0x00) {
                    // Setting LED to indicate unmuted state.
                    flags.currentLedState = B10;
                } else if (receiveBuffer[5] == 0x01) {
                    // Setting LED to indicate muted state.
                    flags.currentLedState = B11;
                }
            }
            // Get
            break;

        case 0x1A:
            // Response from a write request.
            // Nothing here.
            break;

        case 0x07:
            // Repsonse to a ping request.
            if (receiveBuffer[4] != 0x0C) {
                // Bad repsonse, exit out of function.
                return true;
            }

            // Request the mute status from the host.
            requestMuteStatus();
            break;
    }

    return false;
}

/**
 * @brief Is this byte one of the first start bytes (request, response or failed response)?
 *
 * @return bool
 */
bool isStartByte(uint8_t value) {
    return REQUEST_START_BYTE_PING_COMMAND == value || RESPONSE_START_BYTE == value || FAILED_START_BYTE == value;
}

/**
 * @brief Read data from the serial port to a buffer to be processed later.
 *
 * The end of a frame is found using the data length byte (offset 4), not by scanning for the stop bytes.
 * The protocol has no escaping, so data that happens to contain 0x17 0x04 would otherwise end the frame early.
 * The stop bytes are then checked at the position the length says they should be.
 */
void receiveSerialData() {
           uint8_t receivedByte;
    static uint8_t index = 0; // Number of bytes of the current frame we have so far.
                   flags.hasData = false;

    // Check for incoming serial data, as long as we already don't have data.
    while (Serial.available() > 0 && flags.hasData == false) {
        // Read one byte from the serial.
        receivedByte = Serial.read();

        if (0 == index) {
            // Waiting for the first start byte.
            if (!isStartByte(receivedByte)) {
                // Let's continue one and stop the LED flicker.
                return;
            }
        } else if (1 == index) {
            // Waiting for the second start byte.
            if (START_BYTE_2 != receivedByte) {
                // Not a valid second start byte. This byte might be the first start byte of a new frame though.
                index = 0;
                if (isStartByte(receivedByte)) {
                    receiveBuffer[index++] = receivedByte;
                }

                continue;
            }
        } else if (4 == index && receivedByte > MAX_DATA_LENGTH) {
            // Data length is out of range, so this isn't a valid frame. Start over.
            index = 0;
            continue;
        }

        receiveBuffer[index++] = receivedByte;

        // Once we have the length byte, we know how long the whole frame is.
        if (index >= FRAME_OVERHEAD && index == FRAME_OVERHEAD + receiveBuffer[4]) {
            index = 0;

            if (STOP_BYTES[0] == receiveBuffer[FRAME_OVERHEAD + receiveBuffer[4] - 2] &&
                STOP_BYTES[1] == receiveBuffer[FRAME_OVERHEAD + receiveBuffer[4] - 1]) {
                // Got a full frame, so we can process the buffer.
                flags.hasData = true;
            }
            // Otherwise the stop bytes weren't where they should be. Drop the frame and wait for the next start byte.
        }
    }
}

/**
 * @brief Send a read request to the host to retrieve the mute status.
 */
void requestMuteStatus() {
    // Build read request frame.
    uint8_t muteStatusEnquiry[8] = {
        REQUEST_START_BYTE_PING_COMMAND,
        0x01,
        READ_COMMAND,
        0x00,
        0x01,
        0x00,
        STOP_BYTES[0],
        STOP_BYTES[1]
    };

    Serial.write(muteStatusEnquiry, 8);
}

/**
 * @brief 'Broadcast' a ping request to the host.
 */
void sendPingRequest() {
    // Build ping request frame.
    byte pingRequest[19] = {
        REQUEST_START_BYTE_PING_COMMAND,
        0x01,
        REQUEST_START_BYTE_PING_COMMAND,
        0x00,
        12,
        VENDOR_ID[0],
        VENDOR_ID[1],
        VENDOR_ID[2],
        VENDOR_ID[3],
        PRODUCT_ID[0],
        PRODUCT_ID[1],
        PRODUCT_ID[2],
        PRODUCT_ID[3],
        SERIAL_NUM[0],
        SERIAL_NUM[1],
        SERIAL_NUM[2],
        SERIAL_NUM[3],
        STOP_BYTES[0],
        STOP_BYTES[1]
    };

    // Send it off to the world.
    Serial.write(pingRequest, 19);
}

/**
 * @brief Set the LED color / effect based on state flag.
 */
void setLed() {
    if (flags.previousLedState == flags.currentLedState && flags.currentLedState != B01) {
        if (FastLED.getBrightness() != MAX_BRIGHTNESS) {
            // Make ure we are at max brightness.
            FastLED.setBrightness(MAX_BRIGHTNESS);
            FastLED.show();
        }

        // Nothing else to change, so exit out of function.
        return;

    }

    float brightness       = MAX_BRIGHTNESS;
    flags.previousLedState = flags.currentLedState;

    switch (flags.currentLedState) {
        case B01: {
            // Pining state.
            // Yes, this needs to be surrounded by a block, if it's not it doesn't really run. Not sure why though.
            leds[0] = CRGB::Blue;
            // Fast ping: do breathing effect.
            brightness = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
            brightness = map(brightness, 0, 255, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
            break;
        }
        case B10:
            // Unmuted state.
            leds[0] = CRGB::Green;
            brightness = MAX_BRIGHTNESS;
            break;
        case B11:
            // Muted state.
            leds[0] = CRGB::Red;
            brightness = MAX_BRIGHTNESS;
            break;
        default:
            // Unknown state.
            leds[0] = CRGB::DarkOrange;
            brightness = MAX_BRIGHTNESS;
    }

    FastLED.setBrightness(brightness);
    FastLED.show();
}
