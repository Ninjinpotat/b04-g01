/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"

volatile unsigned long lastInterruptTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50 ms
volatile uint32_t edgeCount = 0;
volatile uint8_t timerDone = 0;

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */

// INT1 corresponds to PD1
ISR(INT1_vect) {
    unsigned long currentInterruptTime = millis();
    if (currentInterruptTime - lastInterruptTime > DEBOUNCE_DELAY) {
        bool isPressed = (PIND & (1 << 1));
        if (isPressed) {
            // Toggle the state on press
            buttonState = (buttonState == STATE_RUNNING) ? STATE_STOPPED : STATE_RUNNING;
            stateChanged = true;
        }
        lastInterruptTime = currentInterruptTime;
    }
}

// =============================================================
// Color sensor (TCS3200)
// =============================================================

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */
static void initTimer1() {
    TCCR1A = 0;
    TCCR1B |= (1 << WGM12);              // CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64
    OCR1A = 24999;
    TIMSK1 = (1 << OCIE1A);             // enable compare interrupt
}

static void initColorSensorPins() {
    DDRA |= (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);

    /* Frequency scaling: 20% */
    PORTA |= (1 << PA0);   // S0 HIGH
    PORTA &= ~(1 << PA1);  // S1 LOW
}

static void initEdgeInterrupt() {
    EICRB |= (1 << ISC41) | (1 << ISC40);  // rising edge trigger
    EIMSK |= (1 << INT4);                  // enable INT4
}

ISR(INT4_vect) {
    edgeCount++;
}

ISR(TIMER1_COMPA_vect) {
    timerDone = 1;
}

static uint32_t measureChannel(uint8_t s2, uint8_t s3) {
    /* Set S2 */
    if (s2)
        PORTA |= (1 << PA2);
    else
        PORTA &= ~(1 << PA2);
    /* Set S3 */
    if (s3)
        PORTA |= (1 << PA3);
    else
        PORTA &= ~(1 << PA3);
    _delay_ms(5);  // allow sensor to stabilise
    edgeCount = 0;
    timerDone = 0;
    TCNT1 = 0;     // reset timer
    while (!timerDone);   // wait for 100 ms
    return edgeCount;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    // Set S2/S3 for each channel, measure edge count, multiply by 10
    *r = measureChannel(0, 0) * 10;  // red,   in Hz
    *g = measureChannel(1, 1) * 10;  // green, in Hz
    *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 }

// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            break;

        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.
        case COMMAND_COLOR:
            {
                TPacket pkt = {0};
                //memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_COLOR;
                uint32_t r,g,b;
                readColorChannels(&r, &g, &b);
                pkt.params[0] = r;
                pkt.params[1] = g;
                pkt.params[2] = b;
                //strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                //pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
        break;
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // TODO (Activity 1): configure the button pin and its external interrupt,
    // then call sei() to enable global interrupts.
    DDRD &= ~(1 << 1); //set PD1 as input
    EICRA = 0b00000100; //trigger INT1 on any logical change
    EIMSK = 0b00000010; //enable INT1
    initEdgeInterrupt();
    initTimer1();
    initColorSensorPins();
    sei();
}

void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
