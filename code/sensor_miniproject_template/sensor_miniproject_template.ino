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
#include "robotlib.h"

volatile unsigned long lastInterruptTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms between button presses
volatile uint8_t buttonPhase = 0; // for the button
volatile uint32_t edgeCount = 0;  // for the color sensor
volatile uint8_t timerDone = 0;   // also for the color sensor
unsigned long speed = 150; // (default) motor speed


// =============================================================
// SERVO ARM consts
// =============================================================

#define STEP_TICKS 11 //calculated for 1 degree increase
unsigned long lastStep = 0;
int msPerDeg = 10; // (default) servo speed

// Updated to the PORTK analog pins on the Mega
const int BASE_PIN     = A8;  // PK0
const int SHOULDER_PIN = A9;  // PK1
const int ELBOW_PIN    = A10; // PK2
const int GRIPPER_PIN  = A11; // PK3

int stagecount = 0;

int volatile curr_state[]   = {2300,3666,3688,1750};
int volatile target_state[] = {2300,3666,3688,1750};

// Define checkpoints for each servo (spacing them out in the 20ms period)
#define B_CHECKPOINT 0
#define S_CHECKPOINT 10000
#define E_CHECKPOINT 20000
#define G_CHECKPOINT 30000

// Ensure you have the sys_ms variable defined at the top of your file
// from our bare-metal Timer 2 clock!
extern volatile unsigned long sys_ms = 0; 

// =============================================================
// Packet helpers
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

ISR(INT1_vect) {
    unsigned long currentInterruptTime = millis();
    if (currentInterruptTime - lastInterruptTime > DEBOUNCE_DELAY) {
        
        // Button reads 0 when pushed DOWN, 1 when released UP
        bool isPressed = ((PIND & (1 << 1)) == 0); 
        
        if (isPressed) { 
            // The button was just PUSHED DOWN
            if (buttonPhase == 0) {
                // Phase 1: First Press -> STOP the system
                buttonState = STATE_STOPPED;
                stateChanged = true;
                buttonPhase = 1;
            } 
            else if (buttonPhase == 2) {
                // Phase 3: Second Press -> Do nothing to the system, just update phase
                buttonPhase = 3;
            }
        } 
        else { 
            // The button was just RELEASED UP
            if (buttonPhase == 1) {
                // Phase 2: First Release -> Do nothing to the system, just update phase
                buttonPhase = 2;
            } 
            else if (buttonPhase == 3) {
                // Phase 0: Second Release -> START the system
                buttonState = STATE_RUNNING;
                stateChanged = true;
                buttonPhase = 0;
            }
        }
        
        lastInterruptTime = currentInterruptTime;
    }
}

// =============================================================
// Color sensor (TCS3200) - BARE METAL ON TIMER 2
// =============================================================
// Non-blocking Color Sensor Variables
enum ColorState { COLOR_IDLE, COLOR_READ_R, COLOR_READ_G, COLOR_READ_B };
ColorState currentColorState = COLOR_IDLE;
uint32_t red_val = 0, green_val = 0, blue_val = 0;
bool colorContinuous = true; // Set to true to start sensing immediately

// Variables to handle the 8-bit timer tracking 100ms
volatile uint8_t color_window_active = 0;
volatile uint8_t color_ms_count = 0;

static void initTimer2_ColorSensor() {
    // Configure Timer 2 (8-bit) for CTC mode to fire every 1ms
    TCCR2A = (1 << WGM21); // CTC mode
    TCCR2B = (1 << CS22);  // Prescaler 64
    
    // 16MHz / 64 = 250,000 ticks per second. 
    // 250 ticks = 1 millisecond. (250 - 1 = 249)
    OCR2A = 249;           
    
    TIMSK2 = (1 << OCIE2A); // Enable compare match interrupt
}

static void initColorSensorPins() {
    DDRA |= (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);

    /* Frequency scaling: 20% */
    PORTA |= (1 << PA0);   // S0 HIGH
    PORTA &= ~(1 << PA1);  // S1 LOW
}

static void initEdgeInterrupt() {
    EICRA |= (1 << ISC21) | (1 << ISC20);  // rising edge trigger
    EIMSK |= (1 << INT2);                  // enable INT2 (Pin 19)
}

// Counts the frequency pulses from the sensor
ISR(INT2_vect) {
    edgeCount++;
}

// Timer 2 Interrupt - Fires exactly once every 1 millisecond
ISR(TIMER2_COMPA_vect) {
    sys_ms++;
    if (color_window_active) {
        color_ms_count++;
        if (color_ms_count >= 100) {  // 100ms window reached!
            timerDone = 1;
            color_window_active = 0;  // Stop counting
        }
    }
}

static void startMeasurement(uint8_t s2, uint8_t s3) {
    /* Set S2 */
    if (s2) PORTA |= (1 << PA2);
    else    PORTA &= ~(1 << PA2);
    /* Set S3 */
    if (s3) PORTA |= (1 << PA3);
    else    PORTA &= ~(1 << PA3);

    // Reset counters and start the hardware timer window
    edgeCount = 0;
    timerDone = 0;
    color_ms_count = 0;
    color_window_active = 1; 
}

static void updateContinuousColor() {
    if (!colorContinuous) return;

    switch (currentColorState) {
        case COLOR_IDLE:
            startMeasurement(0, 0); // Start measuring Red
            currentColorState = COLOR_READ_R;
            break;

        case COLOR_READ_R:
            if (timerDone) {
                red_val = edgeCount * 10;
                startMeasurement(1, 1); // Start measuring Green
                currentColorState = COLOR_READ_G;
            }
            break;

        case COLOR_READ_G:
            if (timerDone) {
                green_val = edgeCount * 10;
                startMeasurement(0, 1); // Start measuring Blue
                currentColorState = COLOR_READ_B;
            }
            break;

        case COLOR_READ_B:
            if (timerDone) {
                blue_val = edgeCount * 10;
                
                // All 3 channels are done. Send the packet to Pi!
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_COLOR;
                pkt.params[0]  = red_val;
                pkt.params[1]  = green_val;
                pkt.params[2]  = blue_val;
                sendFrame(&pkt);

                // Reset to IDLE to start the next cycle automatically
                currentColorState = COLOR_IDLE; 
            }
            break;
    }
}

// =============================================================
// SERVO ARM
// =============================================================

int parse3 (const String *s) {
  if (!s) return -1;
  if (s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return (s->charAt(0) - '0') * 100 + (s->charAt(1) - '0') * 10 + (s->charAt(2) - '0');
}

void updateSmoothMotion() {
  unsigned long now = millis(); 
  if (now - lastStep < msPerDeg) {return;}
   lastStep = now;
  // Disable interrupts temporarily for safe array updating
  //cli(); 
  for (int k = 0; k < 4; k++) {
    if (curr_state[k] < target_state[k]) {
      curr_state[k] += STEP_TICKS;
      if (curr_state[k] > target_state[k])
        curr_state[k] = target_state[k];
    }
    else if (curr_state[k] > target_state[k]) {
      curr_state[k] -= STEP_TICKS;
      if (curr_state[k] < target_state[k])
        curr_state[k] = target_state[k];
    }
  }
//   sei(); 
}

// CHANGED TO TIMER 5 and PORT K
ISR(TIMER5_COMPB_vect) {
 
  switch (stagecount) {
    case 0:
      // Turn ON base servo (PORTK bit 0 is A8)
      PORTK |= (1 << 0);
      OCR5B += curr_state[0];
      break;
     
    case 1:
      // Turn OFF base servo
      PORTK &= ~(1 << 0);
      OCR5B = S_CHECKPOINT;
      break;
     
    case 2:
      // Turn ON shoulder servo (PORTK bit 1 is A9)
      PORTK |= (1 << 1);
      OCR5B += curr_state[1];
      break;
     
    case 3:
      // Turn OFF shoulder servo
      PORTK &= ~(1 << 1);
      OCR5B = E_CHECKPOINT;
      break;
     
    case 4:
      // Turn ON elbow servo (PORTK bit 2 is A10)
      PORTK |= (1 << 2);
      OCR5B += curr_state[2];
      break;
     
    case 5:
      // Turn OFF elbow servo
      PORTK &= ~(1 << 2);
      OCR5B = G_CHECKPOINT;
      break;
     
    case 6:
      // Turn ON gripper servo (PORTK bit 3 is A11)
      PORTK |= (1 << 3);
      OCR5B += curr_state[3];
      break;
     
    case 7:
      // Turn OFF gripper servo
      PORTK &= ~(1 << 3);
      OCR5B = B_CHECKPOINT;
      stagecount = -1; // Will become 0 after increment
      break;
  }
  stagecount++;
}

// CHANGED TO TIMER 5
ISR(TIMER5_COMPA_vect) {
  //updateSmoothMotion();
}

// Call this from your main setup() function
void initArmTimer5() {
  // Set PK0, PK1, PK2, PK3 as outputs
  DDRK |= 0b00001111;
 
  // Clear all servo pins to LOW
  PORTK &= ~0b00001111;
 
  cli(); // Disable interrupts during setup
 
  // Use CTC mode instead of Fast PWM, mapped to Timer 5
  TCCR5A = 0b00000000;  // No PWM output pins
  TCCR5B = 0b00001010;  // WGM52=1 (CTC), CS51=1 (prescaler=8)
 
  // OCR5A is now TOP (defines the 20ms period)
  OCR5A = 39999;  // 20ms period at 16MHz/8 = 2MHz (0.5us per tick)
  OCR5B = 0;      // Start at beginning
 
  TCNT5 = 0;
 
  // Enable Compare Match A and B interrupts for Timer 5
  TIMSK5 = 0b00000110;
 
  sei(); // Enable interrupts
}

// =============================================================
// Command handler
// =============================================================

dir lastMove = STOP;
unsigned long lastMoveTime = 0;
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            if (buttonState == STATE_STOPPED) {
                // STARTING THE ROBOT VIA SOFTWARE
                buttonState = STATE_RUNNING;
                buttonPhase = 0; // WAKE UP: Reset hardware memory to Phase 0
            } else {
                // STOPPING THE ROBOT VIA SOFTWARE
                buttonState = STATE_STOPPED;
                buttonPhase = 2; // SLEEP: Fast-forward hardware memory to Phase 2 (Pretend it was pressed & released)
            }
            stateChanged = false;
            sei();
            
            {
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
            }
            sendStatus(buttonState);
            
            // Safety: Only hit the brakes if we just STOPPED the robot!
            if (buttonState == STATE_STOPPED) {
                stop(); 
            }
            break;

        case COMMAND_COLOR:
            {
                // Toggle the continuous sensing on/off
                colorContinuous = !colorContinuous; 
                
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
            }
            break;
        
        case COMMAND_W:
            {   
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_MOVEMENT;

                speed = 150; // Set speed for Forward
                pkt.params[0] = speed;
                strncpy(pkt.data, "Forwards", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';

                forward(speed);
                lastMove = GO;
                lastMoveTime = millis(); // Start the movement timer
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
            break;
        
        case COMMAND_A:
            {   
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_MOVEMENT;

                speed = 250; // Set speed for Left Turn
                pkt.params[0] = speed;
                strncpy(pkt.data, "Left turn", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';

                cw(speed);
                lastMove = CW;
                lastMoveTime = millis(); // Start the movement timer
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
            break;
        
        case COMMAND_S:
            {   
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_MOVEMENT;

                speed = 150; // Set speed for Backwards
                pkt.params[0]  = speed;
                strncpy(pkt.data, "Backwards", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';

                backward(speed);
                lastMove = BACK;
                lastMoveTime = millis(); // Start the movement timer
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
            break;
        
        case COMMAND_D:
            {   
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_MOVEMENT;

                speed = 250; // Set speed for Right Turn
                pkt.params[0] = speed;
                strncpy(pkt.data, "Right turn", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';

                ccw(speed);
                lastMove = CCW;
                lastMoveTime = millis(); // Start the movement timer
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
            break;
        
        case COMMAND_PLUS:
            {   
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_MOVEMENT;

                speed += 10;
                speed = constrain(speed, 0, 255);

                pkt.params[0] = speed;
                strncpy(pkt.data, "Increasing speed by 10", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';

                // switch (lastMove) { //execute last movement with updated speed
                //     case GO:
                //         forward(speed);
                //         break;
                //     case BACK:
                //         backward(speed);
                //         break;
                //     case CCW:
                //         ccw(speed);
                //         break;
                //     case CW:
                //         cw(speed);
                //         break;
                //     case STOP: //do nothing
                //         break;
                // }
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
            break;
        
        case COMMAND_MINUS:
            {   
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_MOVEMENT;

                speed -= 10;
                speed = constrain(speed, 0, 255);
                
                pkt.params[0] = speed;
                strncpy(pkt.data, "Decreasing speed by 10", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';

                // switch (lastMove) { //execute last movement with updated speed
                //     case GO:
                //         forward(speed);
                //         break;
                //     case BACK:
                //         backward(speed);
                //         break;
                //     case CCW:
                //         ccw(speed);
                //         break;
                //     case CW:
                //         cw(speed);
                //         break;
                //     case STOP: //do nothing
                //         break;
                // }
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
            break;
        
        case COMMAND_STOP:
            {   
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_MOVEMENT;
                //strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                //pkt.data[sizeof(pkt.data) - 1] = '\0';
                stop();
                sendFrame(&pkt);
            }
            sendStatus(STATE_RUNNING);
            break;
        
        case COMMAND_ARM_HOME:
            {
                target_state[0] = 2800;
                target_state[1] = 2600;
                target_state[2] = 3688;
                target_state[3] = 1750;
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
                break;
            }
        
        case COMMAND_ARM_BASE:
            {
                int deg = constrain(cmd->params[0], 0, 180);
                target_state[0] = 1600 + (deg / 180.0) * 3000;
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
                break;
            }
        
        case COMMAND_ARM_SHOULDER:
            {
                int deg = constrain(cmd->params[0], 0, 180);
                target_state[1] = 2000 + (deg / 180.0) * 3110;
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
                break;
            }
        
        case COMMAND_ARM_ELBOW:
            {
                int deg = constrain(cmd->params[0], 40, 140);
                target_state[2] = 2300 + (deg / 180.0) * 2777;
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
                break;
            }
        
        case COMMAND_ARM_GRIPPER:
            {
                int deg = constrain(cmd->params[0], 0, 150);
                target_state[3] = 1450 + (deg / 180.0) * 600;
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
                break;
            }
        
        case COMMAND_ARM_SPEED:
            {
                msPerDeg = constrain(cmd->params[0], 20, 50);
                TPacket pkt = {0};
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                sendFrame(&pkt);
                break;
            }
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    usartInit(103); 

    // E-stop button setup
    DDRD &= ~(1 << 1);  //set PD1 as input
    PORTD |= (1 << 1);  //Enable internal pull-up resistor!
    EICRA = 0b00000100; //trigger INT1 on any logical change
    EIMSK = 0b00000010; //enable INT1
    
    // --- ADD THESE LINES HERE ---
    initEdgeInterrupt();
    initColorSensorPins();
    initTimer2_ColorSensor();

    //Other setups
    // 
    initArmTimer5();
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
    // --- 3. Auto-Stop Timer for Discrete Movements ---
    // If moving, automatically stop after 300 milliseconds
    if (lastMove != STOP && (millis() - lastMoveTime > 300)) {
        stop();
        lastMove = STOP;
    }
    updateSmoothMotion();
    updateContinuousColor(); // Add this line!
}
