#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Setup for NRF24
RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001";

// Joystick Pins
#define JOYSTICK1_X A0
#define JOYSTICK1_Y A1
#define JOYSTICK2_X A2
#define JOYSTICK2_Y A3

// Button Pins
#define BUTTON1 2
#define BUTTON2 3
#define BUTTON3 4
#define BUTTON4 5

// Array to hold channel values
int channels[8];

// Function to read joystick values
void readJoysticks() {
    channels[0] = analogRead(JOYSTICK1_X); // Read Joystick 1 X-axis
    channels[1] = analogRead(JOYSTICK1_Y); // Read Joystick 1 Y-axis
    channels[2] = analogRead(JOYSTICK2_X); // Read Joystick 2 X-axis
    channels[3] = analogRead(JOYSTICK2_Y); // Read Joystick 2 Y-axis

    // Map the analog values (0-1023) to PPM pulse widths (1000-2000 microseconds)
    for (int i = 0; i < 4; i++) {
        channels[i] = map(channels[i], 0, 1023, 1000, 2000);
    }
}

// Function to read button states
void readButtons() {
    channels[4] = digitalRead(BUTTON1) == HIGH ? 2000 : 1000; // Channel 5
    channels[5] = digitalRead(BUTTON2) == HIGH ? 2000 : 1000; // Channel 6
    channels[6] = digitalRead(BUTTON3) == HIGH ? 2000 : 1000; // Channel 7
    channels[7] = digitalRead(BUTTON4) == HIGH ? 2000 : 1000; // Channel 8
}

void setup() {
    Serial.begin(9600);
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_LOW);
    radio.stopListening();

    // Initialize button pins
    pinMode(BUTTON1, INPUT_PULLUP);
    pinMode(BUTTON2, INPUT_PULLUP);
    pinMode(BUTTON3, INPUT_PULLUP);
    pinMode(BUTTON4, INPUT_PULLUP);
}

void loop() {
    readJoysticks();
    readButtons();
    radio.write(&channels, sizeof(channels));
    delay(20); // 50Hz transmission rate
}
