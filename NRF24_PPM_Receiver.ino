#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Setup for NRF24
RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001";

// PPM output pin
#define PPM_PIN 3

// PPM signal parameters
#define PPM_FRAME_LENGTH 22500 // Total frame length in microseconds (22.5ms for ~44Hz)
#define PPM_PULSE_LENGTH 300   // Pulse length in microseconds
#define CHANNEL_NUMBER 8       // Number of channels

// PPM values array
int channels[CHANNEL_NUMBER];

// Timing variables
unsigned long lastFrameTime = 0;
unsigned int ppm[CHANNEL_NUMBER + 1]; // Includes sync pulse

void setup() {
    Serial.begin(9600);
    pinMode(PPM_PIN, OUTPUT);
    digitalWrite(PPM_PIN, LOW);

    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_LOW);
    radio.startListening();
}

void loop() {
    if (radio.available()) {
        radio.read(&channels, sizeof(channels));

        // For debugging: Print PPM values to Serial Monitor
        for (int i = 0; i < CHANNEL_NUMBER; i++) {
            Serial.print("Channel ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(channels[i]);
        }

        // Generate PPM signal
        generatePPM();
    }
}

void generatePPM() {
    unsigned long currentTime = micros();
    if (currentTime - lastFrameTime >= PPM_FRAME_LENGTH) {
        lastFrameTime = currentTime;

        for (int i = 0; i < CHANNEL_NUMBER; i++) {
            ppm[i] = channels[i];
        }
        ppm[CHANNEL_NUMBER] = PPM_FRAME_LENGTH - (CHANNEL_NUMBER * PPM_PULSE_LENGTH) - sumArray(channels, CHANNEL_NUMBER);

        // Output PPM signal
        for (int i = 0; i < CHANNEL_NUMBER + 1; i++) {
            digitalWrite(PPM_PIN, HIGH);
            delayMicroseconds(PPM_PULSE_LENGTH);
            digitalWrite(PPM_PIN, LOW);
            delayMicroseconds(ppm[i]);
        }
    }
}

unsigned int sumArray(int arr[], int length) {
    unsigned int sum = 0;
    for (int i = 0; i < length; i++) {
        sum += arr[i];
    }
    return sum;
}
