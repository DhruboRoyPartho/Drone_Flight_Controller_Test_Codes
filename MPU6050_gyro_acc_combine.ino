// Project: Drone Flight Controller - Gyro & Acc meter combine with Kalman filter
// Code Author: Dhrubo Roy Partho
// Date: 25/05/2024
// Version: 1.0v

#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

float RateCalibrationNumber, RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

int16_t AccXLSB;
int16_t AccYLSB;
int16_t AccZLSB;

float AccXCalibrationRate = 0, AccYCalibrationRate = 0, AccZCalibrationRate = 0;

uint32_t LoopTimer;

// Kalman 1D filter
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[] = {0, 0};    //1st one is angle prediction 2nd one is
                                    // uncertainty prediction

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    // KalmanInput = rotation rate
    // KalmanMeasurement = accelerometer angle
    // KalmanState = angle calculated with the Kalman filter

    KalmanState = KalmanState + 0.004*KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty + 3*3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty = (1-KalmanGain) * KalmanUncertainty;

    Kalman1DOutput[0] = KalmanState;    // Kalman filter output
    Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals() {
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    // Accelerometer config
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();

    // Gyro data read
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 6);

    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    RateRoll = (float)GyroX/65.5;
    RatePitch = (float)GyroY/65.5;
    RateYaw = (float)GyroZ/65.5;

    // Accelerometer data read
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 6);
    AccXLSB = Wire.read() << 8 | Wire.read();
    AccYLSB = Wire.read() << 8 | Wire.read();
    AccZLSB = Wire.read() << 8 | Wire.read();

    AccX = (float)AccXLSB/4096; //- 0.16;
    AccY = (float)AccYLSB/4096; //- 0.07;
    AccZ = (float)AccZLSB/4096; //- 0.99;

    AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(3.142/180);
    AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(3.142/180);
}

void setup() {
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Wire.setClock(400000);
    Wire.begin();
    delay(250);

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    for(RateCalibrationNumber=0;RateCalibrationNumber<2000;RateCalibrationNumber++){
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;

        AccXCalibrationRate += AccX;
        AccYCalibrationRate += AccY;
        AccZCalibrationRate += AccZ;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;

    AccXCalibrationRate /= 2000;
    AccYCalibrationRate /= 2000;
    AccZCalibrationRate /= 2000;

    LoopTimer = micros();

    Serial.print(RateCalibrationRoll); Serial.print(" ");
    Serial.print(RateCalibrationPitch); Serial.print(" ");
    Serial.print(RateCalibrationYaw); Serial.print(" ");
    Serial.print(AccXCalibrationRate); Serial.print(" ");
    Serial.print(AccYCalibrationRate); Serial.print(" ");
    Serial.println(AccZCalibrationRate);

    delay(2000);
}

void loop() {
    gyro_signals();

    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;


    // Kalman filter
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];




    // Print Section

    // Serial.print("Roll rate[0/s]: ");
    // Serial.print(RateRoll);
    // Serial.print(" Pitch rate[0/s]: ");
    // Serial.print(RatePitch);
    // Serial.print(" Yaw rate[0/s]: ");
    // Serial.println(RateYaw);

    // Serial.print("Acceleration X[g]= ");
    // Serial.print(AccX);
    // Serial.print(" Acceleration Y[g]= ");
    // Serial.print(AccY);
    // Serial.print(" Acceleration Z[g]= ");
    // Serial.println(AccZ);

    Serial.print("Roll Angle= ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" Pitch Angle= ");
    Serial.println(KalmanAnglePitch);

    while(micros() - LoopTimer < 4000);
    LoopTimer = micros();
}
