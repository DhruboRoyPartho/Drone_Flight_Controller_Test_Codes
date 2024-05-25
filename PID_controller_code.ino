// Project: Drone Flight Controller - PID Control
// Code Author: Dhrubo Roy Partho
// Date: 25/05/2024
// Version: 1.0v

// !!!!!! issues: ppm, timer channel in uno 

#include <Wire.h>
#include <PulsePosition.h>


// Pin define
#define led1 5
#define led2 13
#define led3 6
#define motor1 1
#define motor2 2
#define motor3 3
#define motor4 4


// MPU6050 variables
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
// Mpu6050 variables end


// Kalman filter variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[] = {0, 0};    //1st one is angle prediction 2nd one is
                                    // uncertainty prediction
// Kalman filter variables end


// PulsePosition variables
PulsePositionInput ReceiverInput(RISING);
float ReceivedValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;
// PulsePosition variables end


// Battery variables
float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed = 0;
float BatteryDefault = 1300;
// Battery variables end


// Timer variable start
uint32_t LoopTimer;
// Timer variable end


// Variables for PID and RF Controller
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputPitch, InputYaw, InputThrottle;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

float PRateRoll = 0.6;
float PRatePitch = PRateRoll;
float PRateYaw = 2;

float IRateRoll = 3.5;
float IRatePitch = IRateRoll;
float IRateYaw = 12;

float DRateRoll = 0.03;
float DRatePitch = DRateRoll;
float DRateYaw = 0;
// Variables for PID and RF Controller end


// Motor input variables
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
// Motor input variables end


// Function for battery voltage calculation
void battery_voltage() {
    Voltage = (float)analogRead(15)/62;     //!!!!! 15 Pin can be change
    Current - (float)analogRead(21)*0.089;  //!!!!! 21 Pin can be change
}


// Receiver reading function
void read_receiver() {
    ChannelNumber = ReceivedInput.available();
    if(ChannelNumber > 0) {
        for(int i=1;i<=ChannelNumber;i++){
            ReceivedValue[i-1] = ReceivedInput.read(i);
        }
    }
}


// MPU6050 signal reader function
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


// Kalman filter function 1D
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


// PID function
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
    float Pterm = P*Error;
    float Iterm = PrevIterm + I*(Error+PrevError)*0.004/2;

    if(Iterm > 400) Iterm = 400;
    else if(Iterm < -400) Iterm = -400;
    float Dterm = D*(Error-PrevError)/0.004;    // 250 Hz process
    float PIDOutput = Pterm + Iterm + Dterm;
    if(PIDOutput > 400) PIDOutput = 400;
    else if(PIDOutput < -400) PIDOutput = -400;

    // Return the output of the PID function
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}


// PID reset function
void reset_pid() {
    PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevErrorRateYaw = 0;
}


void setup() {
    // Visualizing the setup phase using the LED
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);


    // MPU6050 initialization
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
    
    // Calibration
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

    analogWriteFrequency(1, 250);
    analogWriteFrequency(2, 250);
    analogWriteFrequency(3, 250);
    analogWriteFrequency(4, 250);
    analogWriteResolution(12);

    digitalWrite(led3, HIGH);
    // Battery checking
    battery_voltage();
    if(Voltage > 8.3) {
        digitalWrite(led1, LOW);
        BatteryAtStart = BatteryDefault;
    }
    else if(Voltage < 7.5){
        BatteryAtStart = 30/100*BatteryDefault;
    }
    else{
        digitalWrite(led1, LOW);
        BatteryAtStart = (82*Voltage-580)/180*BatteryDefault;
    }

    // Avoid accidental lift off after the setup process
    ReceiveInput.begin(14);
    while(ReceivedValue[2] < 1020 || ReceivedValue[2] > 1050){
        read_receiver();
        delay(4);
    }

    LoopTimer = micros();
}

void loop() {
    // MPU6050 reading
    gyro_signals();
    // Correction
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;


    // Radio receiver reading
    read_receiver();

    // Conversion of received values from radio controller
    DesiredRateRoll = 0.15*(ReceivedValue[0] - 1500);
    DesiredRatePitch = 0.15*(ReceivedValue[1] - 1500);
    InputThrottle = ReceivedValue[2];
    DesiredRateYaw = 0.15*(ReceivedValue[3] - 1500);

    //Error calculation
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    // PID calling
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];

    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];


    // Limiting throttle input
    if(InputThrottle > 1800) InputThrottle = 1800;

    // Motor controlling
    MotorInput1 = 1.024*(InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = 1.024*(InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = 1.024*(InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = 1.024*(InputThrottle + InputRoll - InputPitch + InputYaw);

    // Motor speed clipping
    if(MotorInput1 > 2000) MotorInput1 = 1999;
    if(MotorInput2 > 2000) MotorInput2 = 1999;
    if(MotorInput3 > 2000) MotorInput3 = 1999;
    if(MotorInput4 > 2000) MotorInput4 = 1999;

    // 18% minimul power during flight
    int ThrottleIdle = 1180;
    if(MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if(MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if(MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if(MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

    // Turning off motors
    int ThrottleCutOff = 1000;
    if(ReceivedValue[2] < 1050){
        MotorInput1 = ThrottleCutOff;
        MotorInput2 = ThrottleCutOff;
        MotorInput3 = ThrottleCutOff;
        MotorInput4 = ThrottleCutOff;
        reset_pid();
    }

    // Sending signals to the motors
    analogWrite(motor1, MotorInput1);
    analogWrite(motor2, MotorInput2);
    analogWrite(motor3, MotorInput3);
    analogWrite(motor4, MotorInput4);

    // Keep tracking battery level
    battery_voltage();
    CurrentConsumed = Current*1000*0.004/3600 + CurrentConsumed;
    BatteryRemaining = (BatteryAtStart - CurrentConsumed)/BatteryDefault*100;
    if(BatteryRemaining <= 30) digitalWrite(led1, HIGH);
    else digitalWrite(led1, LOW);

    // 250 Hz control loop for delay
    while(micros() - LoopTimer < 4000);
    LoopTimer = micros();
}
