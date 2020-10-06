#include "Arduino.h"

#include <EEPROM.h>

#include <MPU9250.h>
#include "utility/vector.h"
#include "utility/quaternion.h"
#include <quaternionFilters.h>

#include <cmath>
#include <cstring>

typedef struct {
    int16_t Begin;   // 2  Debut
    uint16_t Cpt;      // 2  Compteur trame or Code info or error
    float gyro[3];   // 12 [Y, P, R]    gyro
    float acc[3];    // 12 [x, y, z]    Acc
    int16_t End;      // 2  Fin
} _hatire;

typedef struct {
    int16_t Begin;   // 2  Debut
    uint16_t Code;     // 2  Code info
    char Msg[24];   // 24 Message
    int16_t End;      // 2  Fin
} _msginfo;

typedef struct {
    float orientation[3];

    float magBias[3];
    float magScale[3];
    float gyroBias[3];
    float accelBias[3];

} _eprom_save;

char Version[] = "HAT V 1.10";

MPU9250 mpu = MPU9250();

_eprom_save calibrationOffsets;

// Control/status vars
bool debug = false;
bool mpuReady = false;
bool active = false;

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

_hatire hatire;
_msginfo msginfo;

bool AskCalibrate = false;  // set true when calibrating is ask
int CptCal = 0;
const int NbCal = 5;

constexpr int CAL_BUTTON = 32;
uint32_t lastUpdate = 0;

//The setup function is called once at startup of the sketch
void setup() {
// Add your initialization code here
    pinMode(CAL_BUTTON, INPUT_PULLDOWN);

    Serial.begin();
    delay(2000);
    PrintCodeSerial(2000, Version, true);

    hatire.Begin = 0xAAAA;
    hatire.Cpt = 0;
    hatire.End = 0x5555;

    msginfo.Begin = 0xAAAA;
    msginfo.Code = 0;
    msginfo.End = 0x5555;

    Wire.begin();

    PrintCodeSerial(3001, "Initializing I2C", true);

    if (!mpu.begin()) {
        PrintCodeSerial(9007, "MPU9250 ERROR", true);
    } else {
        PrintCodeSerial(3003, "MPU9250 OK", true);
        mpuReady = true;
        PrintCodeSerial(3002, "Initializing EEPROM", true);
        EEPROM.begin();

        ReadParams();

        while (Serial.available() && Serial.read())
            ; // empty buffer

        PrintCodeSerial(3012, "READY", true);
    }

}

double mapFloat(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void calibrate() {
    CptCal = 0;
    clearOffsets();
    AskCalibrate = true;
}

// The loop function is called in an endless loop
void loop() {
//Add your repeated code here
    if (Serial.available() > 0)
        serialEvent();

    if (digitalRead(CAL_BUTTON) == 1) {
        calibrate();
    }

    if (mpuReady) {

        if (!mpu.dataReady()) {
            return;
        }

        float ax, ay, az, gx, gy, gz, mx, my, mz;

        mpu.readAcceleration(ax, ay, az);
        mpu.readGyro(gx, gy, gz);
        bool res = mpu.readMagnet(mx, my, mz);

        if (!res)
            return;

        uint32_t Now = micros();
        // Set integration time by time elapsed since last filter update
        double deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = Now;

        // ENU orientation with chip inverse with FSYNC pin to north: -- misaligns the chip but stable
        //MadgwickQuaternionUpdate(ax, -ay, -az, gx * PI / 180.0f, -gy * PI / 180.0f, -gz * PI / 180.0f, my, -mx, mz,
        //deltat);

        // NWU orientation with chip inverse with FSYNC pin to north: works good
        MadgwickQuaternionUpdate(-ay, -ax, -az, -gy * PI / 180.0f, -gx * PI / 180.0f, -gz * PI / 180.0f, -mx, -my, mz,
                deltat);

        // NWU orientation with chip correct. VCC to North - works
        //MadgwickQuaternionUpdate(ay, -ax, az, gy * PI / 180.0f, -gx * PI / 180.0f, gz * PI / 180.0f, mx, -my, -mz, deltat);

        // NWU orientation with chip looks to east. VCC to North
        //MadgwickQuaternionUpdate(ay, az, ax, gy * PI / 180.0f, gz * PI / 180.0f, gx * PI / 180.0f, mx, -mz, my, deltat);

        if (active) {

            const float *q = getQ();
            //format : yaw, pitch, roll;

            float pitch, yaw, roll;
            float a12, a22, a31, a32, a33;       // rotation matrix coefficients for Euler angles and gravity components
            float lin_ax, lin_ay, lin_az;        // linear acceleration (acceleration with gravity component subtracted)

            a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
            a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
            a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
            a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
            a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
            pitch = -asinf(a32);
            roll = atan2f(a31, a33);
            yaw = atan2f(a12, a22);
            pitch *= 180.0f / PI;
            yaw *= 180.0f / PI;
            roll *= 180.0f / PI;

            lin_ax = ax + a31;
            lin_ay = ay + a32;
            lin_az = az - a33;

            if (debug) {
                /*
                 Serial.println("RAW:");

                 Serial.print("ax = ");
                 Serial.print((int) 1000 * ax);
                 Serial.print(" ay = ");
                 Serial.print((int) 1000 * ay);
                 Serial.print(" az = ");
                 Serial.print((int) 1000 * az);

                 Serial.print(" gx = ");
                 Serial.print(gx, 2);
                 Serial.print(" gy = ");
                 Serial.print(gy);
                 Serial.print(" gz = ");
                 Serial.print(gz, 2);

                 Serial.print(" mx = ");
                 Serial.print((int) mx);
                 Serial.print(" my = ");
                 Serial.print((int) my);
                 Serial.print(" mz = ");
                 Serial.println((int) mz);

                 Serial.println("QUAT:");
                 Serial.print("w:");
                 Serial.print(q[0]);
                 Serial.print(" x:");
                 Serial.print(q[1]);
                 Serial.print(" y:");
                 Serial.print(q[2]);
                 Serial.print(" z:");
                 Serial.println(q[3]);

                 Serial.println("EULER :");
                 Serial.print("Yaw(X):");
                 Serial.print(yaw);
                 Serial.print(" Pitch(Y):");
                 Serial.print(pitch);
                 Serial.print(" Roll(Z):");
                 Serial.println(roll);
                 */

                Serial.print(mx, 3);
                Serial.print(", ");
                Serial.print(my, 3);
                Serial.print(", ");
                Serial.println(mz, 3);

                /*
                 imu::Vector<3> myVec = imu::Quaternion(q[0], q[1], q[2], q[3]).toEuler();

                 Serial.println("*MY* EULER:");
                 Serial.print("Yaw(X):");
                 Serial.print(myVec.x() * 180.0f / M_PI);
                 Serial.print(" Pitch(Y):");
                 Serial.print(myVec.y() * 180.0f / M_PI);
                 Serial.print(" Roll(Z):");
                 Serial.println(myVec.z() * 180.0f / M_PI);
                 */

            }

            if (AskCalibrate) {
                if (CptCal >= NbCal) {
                    CptCal = 0;
                    calibrationOffsets.orientation[0] = calibrationOffsets.orientation[0] / NbCal;
                    calibrationOffsets.orientation[1] = calibrationOffsets.orientation[1] / NbCal;
                    calibrationOffsets.orientation[2] = calibrationOffsets.orientation[2] / NbCal;
                    AskCalibrate = false;
                    SaveParams();
                } else {
                    calibrationOffsets.orientation[0] += yaw;
                    calibrationOffsets.orientation[1] += pitch;
                    calibrationOffsets.orientation[2] += roll;

                    CptCal++;
                }

                hatire.gyro[0] = 0;
                hatire.gyro[1] = 0;
                hatire.gyro[2] = 0;
                hatire.acc[0] = 0;
                hatire.acc[1] = 0;
                hatire.acc[2] = 0;
            } else {
                //send order : yaw, pitch, roll
                float rawYaw = yaw - calibrationOffsets.orientation[0];
                float rawPitch = pitch - calibrationOffsets.orientation[1];
                float rawRoll = roll - calibrationOffsets.orientation[2];

                hatire.gyro[0] = rawYaw > 180.0f ? rawYaw - 360.0f : rawYaw;
                hatire.gyro[1] = rawPitch > 90.0f ? 180.0f - rawPitch : rawPitch;
                hatire.gyro[2] = rawRoll > 180.0f ? rawRoll - 360.0f : rawRoll;

                hatire.acc[0] = lin_ax;
                hatire.acc[1] = lin_ay;
                hatire.acc[2] = lin_az;

            }
            if (!debug)

                Serial.write((byte*) &hatire, 30);

            else {
                /*
                 Serial.println("CALCULATED:");
                 Serial.print("Yaw(X):");
                 Serial.print(hatire.gyro[0]);
                 Serial.print(" Pitch(Y):");
                 Serial.print(hatire.gyro[1]);
                 Serial.print(" Roll(Z):");
                 Serial.println(hatire.gyro[2]);
                 */
            }

            hatire.Cpt++;
            if (hatire.Cpt > 999) {
                hatire.Cpt = 0;
            }
        }
    }

}

// ================================================================
// ===                    Serial Command                        ===
// ================================================================
void serialEvent() {
    char commande = (char) Serial.read();
    switch (commande) {
        case 'S':
            PrintCodeSerial(5001, "HAT START", true);
            if (mpuReady == true) {
                hatire.Cpt = 0;
                active = true;
            } else {
                PrintCodeSerial(9011, "Error BNO doesn't work", true);
            }
            break;

        case 's':
            PrintCodeSerial(5002, "HAT STOP", true);
            active = false;
            break;

        case 'R':
            PrintCodeSerial(5003, "HAT RESET", true);

            mpuReady = false;
            setup();

            break;

        case 'Z':
            clearOffsets();
            SaveParams();
            PrintCodeSerial(3021, "Calibration erased", true);

            break;

        case 'D':

            debug = !debug;
            if (debug)
                PrintCodeSerial(3081, "Debug is ON!", true);
            else
                PrintCodeSerial(3081, "Debug is OFF!", true);
            break;

        case 'C':
            if (!mpuReady) {
                PrintCodeSerial(9091, "MPU is not ready!", true);
                return;
            } else {
                PrintCodeSerial(3091, "Calibration requested", true);
                calibrate();
            }
            break;

        case 'V':
            PrintCodeSerial(2000, Version, true);
            break;

        case 'X':

            if (!mpuReady) {
                PrintCodeSerial(9091, "MPU is not ready!", true);
                return;
            } else {
                PrintCodeSerial(3005, "Calibration Sequence", true);

                mpu.calibrateSensors();
                SaveParams();

                PrintCodeSerial(3007, "Calibration Complete", false);
                break;
            }
        case 'I': {

            Serial.println();
            Serial.print("Version : \t");
            Serial.println(Version);
            Serial.println("Gyroscopes offsets");
            Serial.print("Yaw: ");
            Serial.println(calibrationOffsets.orientation[0]);
            Serial.print("Pitch : ");
            Serial.println(calibrationOffsets.orientation[1]);
            Serial.print("Roll: ");
            Serial.println(calibrationOffsets.orientation[2]);

            Serial.println("Magnetometer bias");
            Serial.print("X : ");
            Serial.print(calibrationOffsets.magBias[0], 3);
            Serial.print(" Y : ");
            Serial.print(calibrationOffsets.magBias[1], 3);
            Serial.print(" Z : ");
            Serial.println(calibrationOffsets.magBias[2], 3);

            Serial.println("Magnetometer scale");
            Serial.print("X : ");
            Serial.print(calibrationOffsets.magScale[0], 3);
            Serial.print(" Y : ");
            Serial.print(calibrationOffsets.magScale[1], 3);
            Serial.print(" Z : ");
            Serial.println(calibrationOffsets.magScale[2], 3);

            Serial.println("Accelerometer Bias");
            Serial.print("X : ");
            Serial.print(calibrationOffsets.accelBias[0], 3);
            Serial.print(" Y : ");
            Serial.print(calibrationOffsets.accelBias[1], 3);
            Serial.print(" Z : ");
            Serial.println(calibrationOffsets.accelBias[2], 3);

            Serial.println("Gyro bias");
            Serial.print("X : ");
            Serial.print(calibrationOffsets.gyroBias[0], 3);
            Serial.print(" Y : ");
            Serial.print(calibrationOffsets.gyroBias[1], 3);
            Serial.print(" Z : ");
            Serial.println(calibrationOffsets.gyroBias[2], 3);

            PrintCodeSerial(3021, "Init Complete", false);

        }
            break;

        default:
            break;
    }
}

// ================================================================
// ===               PRINT SERIAL FORMAT                        ===
// ================================================================
void PrintCodeSerial(uint16_t code, const char Msg[24], bool EOL) {
    msginfo.Code = code;
    memset(msginfo.Msg, 0x00, 24);
    strcpy(msginfo.Msg, Msg);
    if (EOL)
        msginfo.Msg[23] = 0x0A;
    // Send HATIRE message to  PC
    Serial.write((byte*) &msginfo, 30);
}

// ================================================================
// ===                    DELETE OFFSETS                        ===
// ================================================================
void clearOffsets() {
    calibrationOffsets.orientation[0] = 0;
    calibrationOffsets.orientation[1] = 0;
    calibrationOffsets.orientation[2] = 0;
}

// ================================================================
// ===                    SAVE PARAMS                           ===
// ================================================================
void SaveParams() {

    std::memcpy(calibrationOffsets.accelBias, mpu.getAccelBias(), sizeof(float) * 3);
    std::memcpy(calibrationOffsets.gyroBias, mpu.getGyroBias(), sizeof(float) * 3);
    std::memcpy(calibrationOffsets.magBias, mpu.getMagBias(), sizeof(float) * 3);
    std::memcpy(calibrationOffsets.magScale, mpu.getMagScale(), sizeof(float) * 3);

    EEPROM.put(0, calibrationOffsets);
}

// ================================================================
// ===                    READ PARAMS                           ===
// ================================================================
void ReadParams() {
    EEPROM.get(0, calibrationOffsets);

    std::memcpy(mpu.getAccelBias(), calibrationOffsets.accelBias, sizeof(float) * 3);
    std::memcpy(mpu.getGyroBias(), calibrationOffsets.gyroBias, sizeof(float) * 3);
    std::memcpy(mpu.getMagBias(), calibrationOffsets.magBias, sizeof(float) * 3);
    std::memcpy(mpu.getMagScale(), calibrationOffsets.magScale, sizeof(float) * 3);
}
