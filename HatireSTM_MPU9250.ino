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
    imu::Quaternion baseQuat;

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
    PrintCodeSerial(2000, Version, true);

    hatire.Begin = 0xAAAA;
    hatire.Cpt = 0;
    hatire.End = 0x5555;

    msginfo.Begin = 0xAAAA;
    msginfo.Code = 0;
    msginfo.End = 0x5555;

    PrintCodeSerial(3002, "Initializing EEPROM", true);
    EEPROM.begin();
    PrintCodeSerial(3001, "Initializing I2C", true);
    Wire.begin();

    mpu.setScales(MPU9250::AFS_2G, MPU9250::GFS_250DPS, MPU9250::MFS_16BITS, MPU9250::M_100HZ);

    PrintCodeSerial(3001, "Initializing MPU", true);
    if (!mpu.begin()) {
        PrintCodeSerial(9007, "MPU9250 ERROR", true);
    } else {
        PrintCodeSerial(3003, "MPU9250 OK", true);
        mpuReady = true;

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

float ax, ay, az, gx, gy, gz, mx, my, mz;
float sum = 0.0f;

bool yesdebug = false;

uint32_t count = 0, sumCount = 0;  // used to control display output rate

// The loop function is called in an endless loop
void loop() {
//Add your repeated code here
    if (Serial.available() > 0)
        serialEvent();

    if (digitalRead(CAL_BUTTON) == 1) {
        calibrate();
    }

    if (mpuReady) {

        if (mpu.dataReady()) {

            mpu.readAcceleration(ax, ay, az);
            mpu.readGyro(gx, gy, gz);
        }

        mpu.readMagnet(mx, my, mz);

        //if (!mpu.readMagnet(mx, my, mz))
        //    return;

        uint32_t Now = micros();
        // Set integration time by time elapsed since last filter update
        double deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = Now;

        sum += deltat; // sum for averaging filter update rate
        sumCount++;

        // NWU orientation with chip inverse with FSYNC pin to north
        MahonyQuaternionUpdate(-ay, -ax, -az, -gy * PI / 180.0f, -gx * PI / 180.0f, -gz * PI / 180.0f, -mx, -my, mz, deltat);

        // NWU orientation with chip correct. VCC to North - works
        //MadgwickQuaternionUpdate(ay, -ax, az, gy * PI / 180.0f, -gx * PI / 180.0f, gz * PI / 180.0f, mx, -my, -mz, deltat);

        // NWU orientation with chip looks to east. VCC to North
        //MahonyQuaternionUpdate(ay, az, ax, gy * PI / 180.0f, gz * PI / 180.0f, gx * PI / 180.0f, mx, -mz, my, deltat);

        // NWU orientation with FSYNC bottom; gyro Z+ to north
        //MadgwickQuaternionUpdate(az, ax, ay, gz * PI / 180.0f, gx * PI / 180.0f, gy * PI / 180.0f, -mz, my, mx, deltat);

        if (active) {

            const float *q = getQ();

            imu::Quaternion sensorQuat = imu::Quaternion(q[0], q[1], q[2], q[3]);

            if (debug) {
                uint32_t delt_t = millis() - count;
                if (delt_t > 500) {
                    yesdebug = true;
                }
            }

            if (yesdebug) {
                Serial.print(1000 * ax, 2);
                Serial.print(", ");
                Serial.print(1000 * ay, 2);
                Serial.print(", ");
                Serial.print(1000 * az, 2);
                Serial.print(", ");

                Serial.print(gx, 2);
                Serial.print(", ");
                Serial.print(gy);
                Serial.print(", ");
                Serial.print(gz, 2);
                Serial.print(", ");

                Serial.print(mx, 2);
                Serial.print(", ");
                Serial.print(my, 2);
                Serial.print(", ");
                Serial.print(mz, 2);
                Serial.print(" || ");

                Serial.print((float) sumCount / sum, 2);
                Serial.print(" Hz");

                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

                sumCount = 0;
                sum = 0;
            }

            if (AskCalibrate) {

                calibrationOffsets.baseQuat = sensorQuat;

                hatire.gyro[0] = 0;
                hatire.gyro[1] = 0;
                hatire.gyro[2] = 0;
                hatire.acc[0] = 0;
                hatire.acc[1] = 0;
                hatire.acc[2] = 0;

                SaveParams();

                AskCalibrate = false;

            } else {

                // calculate Bt * q * B  to change basis:
                // note that we can do this because sensor output quaternions are always unit size.
                const imu::Quaternion &baseQ = calibrationOffsets.baseQuat;
                imu::Quaternion calibratedQ2 = baseQ.conjugate() * sensorQuat;
                //imu::Quaternion calibratedQ1 = calibratedQ1 * baseQ;

                if (yesdebug) {

                    Serial.print(" || ");
                    Serial.print(sensorQuat.w());
                    Serial.print(", ");
                    Serial.print(sensorQuat.x());
                    Serial.print(", ");
                    Serial.print(sensorQuat.y());
                    Serial.print(", ");
                    Serial.print(sensorQuat.z());

                    Serial.print(" || ");
                    Serial.print(baseQ.w());
                    Serial.print(", ");
                    Serial.print(baseQ.x());
                    Serial.print(", ");
                    Serial.print(baseQ.y());
                    Serial.print(", ");
                    Serial.print(baseQ.z());
                    /*
                     Serial.print(" || ");
                     Serial.print(calibratedQ1.w());
                     Serial.print(", ");
                     Serial.print(calibratedQ1.x());
                     Serial.print(", ");
                     Serial.print(calibratedQ1.y());
                     Serial.print(", ");
                     Serial.print(calibratedQ1.z());
                     */
                    Serial.print(" || ");
                    Serial.print(calibratedQ2.w());
                    Serial.print(", ");
                    Serial.print(calibratedQ2.x());
                    Serial.print(", ");
                    Serial.print(calibratedQ2.y());
                    Serial.print(", ");
                    Serial.print(calibratedQ2.z());
                }

                //send order : yaw, pitch, roll
                imu::Vector<3> euler = calibratedQ2.toEuler();

                hatire.gyro[0] = euler.x() * 180 / M_PI;
                hatire.gyro[1] = euler.y() * 180 / M_PI;
                hatire.gyro[2] = euler.z() * 180 / M_PI;

                hatire.acc[0] = 0;
                hatire.acc[1] = 0;
                hatire.acc[2] = 0;

            }

            if (!yesdebug && !debug) {
                Serial.write((byte*) &hatire, 30);

                hatire.Cpt++;
                if (hatire.Cpt > 999) {
                    hatire.Cpt = 0;
                }

            } else if (yesdebug) {
                Serial.print(" || ");
                Serial.print(hatire.gyro[0]);
                Serial.print(", ");
                Serial.print(hatire.gyro[1]);
                Serial.print(", ");
                Serial.println(hatire.gyro[2]);
            }

            if (yesdebug)
                count = millis();

            yesdebug = false;
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

                mpu.calibrateSensors(MPU9250::AXIS_Z);
                SaveParams();

                PrintCodeSerial(3007, "Calibration Complete", false);
                break;
            }
        case 'I': {

            Serial.println();
            Serial.print("Version : \t");
            Serial.println(Version);
            Serial.println("Base Quaternion");
            imu::Quaternion &baseQ = calibrationOffsets.baseQuat;
            Serial.print("W: ");
            Serial.print(baseQ.w());
            Serial.print(" X : ");
            Serial.print(baseQ.x());
            Serial.print(" Y: ");
            Serial.print(baseQ.y());
            Serial.print(" Z: ");
            Serial.println(baseQ.z());

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
    calibrationOffsets.baseQuat = imu::Quaternion();
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
