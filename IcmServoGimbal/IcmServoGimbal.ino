#include <Icm20948.h>
#include <SensorTypes.h>
#include "Icm20948MPUFifoControl.h"
#include <Servo.h>
#include <Wire.h>

// Serial Monitor port
#define debug_port_ Serial

// #define DEBUG
#ifdef DEBUG
#define debugPrint(x) debug_port_.print(x)
#define debugPrintln(x) debug_port_.println(x)
#else
#define debugPrint(x)
#define debugPrintln(x)
#endif

const float kDeg2Rad = 0.017453293f;
const float kRad2Deg = 57.295779513f;

// ------------------- ICM-20948 Definitions -------------------
int rc = 0;
#define THREE_AXES 3
static int unscaled_bias[THREE_AXES * 2];
/* FSR configurations */
int32_t cfg_acc_fsr = 4;    // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
static const uint8_t dmp3_image[] =
    {
#include "icm20948_img.dmp3a.h"
};
/*
  Mounting matrix configuration applied for Accel, Gyro and Mag
*/

static const float cfg_mounting_matrix[9] = {
    1.f, 0, 0,
    0, 1.f, 0,
    0, 0, 1.f};

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
    INV_SENSOR_TYPE_ACCELEROMETER,
    INV_SENSOR_TYPE_GYROSCOPE,
    INV_SENSOR_TYPE_RAW_ACCELEROMETER,
    INV_SENSOR_TYPE_RAW_GYROSCOPE,
    INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
    INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
    INV_SENSOR_TYPE_BAC,
    INV_SENSOR_TYPE_STEP_DETECTOR,
    INV_SENSOR_TYPE_STEP_COUNTER,
    INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
    INV_SENSOR_TYPE_ROTATION_VECTOR,
    INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
    INV_SENSOR_TYPE_MAGNETOMETER,
    INV_SENSOR_TYPE_SMD,
    INV_SENSOR_TYPE_PICK_UP_GESTURE,
    INV_SENSOR_TYPE_TILT_DETECTOR,
    INV_SENSOR_TYPE_GRAVITY,
    INV_SENSOR_TYPE_LINEAR_ACCELERATION,
    INV_SENSOR_TYPE_ORIENTATION,
    INV_SENSOR_TYPE_B2S};
#define AK0991x_DEFAULT_I2C_ADDR 0x0C   /* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR 0x0E /* The secondary I2C address for AK0991x Magnetometers */

#define ICM_I2C_ADDR_REVA 0x68 /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB 0x69 /* I2C slave address for INV device on Rev B board */

#define AD0_VAL 1 // The value of the last bit of the I2C address.
uint8_t I2C_Address = 0x69;
inv_icm20948_t icm_device_;
// ---------------------------------------------------------

// ------------ IMU Definitions ------------
bool new_icm_data_ = false;
float quat_[4];
const unsigned int kImuIntervalMs = 10;
float attitude_rad_[3]; // roll,pitch,yaw

// ------------ Servo Definitions -------------
const int kNumServos = 2;

uint servo_pins_[kNumServos] = {MISO, MOSI};
Servo servos_[kNumServos];

uint servo_target_pw_us_[kNumServos] = {1500, 1500};
float gimbal_gain_scaled_[kNumServos] = {200.0f, 100.0f};

void setup()
{
    //------------- Serial Port Setup -----------------
#ifdef DEBUG
    debug_port_.begin(115200);
    while (!debug_port_)
    {
    }
#endif

    //------------- ICM Setup --------------
    debugPrintln("ICM Setup...");
    icmSetup();
    debugPrintln("Done!");
    // ----------- Servo Setup --------------
    debugPrintln("Servo Setup...");
    servoSetup();
    debugPrintln("Done!");
}

void loop()
{
    pollIcm();
    if (imuTasks())
    {
        moveServos();
    }
}