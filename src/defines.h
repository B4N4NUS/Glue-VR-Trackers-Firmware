#define IMU IMU_MPU6050
#define SECOND_IMU IMU_MPU6050
#define BOARD BOARD_WEMOSD1MINI
#define IMU_ROTATION DEG_270
#define SECOND_IMU_ROTATION DEG_270

// Battery monitoring options (comment to disable):
// BAT_EXTERNAL for ADC pin, BAT_INTERNAL for internal - can detect only low battery, BAT_MCP3021 for external ADC
#define BATTERY_MONITOR BAT_EXTERNAL
#define BATTERY_SHIELD_RESISTANCE 180 //130k BatteryShield, 180k SlimeVR or fill in external resistor value in kOhm

#define PIN_IMU_SDA D2
#define PIN_IMU_SCL D1
#define PIN_IMU_INT D5
#define PIN_IMU_INT_2 D6
#define PIN_BATTERY_LEVEL A0
