#ifndef __CONFIG_H
#define __CONFIG_H

#define VERSIONSTR "20190704a"

// DEBUG Switches
#ifdef DEBUG
# define DEBUG_LEVEL 1
# define TX_INTERVAL 500
#else
# define DEBUG_LEVEL 0
# define TX_INTERVAL 5
#endif

#define DATALOG_INTERVAL TX_INTERVAL * 2

#define MOTOR_STEPS 200
#define RPM 120
#define MICROSTEPS 2

#define STPA_DIR 26
#define STPA_STEP 25

#define STPB_DIR 21
#define STPB_STEP 17

#define STPC_DIR 33
#define STPC_STEP 15

#define STP_EN 18
#define STP_MS1 23
#define STP_MS2 22
#define STP_MS3 32

#define ONE_WIRE_BUS 19

#define LED_COUNT 4
#define LED_PIN 16
#define LED_COLOR_SATURATION 128

#endif // __CONFIG_H
