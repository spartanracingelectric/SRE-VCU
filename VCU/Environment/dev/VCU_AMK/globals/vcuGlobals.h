/*****************************************************************************
 * vcuGlobals.h -  Global Variables and Constants
 ******************************************************************************
 * This file centralizes all global variables and constants that were
 * scattered throughout the original codebase.
 ****************************************************************************/

#ifndef _VCU_GLOBALS_H
#define _VCU_GLOBALS_H

#include "IO_Driver.h"
#include "sensors.h"

//=============================================================================
// SYSTEM TIMING CONSTANTS
//=============================================================================

// Main control loop timing
#define VCU_MAIN_LOOP_PERIOD_US 10000UL      // 10ms main loop period
#define VCU_CALIBRATION_TIMEOUT_US 5000000UL // 5 seconds calibration timeout
#define VCU_CAN_TIMEOUT_US 50000UL           // 50ms CAN timeout
#define VCU_ADC_WASTE_LOOP_US 55555UL        // ADC stabilization time

// Precharge and startup timing
#define VCU_PRECHARGE_SOAK_TIME_US 10000000UL // 10 seconds precharge
#define VCU_RTDS_DURATION_US 1500000UL        // 1.5 seconds ready to drive sound
#define VCU_ECO_BUTTON_HOLD_TIME_US 3000000UL // 3 seconds eco button hold
#define VCU_ECO_BUTTON_DEBOUNCE_US 10000UL    // 10ms eco button debounce

//=============================================================================
// TORQUE AND MOTOR CONTROL CONSTANTS
//=============================================================================

// Torque calculation constants
#define VCU_NOMINAL_TORQUE_NM 9.8f
#define VCU_MAX_TORQUE_NM 21.0f
#define VCU_TORQUE_SCALING_FACTOR 10000
#define VCU_TORQUE_LIMIT_SCALE_FACTOR 10

// Calculated torque constants
#define VCU_TORQUE_MAX_CALC ((VCU_MAX_TORQUE_NM / VCU_NOMINAL_TORQUE_NM) / 100.0f)
#define VCU_TORQUE_LIMIT_POS_01NM ((ubyte2)(VCU_MAX_TORQUE_NM * VCU_TORQUE_LIMIT_SCALE_FACTOR))
#define VCU_TORQUE_LIMIT_NEG_01NM 0 // set -210 for regen

// Brake pressure constants
#define VCU_BRAKES_ON_PERCENT 0.08f
#define VCU_BRAKE_THRESHOLD_PERCENT 0.02f

//=============================================================================
// CAN COMMUNICATION CONSTANTS
//=============================================================================

// CAN message limits
#define VCU_CAN_MAX_MESSAGE_ID 0x7FF
#define VCU_CAN_MAX_MESSAGE_COUNT 128
#define VCU_CAN_STD_FRAME_SIZE 8

// CAN bus configuration
#define VCU_CAN0_BUS_SPEED 500
#define VCU_CAN1_BUS_SPEED 500
#define VCU_CAN0_READ_LIMIT 40
#define VCU_CAN0_WRITE_LIMIT 40
#define VCU_CAN1_READ_LIMIT 20
#define VCU_CAN1_WRITE_LIMIT 20

// AMK Drive Inverter CAN IDs
#define VCU_DI_BASE_CAN_ID_OUTGOING 0x183
#define VCU_DI_BASE_CAN_ID_INCOMING 0x282

// BMS CAN IDs
#define VCU_BMS_BASE_ADDRESS 0x600
#define VCU_BMS_MASTER_FAULTS 0x002
#define VCU_BMS_MASTER_WARNINGS 0x004
#define VCU_BMS_MASTER_SYSTEM_STATUS 0x010
#define VCU_BMS_PACK_SAFE_OPERATING_ENVELOPE 0x011
#define VCU_BMS_MASTER_LOCAL_BOARD_MEASUREMENTS 0x012
#define VCU_BMS_DIGITAL_INPUTS_AND_OUTPUTS 0x013
#define VCU_BMS_PACK_LEVEL_MEASUREMENTS_1 0x020
#define VCU_BMS_PACK_LEVEL_MEASUREMENTS_2 0x021
#define VCU_BMS_CELL_VOLTAGE_SUMMARY 0x022
#define VCU_BMS_CELL_TEMPERATURE_SUMMARY 0x023
#define VCU_BMS_PACK_LEVEL_MEASUREMENTS_3 0x024
#define VCU_BMS_CELL_VOLTAGE_DATA 0x030
#define VCU_BMS_CELL_TEMPERATURE_DATA 0x080
#define VCU_BMS_CELL_SHUNTING_STATUS_1 0x0D0
#define VCU_BMS_CELL_SHUNTING_STATUS_2 0x0D1
#define VCU_BMS_CELL_SHUNTING_STATUS_3 0x0D2
#define VCU_BMS_CELL_SHUNTING_STATUS_4 0x0D3
#define VCU_BMS_CONFIGURATION_INFORMATION 0x0FC
#define VCU_BMS_FIRMWARE_VERSION_INFORMATION 0x0FE

// DAQ CAN IDs
#define VCU_DAQ_ACCEL_DATA 0x400
#define VCU_DAQ_GPS_DATA 0x401
#define VCU_DAQ_GYRO_DATA 0x402
#define VCU_DAQ_DETECTION_PCB 0x403

// Instrument Cluster CAN IDs
#define VCU_IC_BASE_ADDRESS 0x702

// Safety and Debug CAN IDs
#define VCU_SAFETY_DEBUG 0x5FF
#define VCU_DEBUG_MESSAGE_BASE 0x500

//=============================================================================
// BIT MASK CONSTANTS
//=============================================================================

// AMK Drive Inverter status bits
#define VCU_SYSTEM_READY_STATUS 0x01
#define VCU_ERROR_STATUS 0x02
#define VCU_WARNING_STATUS 0x04
#define VCU_QUIT_DC_ON_STATUS 0x08
#define VCU_DC_ON_VAL_STATUS 0x10
#define VCU_QUIT_INVERTER_ON_STATUS 0x20
#define VCU_INVERTER_ON_STATUS 0x40
#define VCU_DERATING_STATUS 0x80

// BMS fault flags
#define VCU_BMS_CELL_OVER_VOLTAGE_FLAG 0x01
#define VCU_BMS_CELL_UNDER_VOLTAGE_FLAG 0x02
#define VCU_BMS_CELL_OVER_TEMPERATURE_FLAG 0x04

//=============================================================================
// SCALING AND CONVERSION CONSTANTS
//=============================================================================

// BMS scaling factors
#define VCU_BMS_VOLTAGE_SCALE 1000 // V*1000, milliVolts to Volts
#define VCU_BMS_CURRENT_SCALE 1000 // A*1000, milliAmps to Amps
#define VCU_BMS_POWER_SCALE (VCU_BMS_VOLTAGE_SCALE * VCU_BMS_CURRENT_SCALE)
#define VCU_BMS_TEMPERATURE_SCALE 10 // degC*10, deciCelsius to Celsius
#define VCU_BMS_PERCENT_SCALE 10     // %*10, percent*10 to percent
#define VCU_BMS_AMP_HOURS_SCALE 10   // Ah*10, deciAmpHours to AmpHours

// DAQ scaling factors
#define VCU_DAQ_ACCEL_SCALE 0.01f
#define VCU_DAQ_ACCEL_OFFSET 320.0f
#define VCU_DAQ_GYRO_SCALE 0.0078125f
#define VCU_DAQ_GYRO_OFFSET 250.0f
#define VCU_DAQ_GPS_SCALE 0.001f

// Wheel speed calculation constants
#define VCU_NUM_BUMPS 16
#define VCU_WHEEL_DIAMETER 16 // Inches
#define VCU_NUM_BUMPS_D ((double)VCU_NUM_BUMPS)
#define VCU_WHEEL_DIAMETER_D ((double)VCU_WHEEL_DIAMETER)

//=============================================================================
// SAFETY AND THRESHOLD CONSTANTS
//=============================================================================

// BMS safety thresholds
#define VCU_BMS_MAX_CELL_MISMATCH_V 1.00f
#define VCU_BMS_MIN_CELL_VOLTAGE_WARNING 3.20f
#define VCU_BMS_MAX_CELL_TEMPERATURE_WARNING 55.0f

// Cooling system thresholds
#define VCU_COOLING_WATER_PUMP_MIN 0.2f
#define VCU_COOLING_WATER_PUMP_LOW 25
#define VCU_COOLING_WATER_PUMP_HIGH 40
#define VCU_COOLING_MOTOR_FAN_LOW 38
#define VCU_COOLING_MOTOR_FAN_HIGH 43
#define VCU_COOLING_RAD_FAN_LOW 25
#define VCU_COOLING_RAD_FAN_HIGH 40
#define VCU_COOLING_BATTERY_FAN_LOW 38
#define VCU_COOLING_BATTERY_FAN_HIGH 43

// Ground speed thresholds
#define VCU_MIN_SPEED_FOR_REGEN_KPH 15
#define VCU_BMS_VOLTAGE_FOR_REGEN 38500

//=============================================================================
// CHANNEL AND PIN CONSTANTS
//=============================================================================

// RTDS channel
#define VCU_RTDS_CHANNEL 1

// Error reading constant
#define VCU_ERROR_READING_LIMIT_VALUE -1

//=============================================================================
// GLOBAL VARIABLE DECLARATIONS
//=============================================================================

// System timing variables
extern ubyte4 timestamp_Precharge;
extern ubyte4 timestamp_SoftBSPD;
extern bool prevHVILState;

// Sensor extern declarations (defined in sensors.c)
extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
extern Sensor Sensor_BPS1;
extern Sensor Sensor_SAS;
extern Sensor Sensor_LVBattery;
extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;
extern Sensor Sensor_RTDButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_DRSButton;
extern Sensor Sensor_DRSKnob;
extern Sensor Sensor_LCButton;
extern Sensor Sensor_HVILTerminationSense;
extern Sensor Sensor_TVButton;
extern Sensor Sensor_TEMP_BrakingSwitch;
extern Sensor Sensor_TCSKnob;
extern Sensor Sensor_TCSSwitchUp;
extern Sensor Sensor_WPS_FL;
extern Sensor Sensor_WPS_FR;
extern Sensor Sensor_WPS_RL;
extern Sensor Sensor_WPS_RR;

//=============================================================================
// UTILITY MACROS
//=============================================================================

// Byte extraction macros
#define EXTRACT_UINT16_LE(data, offset) \
    ((ubyte2)((data)[(offset) + 1] << 8) | (ubyte2)((data)[(offset)]))

#define EXTRACT_UINT32_LE(data, offset)     \
    ((ubyte4)((data)[(offset) + 3] << 24) | \
     (ubyte4)((data)[(offset) + 2] << 16) | \
     (ubyte4)((data)[(offset) + 1] << 8) |  \
     (ubyte4)((data)[(offset)]))

// Bit manipulation macros
#define SET_BIT(value, bit) ((value) |= (1 << (bit)))
#define CLEAR_BIT(value, bit) ((value) &= ~(1 << (bit)))
#define TOGGLE_BIT(value, bit) ((value) ^= (1 << (bit)))
#define CHECK_BIT(value, bit) (((value) >> (bit)) & 1)

// Scaling macros
#define APPLY_SCALING(raw, scale, offset) \
    (((float4)(raw) * (scale)) + (offset))

#define APPLY_INVERSE_SCALING(scaled, scale, offset) \
    ((ubyte4)(((scaled) - (offset)) / (scale)))

//=============================================================================
// FUNCTION PROTOTYPES
//=============================================================================

// Global initialization
void VCU_Globals_Initialize(void);

// Timing utilities
ubyte4 VCU_GetSystemTimeUS(void);
bool VCU_IsTimeElapsed(ubyte4 startTime, ubyte4 duration);

// Safety utilities
bool VCU_IsSystemSafe(void);
void VCU_SetSystemFault(bool fault);

#endif // _VCU_GLOBALS_H
