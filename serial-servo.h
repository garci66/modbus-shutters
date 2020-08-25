#ifndef _GARCI_SERIAL_SERVO
#define _GARCI_SERIAL_SERVO

#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <EEPROM.h>
#include <limits.h>
#include <avr/wdt.h>

#define GARCI_MAGIC 0xD6CC
#define GARCI_VERSION 1

#define DIRECTION_UP 1
#define DIRECTION_DN -1
#define DIRECTION_STOP 0
#define MODBUS_UPDATE_INTERVAL 100

enum class mbCoils {_START, cmdStepUp=_START, cmdStepDown, cmdUp, cmdDn, cmdToggleCalibrationMode, cmdStoreMinPos, cmdStoreMaxPos, cmdResetTripped, cmdResetPosition, cmdSaveSettings, cmdReboot, cmdFactoryReset, _LAST};
enum class mbInputs {_START, atTop=_START, atBottom, safetyTripped, isCalibrationMode, _LAST};
enum class mbHoldingRegisters {_START, baudRate100x=_START, slaveId, pwmPreScaler, servoZero, fullSpeedDelta, targetApproach10x, deadBand, minPos, maxPos, stepSize, safetyInterval, reqTargetPosition, reqPercentPosition,  _LAST};
enum class mbInputRegisters {_START, currentPosition=_START, currentPercentPosition, currentFalseCounter, currentTarget, _LAST};

inline mbCoils& operator++(mbCoils& d,int)  {  return d=static_cast<mbCoils>(static_cast<int>(d)+1);  }; 
inline mbInputs& operator++(mbInputs& d,int)  {  return d=static_cast<mbInputs>(static_cast<int>(d)+1);  }; 
inline mbHoldingRegisters& operator++(mbHoldingRegisters& d,int)  {  return d=static_cast<mbHoldingRegisters>(static_cast<int>(d)+1);  }; 
inline mbInputRegisters& operator++(mbInputRegisters& d,int)  {  return d=static_cast<mbInputRegisters>(static_cast<int>(d)+1);  }; 

struct config {
    uint16_t magic;
    uint8_t version;
    uint16_t baudRate100x;
    uint8_t slaveId;
    uint8_t pwmPreScaler;
    uint8_t servoZero;
    uint8_t fullSpeedDelta;
    uint8_t targetApproach10x;
    uint8_t deadBand;
    int16_t lastPos;
    int16_t minPos;
    int16_t maxPos;
    uint8_t stepSize;
    uint16_t safetyInterval;
    uint16_t maxRuntimeS;
    bool safetyTripped;
};

struct config defaultConfig {
    GARCI_MAGIC, 
    GARCI_VERSION,
    96,                 //baudRate100x 
    1,                  //slaveId
    5,                  //pwmPreScaler
    185,                //servoZero
    16,                 //fullSpeedDelta
    30,                 //targetApproach10x
    2,                  //deadBand
    0,                  //lastPos
    INT16_MIN,          //minPos
    INT16_MAX,          //maxPos
    16,                 //stepSize
    1000,               //safetyInterval
    90,                 //maxRuntimeS
    false               //safetyTripped
};

#endif
