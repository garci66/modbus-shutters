#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <EEPROM.h>
#include <limits.h>
#define GARCI_MAGIC 0xD6CC
#define GARCI_VERSION 1
#define SERVOPIN 3
#define COUNTERPIN 2
#define DIRECTION_UP 1
#define DIRECTION_DN -1
#define DIRECTION_STOP 0

enum class mbCoils={cmdStepUp, cmdStepDown, cmdUp, cmdDn, cmdToggleCalibrationMode, cmdStoreMinPos, cmdStoreMaxPos, cmdResetTripped, cmdResetPosition, cmdSaveSettings, cmdReboot, cmdFactoryReset, _LAST};
enum class mbInputs={atTop, atBottom, safetyTripped, isCalibrationMode, _LAST};
enum class mbHoldingRegisters={baudRate100x, slaveId, servoZero, fullSpeedDelta, targetApproach10x, minPos, maxPos, stepSize, safetyInterval, reqTargetPosition, reqPercentPosition,  _LAST};
enum class mbInputRegisters={currentPosition, currentPercentPosition, currentFalseCounter, currentTarget, _LAST}

struct config {
    uint16_t magic;
    uint8_t version;
    uint16_t baudRate100x;
    uint8_t slaveId;
    uint8_t servoZero;
    uint8_t fullSpeedDelta;
    uint8_t targetApproach10x;
    int16_t lastPos;
    int16_t minPos;
    int16_t maxPos;
    uint8_t stepSize;
    uint16_t safetyInterval;
    bool safetyTripped;
};

struct config defaultConfig {
    GARCI_MAGIC, 
    GARCI_VERSION,
    96,                 //baudRate100x 
    1,                  //slaveId
    128,                //servoZero
    16,                 //fullSpeedDelta
    30,                 //targetApproach10x
    0,                  //lastPos
    INT16_MIN,          //minPos
    INT16_MAX,          //maxPos
    16,                 //stepSize
    1000,               //safetyInterval
    false               //safetyTripped
};

struct config runningConfig;
ModbusRTUServer myMb;

int servoVal=0;
int servoDelta=0;
int16_t positionCounter=0;
int16_t falseCounter=0;
bool calibrationMode=false;
bool saveConfig=false;

int currentDirection=0;
int16_t targetPosition=0;

long lastMillis=0;
long printMillis=0;



void setup() {
    EEPROM.get(0,runningConfig);
    if !(runningConfig.magic==GARCI_MAGIC && runningConfig.version==GARCI_VERSION){
        runningConfig=defaultConfig;
        EEPROM.put(0,runningConfig);
    }
    positionCounter=runningConfig.lastPos;
    targetPosition=positionCounter;

    pinMode(COUNTERPIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(COUNTERPIN), updateCounter, CHANGE);

    if (!myMb.begin(runningConfig.slaveId , runningConfig.baudRate100x*100)) {
        Serial.println("Failed to start Modbus RTU Server!");
        while (1);
    }
    // configure coils at address 0x00
    myMb.configureCoils(0x00, mbCoils::_LAST);
    // configure discrete inputs at address 0x00
    myMb.configureDiscreteInputs(0x00, mbInputs::_LAST);
    // configure holding registers at address 0x00
    myMb.configureHoldingRegisters(0x00, mbHoldingRegisters::_LAST);
    // configure input registers at address 0x00
    myMb.configureInputRegisters(0x00, mbInputRegisters::_LAST);
}


void loop() {
    myMb.poll();

    for (mbInputRegisters myInReg=0; i++; i<_LAST){
        switch (myInReg)
        {
        case currentPosition:
            myMb.inputRegisterWrite(myInReg, positionCounter);
            break;
        case currentFalseCounter:
            myMb.inputRegisterWrite(myInReg, falseCounter);
            break;
        default:
            break;
        }
    }

    for (mbCoils myCoil=0; i++; i<_LAST){
        if myMb.coilRead(myCoil){                   //Coil was set to 1 - command triggered
            myMb.coilWrite(myCoil,0);               //Reset the coil to 0. Coils used a push-button cmnd triggers
            switch (myCoil)
            {
            case cmdStepUp:
                targetPosition=moveTargetClamped(targetPosition,runningConfig.stepSize);
                myMb.inputRegisterWrite(mbInputRegisters::currentTarget,targetPosition);
                break;
            case cmdStepDown:
                targetPosition=moveTargetClamped(targetPosition,-runningConfig.stepSize);
                myMb.inputRegisterWrite(mbInputRegisters::currentTarget,targetPosition);
                break;
            case cmdToggleCalibrationMode:
                calibrationMode=!calibrationMode;
                break;
            case cmdStoreMinPos:
                runningConfig.minPos=positionCounter;
                if (runningConfig.minPos>runningConfig.maxPos) {
                    runningConfig.maxPos=runningConfig.minPos;
                    myMb.holdingRegisterWrite(mbHoldingRegisters::maxPos,runningConfig.maxPos);
                }
                myMb.holdingRegisterWrite(mbHoldingRegisters::minPos,runningConfig.minPos);
                break;
            case cmdStoreMaxPos:
                runningConfig.maxPos=positionCounter;
                if (runningConfig.maxPos<runningConfig.minPos){
                    runningConfig.minPos=runningConfig.maxPos;
                    myMb.holdingRegisterWrite(mbHoldingRegisters::minPos,runningConfig.minPos);
                }
                myMb.holdingRegisterWrite(mbHoldingRegisters::maxPos,runningConfig.maxPos);
                break;
            case cmdResetTripped:
                runningConfig.safetyTripped=false;
                saveConfig=true;
                break;
            case cmdResetPosition:
                positionCounter=0;
                targetPosition=0;
                runningConfig.minPos=defaultConfig.minPos;
                runningConfig.maxPos=defaultConfig.maxPos;
                break;
            case cmdFactoryReset:
                runningConfig=defaultConfig;
                EEPROM.put(0,defaultConfig);
                resetFunc();
            case cmdSaveSettings:
                calibrationMode=false;
                saveConfig=true;
                break;
            case cmdReboot:
                resetFunc();
                break;
            default:
                break;
            }
        }

    }

    for (mbHoldingRegisters myReg=0; myReg++; myReg < _LAST) {
        uint16_t myRegValue=myMb.holdingRegisterRead(myReg);
        switch (myReg)
        {
        case baudRate100x:
            runningConfig.baudRate100x=myRegValue;
            break;
        case slaveId:
            runningConfig.slaveId=myRegValue;
            break;
        case servoZero:
            runningConfig.servoZero=myRegValue;
            break;
        case fullSpeedDelta:
            runningConfig.fullSpeedDelta=myRegValue;
            break;
        case targetApproach10x:
            runningConfig.targetApproach10x=myRegValue;
            break;
        case minPos:
            runningConfig.minPos=myRegValue;
            break;
        case maxPos:
            runningConfig.maxPos=myRegValue;
            break;
        case stepSize:
            runningConfig.stepSize=myRegValue;
            break;
        case safetyInterval:
            runningConfig.safetyInterval=myRegValue;
            break;
        case reqTargetPosition:
            if (myRegValue!=UINT16_MAX) {
                targetPosition=moveTargetClamped(myRegValue,0);
                myMb.holdingRegisterWrite(myReg,UINT16_MAX);
            }
            myMb.inputRegisterWrite(mbInputRegisters::currentTarget,targetPosition);
            break;
        case reqPercentPosition:
            if (myRegValue!=UINT16_MAX) {
                targetPosition=moveToPercentPos(myRegValue,);
                myMb.holdingRegisterWrite(myReg,UINT16_MAX);
            }
            myMb.inputRegisterWrite(mbInputRegisters::currentTarget,targetPosition);
            break;
        default:
            break;
        }
    }

    servoDelta=servoClamp(((targetPosition - positionCounter)*(int)runningConfig.targetApproach10x)/10);

    if (servoDelta >0){
        currentDirection=DIRECTION_UP;
    } else if (servoDelta <0) {
        currentDirection=DIRECTION_DN;
    } else {
        currentDirection=DIRECTION_STOP;
    }
    servoVal=runningConfig.servoZero+servoDelta;
    if (servoVal==0){
        digitalWrite(SERVOPIN,servoVal);
    } else {
        analogWrite(SERVOPIN,servoVal); 
    }
}

void updateCounter(){
    positionCounter+=currentDirection;
    if (!currentDirection){
        falseCounter++;
    }
}

int servoClamp(int servoDelta){
    if (servoDelta > runningConfig.fullSpeedDelta) {
        servoDelta=runningConfig.fullSpeedDelta;
    } else if (servoDelta < -runningConfig.fullSpeedDelta){
        servoDelta=-runningConfig.fullSpeedDelta;
    }
    return servoDelta; 
}

int16_t moveTargetClamped(int16_t startTarget, int16_t offset){
    int newTarget=(int)startTarget + int(offset);
    if (newTarget>INT16_MAX) newTarget=INT16_MAX;
    else if (newTarget<INT16_MIN) newTarget=INT16_MIN;
    if (!calibrationMode){
        if (newTarget>runningConfig.maxPos) newTarget=runningConfig.maxPos;
        else if (newTarget<runningConfig.minPos) newTarget=runningConfig.minPos;
    }
    return newTarget;
}

int16_t moveToPercentPos(int percentPos){
    if (percentPos>100) percentPos=100;
    if (percentPos<0) percentPos=0;
    int rangeOfMotion = (runningConfig.maxPos - runningConfig.minPos);
    return moveTargetClamped( rangeOfMotion* percentPos/100 + runningConfig.minPos,0);
}

void(* resetFunc) (void) = 0;