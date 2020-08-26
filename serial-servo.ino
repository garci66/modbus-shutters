#include "serial-servo.h"

#define SERVOPIN 3
#define COUNTERPIN 2

ModbusRTUServerClass myMb;

struct config runningConfig;

int servoVal=0;
int servoDelta=0;
int16_t positionCounter=0;
int16_t falseCounter=0;
bool calibrationMode=false;
bool saveConfig=false;
unsigned long lastCounterTick;
unsigned long lastMotionStart;
unsigned long lastModbusUpdate=0;
int lastDirection=0,currentDirection=0;
int16_t targetPosition=0;


void setup() {
    EEPROM.get(0,runningConfig);
    if (!(runningConfig.magic==GARCI_MAGIC && runningConfig.version==GARCI_VERSION)){
        runningConfig=defaultConfig;
        EEPROM.put(0,runningConfig);
    }
    positionCounter=runningConfig.lastPos;
    targetPosition=positionCounter;

    pinMode(COUNTERPIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(COUNTERPIN), updateCounter, CHANGE);
    pinMode(SERVOPIN, OUTPUT);

    TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = runningConfig.pwmPreScaler & 0x7;

    if (!myMb.begin(runningConfig.slaveId , runningConfig.baudRate100x*100)) {
        Serial.println("Failed to start Modbus RTU Server!");
        while (1);
    }
    // configure coils at address 0x00
    myMb.configureCoils(0x00, (int)mbCoils::_LAST);
    // configure discrete inputs at address 0x00
    myMb.configureDiscreteInputs(0x00, (int)mbInputs::_LAST);
    // configure holding registers at address 0x00
    myMb.configureHoldingRegisters(0x00, (int)mbHoldingRegisters::_LAST);
    // configure input registers at address 0x00
    myMb.configureInputRegisters(0x00, (int)mbInputRegisters::_LAST);
    
    updateMBHoldingRegsFromConfig();
}


void loop() {
    myMb.poll();

    if(millis()>lastModbusUpdate + MODBUS_UPDATE_INTERVAL){
        updateMBInputs();
        updateMBInputRegs();
        updateMBCoils();
        updateMBHoldingRegs();
    }

    if (runningConfig.safetyTripped){
        servoDelta=0;
    } else {
        servoDelta=servoClamp(((targetPosition - positionCounter)*(int)runningConfig.targetApproach10x)/10);
    }

    lastDirection=currentDirection;
    if (servoDelta >0){
        currentDirection=DIRECTION_UP;
    } else if (servoDelta <0) {
        currentDirection=DIRECTION_DN;
    } else {
        currentDirection=DIRECTION_STOP;
        if (lastDirection!=currentDirection){
            saveConfig=true;
        }
    }

    if (currentDirection && (lastDirection!=currentDirection)){
        //we just started moving so lets make sure we dont trip the safetyInterval accidentally;
        lastCounterTick=millis();
        lastMotionStart=lastCounterTick;
    }

    servoVal=runningConfig.servoZero+servoDelta;
    if (servoDelta==0 || runningConfig.safetyTripped){
        OCR2B = 0xff;
    } else {
        OCR2B = servoVal;
    }

    // if (!runningConfig.safetyTripped){
    //     if (currentDirection && (millis()> (lastCounterTick + runningConfig.safetyInterval))){
    //         //We should be moving and we arent!
    //         runningConfig.safetyTripped=true;
    //         saveConfig=true;
    //     }
    //     if (currentDirection && millis()> lastMotionStart + runningConfig.maxRuntimeS*1000){
    //         runningConfig.safetyTripped=true;
    //         saveConfig=true;
    //     }
    // }

    if (saveConfig && !calibrationMode){
        runningConfig.lastPos=positionCounter;
        EEPROM.put(0,runningConfig);
        saveConfig=false;
    }

}

void updateCounter(){
    positionCounter+=currentDirection;
    lastCounterTick=millis();
    if (!currentDirection){
        falseCounter++;
    }
}

int servoClamp(int servoDelta){
    if (servoDelta > runningConfig.fullSpeedDelta) {
        servoDelta=runningConfig.fullSpeedDelta;
    } else if (servoDelta < -runningConfig.fullSpeedDelta){
        servoDelta=-runningConfig.fullSpeedDelta;
    } else if(abs(servoDelta)<=runningConfig.deadBand) {
        servoDelta=0;
    }
    return servoDelta; 
}

int16_t moveTargetClamped(int16_t startTarget, int16_t offset){
    int newTarget=(int)startTarget + (int)offset;
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

void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void updateMBInputs(void){
    for (mbInputs myInput=mbInputs::_START; myInput<mbInputs::_LAST; myInput++){
        switch (myInput)
        {
        case mbInputs::atBottom:
            myMb.discreteInputWrite((int)myInput, positionCounter==runningConfig.minPos);
            break;
        case mbInputs::atTop:
            myMb.discreteInputWrite((int)myInput, positionCounter==runningConfig.maxPos);
            break;
        case mbInputs::safetyTripped:
            myMb.discreteInputWrite((int)myInput, runningConfig.safetyTripped);
            break;
        case mbInputs::isCalibrationMode:
            myMb.discreteInputWrite((int)myInput, calibrationMode);
            break;
        case mbInputs::allwaysTrue:
            myMb.discreteInputWrite((int)myInput, true);
            break;
        default:
            break;
        }
    }
}

void updateMBInputRegs(void){
    for (mbInputRegisters myInReg=mbInputRegisters::_START; myInReg<mbInputRegisters::_LAST; myInReg++){
        switch (myInReg)
        {
        case mbInputRegisters::currentPosition:
            myMb.inputRegisterWrite((int)myInReg, positionCounter);
            break;
        case mbInputRegisters::currentFalseCounter:
            myMb.inputRegisterWrite((int)myInReg, falseCounter);
            break;
        case mbInputRegisters::currentTarget:
            myMb.inputRegisterWrite((int)myInReg, targetPosition);
            break;
        case mbInputRegisters::currentDelta:
            myMb.inputRegisterWrite((int)myInReg, servoDelta);
            break;
        case mbInputRegisters::currentServoValue:
            myMb.inputRegisterWrite((int)myInReg, servoVal);
            break;
        default:
            break;
        }
    }
}

void updateMBCoils(void){
    for (mbCoils myCoil=mbCoils::cmdStepUp; myCoil<mbCoils::_LAST; myCoil++){
        if (myMb.coilRead((int)myCoil)){                   //Coil was set to 1 - command triggered
            myMb.coilWrite((int)myCoil,0);               //Reset the coil to 0. Coils used a push-button cmnd triggers
            switch (myCoil)
            {
            case mbCoils::cmdStepUp:
                targetPosition=moveTargetClamped(targetPosition,runningConfig.stepSize);
                break;
            case mbCoils::cmdStepDown:
                targetPosition=moveTargetClamped(targetPosition,-runningConfig.stepSize);
                break;
            case mbCoils::cmdToggleCalibrationMode:
                calibrationMode=!calibrationMode;
                break;
            case mbCoils::cmdStoreMinPos:
                runningConfig.minPos=positionCounter;
                if (runningConfig.minPos>runningConfig.maxPos) {
                    runningConfig.maxPos=runningConfig.minPos;
                    myMb.holdingRegisterWrite((int)mbHoldingRegisters::maxPos,runningConfig.maxPos);
                }
                myMb.holdingRegisterWrite((int)mbHoldingRegisters::minPos,runningConfig.minPos);
                break;
            case mbCoils::cmdStoreMaxPos:
                runningConfig.maxPos=positionCounter;
                if (runningConfig.maxPos<runningConfig.minPos){
                    runningConfig.minPos=runningConfig.maxPos;
                    myMb.holdingRegisterWrite((int)mbHoldingRegisters::minPos,runningConfig.minPos);
                }
                myMb.holdingRegisterWrite((int)mbHoldingRegisters::maxPos,runningConfig.maxPos);
                break;
            case mbCoils::cmdResetTripped:
                runningConfig.safetyTripped=false;
                saveConfig=true;
                break;
            case mbCoils::cmdResetPosition:
                positionCounter=0;
                targetPosition=0;
                runningConfig.minPos=defaultConfig.minPos;
                runningConfig.maxPos=defaultConfig.maxPos;
                break;
            case mbCoils::cmdFactoryReset:
                runningConfig=defaultConfig;
                EEPROM.put(0,defaultConfig);
                reboot();
            case mbCoils::cmdSaveSettings:
                calibrationMode=false;
                saveConfig=true;
                break;
            case mbCoils::cmdReboot:
                reboot();
                break;
            default:
                break;
            }
        }
    }
}

void updateMBHoldingRegs(void){
    for (mbHoldingRegisters myReg=mbHoldingRegisters::_START; myReg < mbHoldingRegisters::_LAST; myReg++) {
        uint16_t myRegValue=myMb.holdingRegisterRead((int)myReg);
        switch (myReg)
        {
        case mbHoldingRegisters::baudRate100x:
            runningConfig.baudRate100x=myRegValue;
            break;
        case mbHoldingRegisters::slaveId:
            runningConfig.slaveId=myRegValue;
            break;
        case mbHoldingRegisters::pwmPreScaler:
            if (runningConfig.pwmPreScaler!=myRegValue &0x7){
                runningConfig.pwmPreScaler=myRegValue &0x7;
                TCCR2B = runningConfig.pwmPreScaler;
            }
            break;
        case mbHoldingRegisters::servoZero:
            runningConfig.servoZero=myRegValue;
            break;
        case mbHoldingRegisters::fullSpeedDelta:
            runningConfig.fullSpeedDelta=myRegValue;
            break;
        case mbHoldingRegisters::targetApproach10x:
            runningConfig.targetApproach10x=myRegValue;
            break;
        case mbHoldingRegisters::deadBand:
            runningConfig.deadBand=myRegValue;
            break;
            case mbHoldingRegisters::minPos:
            runningConfig.minPos=myRegValue;
            break;
        case mbHoldingRegisters::maxPos:
            runningConfig.maxPos=myRegValue;
            break;
        case mbHoldingRegisters::stepSize:
            runningConfig.stepSize=myRegValue;
            break;
        case mbHoldingRegisters::safetyInterval:
            runningConfig.safetyInterval=myRegValue;
            break;
        case mbHoldingRegisters::maxRuntimeS:
            runningConfig.maxRuntimeS=myRegValue;
            break;
        case mbHoldingRegisters::reqTargetPosition:
            if (myRegValue!=UINT16_MAX) {
                targetPosition=moveTargetClamped(myRegValue,0);
                myMb.holdingRegisterWrite((int)myReg,UINT16_MAX);
            }
            break;
        case mbHoldingRegisters::reqPercentPosition:
            if (myRegValue!=UINT16_MAX) {
                targetPosition=moveToPercentPos(myRegValue);
                myMb.holdingRegisterWrite((int)myReg,UINT16_MAX);
            }
            break;
        default:
            break;
        }
    }
}


void updateMBHoldingRegsFromConfig(void){
    for (mbHoldingRegisters myReg=mbHoldingRegisters::_START; myReg < mbHoldingRegisters::_LAST; myReg++) {
        switch (myReg)
        {
        case mbHoldingRegisters::baudRate100x:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.baudRate100x);  
            break;
        case mbHoldingRegisters::slaveId:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.slaveId);  
            break;
        case mbHoldingRegisters::pwmPreScaler:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.pwmPreScaler);  
            break;
        case mbHoldingRegisters::servoZero:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.servoZero);  
            break;
        case mbHoldingRegisters::fullSpeedDelta:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.fullSpeedDelta);  
            break;
        case mbHoldingRegisters::targetApproach10x:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.targetApproach10x);  
            break;
        case mbHoldingRegisters::deadBand:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.deadBand);  
            break;
        case mbHoldingRegisters::minPos:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.minPos);  
            break;
        case mbHoldingRegisters::maxPos:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.maxPos);  
            break;
        case mbHoldingRegisters::stepSize:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.stepSize);  
            break;
        case mbHoldingRegisters::safetyInterval:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.safetyInterval);  
            break;
        case mbHoldingRegisters::maxRuntimeS:
            myMb.holdingRegisterWrite((int)myReg,runningConfig.maxRuntimeS);  
            break;
        case mbHoldingRegisters::reqTargetPosition:
            myMb.holdingRegisterWrite((int)myReg,UINT16_MAX);  
            break;
        case mbHoldingRegisters::reqPercentPosition:
            myMb.holdingRegisterWrite((int)myReg,UINT16_MAX);  
            break;
        default:
            break;
        }
    }
}
