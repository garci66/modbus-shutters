#include <Servo.h>
#define SERVOPIN 3
#define COUNTERPIN 2
#define POTPIN A0
#define DIRECTION_UP 1
#define DIRECTION_DN -1
#define DIRECTION_STOP 0
#define FULL_SPEED_DELTA 126
#define DIRECTION_MODE 0
#define TARGET_MODE 1

//Servo myservo;
uint8_t servoVal=0;
int8_t servoDelta=0;
uint8_t zeroPoint=184;
int positionCounter=0;
int falseCounter=0;
uint8_t prescaler=  _BV(CS22) | _BV(CS20) ;
bool invertPwm=true;


int operMode=DIRECTION_MODE;
int currentDirection=0;
int desiredDirection=0;
int targetPosition=0;
float targetApproach=3.0;
bool power=true;

int accelInterval=200;

long lastMillis=0;
long printMillis=0;

int potread=0;
char keyRead;
void setup() {
  pinMode(SERVOPIN, OUTPUT);
  pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2B1) | (_BV(COM2B0)&& invertPwm) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = prescaler;
  
  // put your setup code here, to run once:
  //myservo.attach(SERVOPIN);
  Serial.begin(115200);
  //while (!Serial);
  Serial.println("Finished init");
  pinMode(POTPIN, INPUT);
  pinMode(COUNTERPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(COUNTERPIN), updateCounter, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:

//  potread=analogRead(POTPIN);
//  if (potread>500){
//    desiredDirection=DIRECTION_UP;
//  } else if (potread <200) {
//    desiredDirection=DIRECTION_DN;
//  } else {
//    desiredDirection=DIRECTION_STOP;
//  }

  if (operMode==TARGET_MODE){
    servoDelta=clamp( (targetPosition - positionCounter)*targetApproach);
  }

//  if (operMode==DIRECTION_MODE && desiredDirection){
//    if (millis() > (lastMillis + accelInterval)){
//      servoDelta+=desiredDirection;
//      lastMillis=millis();
//    }
//    servoDelta=clamp(servoDelta);   
//  } else if (servoDelta) {
//    if (millis() > (lastMillis + accelInterval)){
//      servoDelta+=(servoDelta>0)?DIRECTION_DN:DIRECTION_UP;
//      lastMillis=millis();
//    } 
//  }

  if (millis()>printMillis+1000){
    Serial.print(servoDelta);
    Serial.print(" ");
    Serial.print(power);
    Serial.print(" ");
    Serial.print(servoVal);
    Serial.print(" ");
    Serial.print(zeroPoint);
    Serial.print(" ");
    Serial.print(TCCR2A,BIN);
    Serial.print(" ");
    Serial.print(TCCR2B,BIN);
    Serial.print(" ");
    Serial.print(OCR2A,BIN);
    Serial.print(" ");
    Serial.println(OCR2B,BIN);

    
    printMillis=millis();
  }
  
  if (Serial.available()) {
    keyRead=Serial.read();
    switch (keyRead)
    {
      case ']':
        targetPosition++;
        break;
      case '[':
        targetPosition--;
        break; 
      case '0':
        servoDelta=0;
        break; 
      case '1':
        servoVal=zeroPoint;
        break; 
      case '2':
        servoVal=zeroPoint-FULL_SPEED_DELTA;
        break; 
      case '3':
        servoVal=zeroPoint+FULL_SPEED_DELTA;
        break; 
      case '5':
        TCCR2B = 4;
        break; 
      case '6':
        TCCR2B = 5;
        break; 
      case '7':
        TCCR2B = 6;
        break; 
      case '8':
        TCCR2B = 7;
        break; 
      case 'p':
        power=!power;
        break;
      case 'z':
        zeroPoint=servoVal;
        break;
      case 'u':
        //desiredDirection=DIRECTION_UP;
        //operMode=DIRECTION_MODE;
        servoDelta++;
        break;              
      case 'd':
        //desiredDirection=DIRECTION_DN;
        //operMode=DIRECTION_MODE;
        servoDelta--;
        break;
      case 's':
        desiredDirection=DIRECTION_STOP;
        operMode=DIRECTION_MODE;
        break;                
      case 'q':
        targetPosition=10;
        operMode=TARGET_MODE;
        break; 
      case 'w':
        targetPosition=40;
        operMode=TARGET_MODE;
        break; 
      case 'e':
        targetPosition=80;
        operMode=TARGET_MODE;
        break;
      case 'x':
        targetPosition=positionCounter;
        break;  
      case 'i':
        invertPwm=!invertPwm;
        if (invertPwm){
          TCCR2A = _BV(COM2B1) | _BV(COM2B0) | _BV(WGM21) | _BV(WGM20);  
        } else {
          TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  
        }
        
        break; 
    }
//    Serial.print("servoDelta:");
//    Serial.println(servoDelta);
//    Serial.print("Accel Interval:");
//    Serial.print(accelInterval);
//    Serial.print("targetApproach:");
//    Serial.println(targetApproach);
  }
  if (servoDelta >0){
    currentDirection=DIRECTION_UP;
  } else if (servoDelta <0) {
    currentDirection=DIRECTION_DN;
  } else {
    currentDirection=DIRECTION_STOP;
  }
  servoVal=zeroPoint+servoDelta;

  if (power){
      //analogWrite(SERVOPIN,servoVal);
      OCR2B = servoVal;
 //     OCR2A = servoVal;
  } else {
      //digitalWrite(SERVOPIN,1);
      //analogWrite(SERVOPIN,1);
      OCR2B = 0xff;
 //     OCR2A = 0xff;
  }


//   if (servoVal==0){
//     digitalWrite(SERVOPIN,servoVal);
//   } else {
//     analogWrite(SERVOPIN,servoVal); 
//   }

}


void updateCounter(){
  positionCounter+=currentDirection;
  if (!currentDirection){
    falseCounter++;
  }
}

int clamp(int servoDelta){
  if (servoDelta > FULL_SPEED_DELTA) {
    servoDelta=FULL_SPEED_DELTA;
  } else if (servoDelta < -FULL_SPEED_DELTA){
    servoDelta=-FULL_SPEED_DELTA;
  }
  return servoDelta; 
}
