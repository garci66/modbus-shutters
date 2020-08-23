#include <Servo.h>
#define SERVOPIN 3
#define COUNTERPIN 2
#define POTPIN A0
#define DIRECTION_UP 1
#define DIRECTION_DN -1
#define DIRECTION_STOP 0
#define FULL_SPEED_DELTA 85
#define DIRECTION_MODE 0
#define TARGET_MODE 1

Servo myservo;
int servoVal=0;
int servoDelta=0;
int zeroPoint=90;
int positionCounter=0;
int falseCounter=0;


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
  // put your setup code here, to run once:
  myservo.attach(SERVOPIN);
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
    Serial.println(zeroPoint);
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
    }
    Serial.print("servoDelta:");
    Serial.println(servoDelta);
    Serial.print("Accel Interval:");
    Serial.print(accelInterval);
    Serial.print("targetApproach:");
    Serial.println(targetApproach);
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
    myservo.write(servoVal);
  } else {
    myservo.writeMicroseconds(0);
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
