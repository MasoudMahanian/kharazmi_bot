//Serial Setting//////////////////////////////////////////////////////////////////////////////
String STR;
String STR2;
float STR1[19];
//Motors Set up//////////////////////////////////////////////////////////////////////////////////
double newP1 ,preP1=10 ,newP2 ,preP2=10 , newP3 ,preP3=10;  // Position for 3 motors
uint8_t m1R=2 , m1L=3 , m2R=4 , m2L=5 ,m3R=6 ,m3L=7; ///Pins of Motors from pwm 2 to pwm 7
double PSetpoint1=0 , Setpoint1=0 ,PSetpoint2=0 , Setpoint2=0 ,PSetpoint3=0 , Setpoint3=0; // Setpoints for 3 motors
///Control Gains//////////////////////////////////////////////////////////////////////////////////////////////////////////
//For motors
int k1,k2,k3;
double aggKp = 400, aggKi = 0, aggKd = 32;
double consKp = 360, consKi = 0, consKd = 30;
//Errors of Motors////////////////////////////////////////////////////////////
double e1=0 , e2=0 , e3=0 , de1=0 , de2=0 , de3=0, ep1=0 , ep2=0 , ep3=0 ; //Errors and diff errors and pre errors for 3 motors
double dt = 0.02046;
double Output1 = 0 , Output2 = 0 , Output3 = 0; // The pwm of 3 motors output
double Speed1 = 0 , Speed2 = 0 ,Speed3 = 0;
double PSpeed1 = 0 , PSpeed2 = 0 , PSpeed3 = 0;
//Encoder Constants///////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t encoderPin1 = 36 , encoderPin2 = 37 , encoderPin3 = 40 , encoderPin4 = 41 , encoderPin5 = 44 , encoderPin6 = 45;
volatile int lastEncoded1 = 0 , lastEncoded2 = 0 , lastEncoded3 = 0;
volatile long encoderValue1 = 0 ,encoderValue2 = 0 ,encoderValue3 = 0;
long lastencoderValue1 = 0 ,lastencoderValue2 = 0 ,lastencoderValue3 = 0  ;
int lastMSB1 = 0 ,lastMSB2 = 0 ,lastMSB3 = 0  ;
int lastLSB1 = 0 , lastLSB2 = 0 , lastLSB3 = 0;
//Interrupt EnCoder//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t but1 = 48 , but2 = 50 , but3 = 52 ; // interrupt input buts for 3 ,otors
const uint8_t Dbut1 = 49 ,Dbut2 = 51, Dbut3 = 53; // interrupt connect
volatile int lastEn1 = 0, lastEn2 = 0,lastEn3 = 0;
volatile long enVal1 = 0,enVal2 = 0,enVal3 = 0 ;
long lastenVal1 = 0 , lastenVal2 = 0 , lastenVal3 = 0;
int lastMB1 = 0 ,lastMB2 = 0 ,lastMB3 = 0  ;
int lastLB1 = 0 , lastLB2 = 0 , lastLB3 = 0;
////Jack constant///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define sensor1 A0 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define sensor2 A1 
#define sensor3 A2
uint8_t j1R=8 , j1L=9 , j2R=10 , j2L=11 ,j3R=12 ,j3L=13; ///Pins of Jacks from pwm 8 to pwm 13
double newPJ1 ,prePJ1=10 ,newPJ2 ,prePJ2=10 , newPJ3 ,prePJ3=10;  // Position for 3 motors
double PJSetpoint1=0 , JSetpoint1=0 ,PJSetpoint2=0 , JSetpoint2=0 ,PJSetpoint3=0 , JSetpoint3=0; // Setpoints for 3 motors
//Control Gains For jacks///////////////////////////
double aggKpj = 400, aggKij = 0, aggKdj = 32;
double consKpj = 360, consKij = 0, consKdj = 30;
//Errors of Motors////////////////////////////////////////////////////////////
double eJ1=0 , eJ2=0 , eJ3=0 , deJ1=0 , deJ2=0 , deJ3=0, epJ1=0 , epJ2=0 , epJ3=0 ; //Errors and diff errors and pre errors for 3 Jacks
double OutputJ1 = 0 , OutputJ2 = 0 , OutputJ3 = 0; // The pwm of 3 Jacks output
double SpeedJ1 = 0 , SpeedJ2 = 0, SpeedJ3 = 0;
double PSpeedJ1 = 0 , PSpeedJ2 = 0 , PSpeedJ3 = 0;
/////Reading data from sharp sensor no Noise
const int numReadings = 50;        // More is better but has delay
int vry_readings[numReadings];      // the readings from the analog input
int vry_readIndex = 0;              // the index of the current reading
int vry_total = 0;                  // the running total
int vry_average = 0;
/////Relay Pin 22 to 33***/////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t In1 = 22 , In2 = 24 , In3 = 26 , In4 = 28 , In5=30 ,In6 = 32 ;// left ot right Grip Orientation
uint8_t InOC1 = 23 , InOC2 = 25 , InOC3 = 27 , InOC4 = 29 , InOC5 = 31, InOC6 = 33 ;// OPen and Close Gripper
int State1=4 , State2=4 ,State3=4 ;
int  StateC1=4 , StateC2=4 ,StateC3=4;
//////////////////////////////////
String l;           /////////////
String r;                   ////
//int State = 3;              ////
//int State2 = 3;             ////
///////////////////////////////
int COUNT=0;

void setup() {
Serial1.begin(115200);
Serial.begin(115200);
//Motor Set up///////////////////////////////////////////////////////////////////////////////////////////////
pinMode(m1R , OUTPUT) ,pinMode(m1L , OUTPUT) ,pinMode(m2R , OUTPUT) ,pinMode(m2L , OUTPUT),pinMode(m3R , OUTPUT) ,pinMode(m3L , OUTPUT) ;
digitalWrite(m1R , LOW) ,digitalWrite(m1L , LOW), digitalWrite(m2R , LOW) ,digitalWrite(m2L, LOW) , digitalWrite(m3R , LOW) ,digitalWrite(m3L, LOW);
 //Interrupt Motor SetUp//////////////////////////////////////////////
  pinMode(but1, INPUT_PULLUP), pinMode(but2, INPUT_PULLUP),pinMode(but3, INPUT_PULLUP),pinMode(Dbut1, OUTPUT) ,pinMode(Dbut2, OUTPUT),pinMode(Dbut3, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(but1) , togg1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(but2) , togg2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(but3) , togg3, CHANGE);
 //Jack Set up///////////////////////////////////////////////////////////////////////////////////////////////
pinMode(j1R , OUTPUT) ,pinMode(j1L , OUTPUT) ,pinMode(j2R , OUTPUT) ,pinMode(j2L , OUTPUT),pinMode(j3R , OUTPUT) ,pinMode(j3L , OUTPUT) ;
digitalWrite(j1R , LOW) ,digitalWrite(j1L , LOW), digitalWrite(j2R , LOW) ,digitalWrite(j2L, LOW) , digitalWrite(j3R , LOW) ,digitalWrite(j3L, LOW);
//Encoder Set up/////////////////////////////////////////////////////////////////////////////////////////////
  pinMode(encoderPin1, INPUT),pinMode(encoderPin2, INPUT),   pinMode(encoderPin3, INPUT),pinMode(encoderPin4, INPUT) ,  pinMode(encoderPin5, INPUT),pinMode(encoderPin6, INPUT);
  digitalWrite(encoderPin1, HIGH), digitalWrite(encoderPin2, HIGH),  digitalWrite(encoderPin3, HIGH), digitalWrite(encoderPin4, HIGH), digitalWrite(encoderPin5, HIGH), digitalWrite(encoderPin6, HIGH) ; //turn pullup resistor on
  attachInterrupt(digitalPinToInterrupt(36), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(37), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(40), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(41), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(44), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(45), updateEncoder3, CHANGE);
//Relay Set up/////////////////////////////////////////////////////////////////////////////////////////////
  pinMode(In1, OUTPUT), pinMode(In2, OUTPUT) ,pinMode(In3, OUTPUT), pinMode(In4, OUTPUT), pinMode(In5, OUTPUT),pinMode(In6, OUTPUT) ,pinMode(InOC1, OUTPUT),pinMode(InOC2, OUTPUT),pinMode(InOC3, OUTPUT),pinMode(InOC4, OUTPUT),pinMode(InOC5, OUTPUT),pinMode(InOC6, OUTPUT) ;
  digitalWrite(In1, HIGH),  digitalWrite(In2, HIGH), digitalWrite(In3, HIGH), digitalWrite(In4, HIGH), digitalWrite(In5, HIGH), digitalWrite(In6, HIGH) ,digitalWrite(InOC1, HIGH), digitalWrite(InOC2, HIGH) ,digitalWrite(InOC3, HIGH) ,digitalWrite(InOC4, HIGH) ,digitalWrite(InOC5, HIGH),digitalWrite(InOC6, HIGH);
 ////////////////////////////////////////////////////////////////////////
 for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    vry_readings[thisReading] = 0;

  }
//////////////////////////////////////////////////////////////////////////
}

void loop() {
  if (Serial1.available()) {
    //Reading the incoming Serial
    String STR = Serial1.readStringUntil('n');
    STR2 = STR;
    for (int i = 0; i < 17; i++) {
      STR.remove(6);
      STR1[i] = STR.toFloat() - 2000;
      STR2.remove(0, 6);
      STR = STR2;
    }
  }
double Time1 = micros();
///InterruptM1/////////////////////////////////////////////////////////////////////////////////////////////
 
 digitalWrite(Dbut1, int (STR1[3]));
 digitalWrite(Dbut2, int (STR1[4]));
 digitalWrite(Dbut3, int (STR1[5]));
////////Motor Control/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Setpoint1=STR1[0];
Setpoint2=STR1[1];
Setpoint3=STR1[2];
k1=STR1[15];//*******************************the gains for passive joints///////////////////////////////
k2=STR1[16];
k3=STR1[17];
//////////////////////Motor1//////////////////////////////////////////
Motor(m1R,m1L,Setpoint1,PSetpoint1,encoderValue1,newP1,preP1,Speed1 ,PSpeed1, e1,ep1,de1,Output1,k1);
 preP1=newP1;
 PSetpoint1=Setpoint1;
 PSpeed1=Speed1;
 ep1=e1;
///////////////////////Motor2////////////////////////////////////////
 Motor(m2R,m2L,Setpoint2,PSetpoint2,encoderValue2,newP2,preP2,Speed2 ,PSpeed2, e2,ep2,de2,Output2,k2);
 preP2=newP2;
 PSetpoint2=Setpoint2;
 PSpeed2=Speed2;
 ep2=e2;
 ///////////////////////Motor3////////////////////////////////////////
 Motor(m3R,m3L,Setpoint3,PSetpoint3,encoderValue3,newP3,preP3,Speed3 ,PSpeed3, e3,ep3,de3,Output3,k3);
 preP3=newP3;
 PSetpoint3=Setpoint3;
 PSpeed3=Speed3;
 ep3=e3;
/////Jack//////////////////////////////////////////////////////////////////////////////////////
JSetpoint1=int (STR1[12]);
JSetpoint1=int (STR1[13]);
JSetpoint1=int (STR1[14]);
float volts1 = analogRead(sensor1) * 0.0048828125 * 3.3 / 5; // value from sensor * (5/1024)
float volts2 = analogRead(sensor2) * 0.0048828125 * 3.3 / 5; // value from sensor * (5/1024)
float volts3 = analogRead(sensor3) * 0.0048828125 * 3.3 / 5; // value from sensor * (5/1024)
////JACK 1 /////////////////////////////////
Jack(j1R,j1L,JSetpoint1,PJSetpoint1,newPJ1,prePJ1,SpeedJ1 ,PSpeedJ1, eJ1,epJ1,deJ1,OutputJ1,volts1);
prePJ1 = newPJ1;
  PJSetpoint1 = JSetpoint1;
  PSpeedJ1 = SpeedJ1;
  epJ1 = eJ1;
 ///JACK 2 /////////////////////////////////
Jack(j2R,j2L,JSetpoint2,PJSetpoint2,newPJ2,prePJ2,SpeedJ2 ,PSpeedJ2, eJ2,epJ2,deJ2,OutputJ2,volts2);
prePJ2 = newPJ2;
  PJSetpoint2 = JSetpoint2;
  PSpeedJ2 = SpeedJ2;
  epJ2 = eJ2;
  ///JACK 3 /////////////////////////////////
Jack(j3R,j3L,JSetpoint3,PJSetpoint3,newPJ3,prePJ3,SpeedJ3 ,PSpeedJ3, eJ3,epJ3,deJ3,OutputJ3,volts3);
prePJ3 = newPJ3;
  PJSetpoint3 = JSetpoint3;
  PSpeedJ3 = SpeedJ3;
  epJ3 = eJ3;
/////GRIPPERS////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
State1=int(STR1[6]);
State2=int(STR1[7]);
State3=int(STR1[8]);
StateC1=int(STR1[9]);
StateC2=int(STR1[10]);
StateC3=int(STR1[11]);
/////GRipperS////////////////////////////////
Gripper(In1 ,In2,State1);
Gripper(In3 ,In4,State2);
Gripper(In5 ,In6,State3);
Gripper(InOC1 ,InOC2,StateC1);
Gripper(InOC3 ,InOC4,StateC2);
Gripper(InOC5 ,InOC6,StateC3);
//if(COUNT%1000==0){
Serial.print(" M1 is=  ");
Serial.print(Setpoint1);
Serial.print(" M2 is= ");
Serial.print(Setpoint2);
Serial.print(" M3 is= ");
Serial.print(Setpoint3);
Serial.print(" J1 is= ");
Serial.print(JSetpoint1);
Serial.print(" J2 is= ");
Serial.print(JSetpoint2);
Serial.print(" J3 is= ");
Serial.print(JSetpoint3);
Serial.print(" State1 is= ");
//Serial.print(State1);
//Serial.print(" State2 is= ");
//Serial.print(State2);
//Serial.print(" State3 is= ");
//Serial.print(State3);
//Serial.print(" StateC1 is= ");
//Serial.print(StateC1);
//Serial.print(" StateC2 is= ");
//Serial.print(StateC2);
//Serial.print(" StateC3 is= ");
//Serial.print(StateC3);
//Serial.print(" k1 is= ");
//Serial.print(k1);
//Serial.print(" k2 is= ");
//Serial.print(k2);
//Serial.print(" k3 is= ");
//Serial.print(k3);
Serial.print(" INT1 is= ");
Serial.print(int (STR1[3]));
Serial.print(" INT2 is= ");
Serial.print(int (STR1[4]));
Serial.print(" INT3 is= ");
Serial.println(int (STR1[5]));
//}
//COUNT=+1;
//if(COUNT>100000){
//  COUNT=0;
//}

double Time2 = micros();
dt = Time2 - Time1;
}
void updateEncoder1() {
  int MSB1 = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB1 = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded1 = (MSB1 << 1) | LSB1; //converting the 2 pin value to single number
  int sum1  = (lastEncoded1 << 2) | encoded1; //adding it to the previous encoded value

  if (sum1 == 0b1101 || sum1 == 0b0100 || sum1 == 0b0010 || sum1 == 0b1011) encoderValue1 ++;
  if (sum1 == 0b1110 || sum1 == 0b0111 || sum1 == 0b0001 || sum1 == 0b1000) encoderValue1 --;

  lastEncoded1 = encoded1; //store this value for next time
}
void updateEncoder2() {
  int MSB2 = digitalRead(encoderPin3); //MSB = most significant bit
  int LSB2 = digitalRead(encoderPin4); //LSB = least significant bit

  int encoded2 = (MSB2 << 1) | LSB2; //converting the 2 pin value to single number
  int sum2  = (lastEncoded2 << 2) | encoded2; //adding it to the previous encoded value

  if (sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) encoderValue2 ++;
  if (sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) encoderValue2 --;

  lastEncoded2 = encoded2; //store this value for next time
}
void updateEncoder3() {
  int MSB3 = digitalRead(encoderPin5); //MSB = most significant bit
  int LSB3 = digitalRead(encoderPin6); //LSB = least significant bit

  int encoded3 = (MSB3 << 1) | LSB3; //converting the 2 pin value to single number
  int sum3  = (lastEncoded3 << 2) | encoded3; //adding it to the previous encoded value

  if (sum3 == 0b1101 || sum3 == 0b0100 || sum3 == 0b0010 || sum3 == 0b1011) encoderValue3 ++;
  if (sum3 == 0b1110 || sum3 == 0b0111 || sum3 == 0b0001 || sum3 == 0b1000) encoderValue3 --;

  lastEncoded3 = encoded3; //store this value for next time
}
void togg1() {
  encoderValue1 = 0;
  Setpoint1=0;
}
void togg2() {
  encoderValue2 = 0;
  Setpoint2=0;
}
void togg3() {
  encoderValue3 = 0;
  Setpoint3=0;
}

void Motor(uint8_t MR , uint8_t ML , double SETPOINT ,double PSETPOINT ,volatile long  ENCODERVALUE, double & NEWP ,double PREP, double & SPEED , double  PSPEED ,double &E , double EP , double  DE , double &OUTPUTT, int K){
  if(MR==2 || MR==3){
    NEWP=ENCODERVALUE*359/1440;
  }
  else if(MR==1){
  NEWP=ENCODERVALUE*359/2400;
  }
SPEED=(SETPOINT-PSETPOINT)/dt;
E=SETPOINT-NEWP;
DE=(E-EP)/dt;
analogWriteResolution(12);
if(abs(E)<1){
  OUTPUTT=K*(consKp*E+consKd*DE);
}
else{
  OUTPUTT=K*(aggKp*E+aggKd*DE);
}
if(OUTPUTT>=0){ //CW rotation (RIGHT)
  if(OUTPUTT>255){
    analogWrite(MR , 255) ; /// PWM for Speed Control
analogWrite(ML , 0) ; 
  }
  else{
analogWrite(MR , abs(OUTPUTT)) ; /// PWM for Speed Control
analogWrite(ML , 0) ; 
}
}
else if(OUTPUTT<=0){//CCW ROTATION (LEFT)
  if(abs(OUTPUTT)>255){
    analogWrite(MR , 0) ; /// PWM for Speed Control
analogWrite(ML , 255) ;
  }
  else{
analogWrite(MR , 0) ; /// PWM for Speed Control
analogWrite(ML , abs(OUTPUTT)) ;
}
}
}
void Jack(uint8_t JR , uint8_t JL , double JSETPOINT ,double PJSETPOINT, double & NEWPJ ,double PREPJ, double & SPEEDJ , double  PSPEEDJ ,double &EJ , double EPJ , double  DEJ , double &OUTPUTTJ,float VOLTS){
analogWriteResolution(12);
///CALCULATING DISTANCE *******************************
 vry_total = vry_total - vry_readings[vry_readIndex];
  float distance = 13 * pow(VOLTS, -1) * 10; // worked out from datasheet graph
 vry_readings[vry_readIndex] = distance;
  vry_total = vry_total + vry_readings[vry_readIndex];
  vry_readIndex += 1;
  if (vry_readIndex >= numReadings) {
    // ...wrap around to the beginning:
    vry_readIndex = 0;
  }
  vry_average = vry_total / numReadings;
  NEWPJ = vry_average-20-30;//20  for regulayion and 30 for destance of end blue
  SPEEDJ = (JSETPOINT - PJSETPOINT) / dt;
  EJ=JSETPOINT-NEWPJ;
  DEJ=(EJ-EPJ)/dt;
if (NEWPJ>50 && NEWPJ<1500){//****************************************THE ALLOWABLE AREA*******************CHECK IT  

if(JSETPOINT>2999){//"********************STOP DATA *******************" 
digitalWrite(JR,LOW);
digitalWrite(JL,LOW);
}
 else if(JSETPOINT==2100){//"***********HALF SPEED FORWARD (RIGHT)****************"
   analogWrite(JR,255/2);
   analogWrite(JL,0); 
 }
 else if(JSETPOINT==1600){//"***********HALF SPEED BACKWARD  (LEFT)****************"
   analogWrite(JR,0);
   analogWrite(JL,255/2); 
 }
  else if(JSETPOINT==2500){//"***********FULL SPEED FORWARD (RIGHT)****************"
   analogWrite(JR,255);
   analogWrite(JL,0); 
 }
 else if(JSETPOINT==2000){//"***********FULL SPEED BACKWARD (LEFT)****************"
   analogWrite(JR,0);
   analogWrite(JL,255); 
 }

 else if(JSETPOINT<1599){// "LIMIT NUMBER FOR POSITION CONTROL"
 
if(abs(EJ)<5){
  if(abs(EJ)<3){
OUTPUTTJ=0.1;
  }
  else{
    OUTPUTTJ=consKpj*EJ+consKdj*DEJ;
    }
  }
  else{
        OUTPUTTJ=aggKpj*EJ+aggKdj*DEJ;
}
if(OUTPUTTJ>0){///Should go to Right
  if(OUTPUTTJ>255){
     analogWrite(JR,255);
   analogWrite(JL,0);
  }
  else{
   analogWrite(JR,abs(OUTPUTTJ));
   analogWrite(JL,0);
}
}
else if(OUTPUTTJ<=0){
  if(abs(OUTPUTTJ>255)){
     analogWrite(JR,0);
   analogWrite(JL,255);
  }
   else{
   analogWrite(JR,0);
   analogWrite(JL,abs(OUTPUTTJ));
}
}
 }
}
else{
  analogWrite(JR,0);
   analogWrite(JL,0);
}
}
void Gripper(double INO,double INC, int STATE){
   if (STATE ==1) {// "*******OPEN GRIP OR CCW ************"
    digitalWrite(INO, LOW);
    digitalWrite(INC, HIGH);
  }
  else if (STATE ==0 ) {//"********CLOSE GRIP OR CW *******"
    digitalWrite(INO, HIGH);
    digitalWrite(INC, LOW);
  }
  else if (STATE == 3 || STATE == 4) {//4"*********STOP CONDITION"

    digitalWrite(INO, HIGH);
    digitalWrite(INC, HIGH);
  }
  else {
    digitalWrite(INO, HIGH);
    digitalWrite(INC, HIGH);
  }
}

