double newPosition , PrePosition = 0; /// newPosition == y(t)    PrePosition = y(t-1)
int PinAmotor = 6 ; // motor power pins
int PinBmotor = 7 ;
int In1=12;
int In2=8;
String l;
String r;
String State;
int pot=0;
double PSetpoint=0;

double Setpoint=0;
//double newPosition;
//Define the aggressive and conservative Tuning Parameters
double aggKp=9, aggKi=0, aggKd=2;
double consKp=7, consKi=0, consKd=1;
double e=0;
double de=0;
double pe=0;
double ie=0;
double ep=0;
double dt=0.02046;
 double Output=0;
double pde=0;
double dde=0;
double Speed=0;
double PSpeed=0;


int encoderPin1 = 2;
int encoderPin2 = 3;
 
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
 
long lastencoderValue = 0;
 
int lastMSB = 0;
int lastLSB = 0;
 
void setup() {
  
  Serial.begin(115200);
  pinMode(PinAmotor , OUTPUT);
  pinMode(PinBmotor , OUTPUT);
  pinMode(pot,INPUT);
  digitalWrite(PinAmotor , LOW) ;
  digitalWrite(PinBmotor , LOW) ;
 
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
 
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
 
  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);



  pinMode(In1,OUTPUT);
pinMode(In2,OUTPUT);
digitalWrite(In1,HIGH);
digitalWrite(In2,HIGH);
 
}
 
void loop(){ 
  //Do stuff here
 if(Serial.available()>0){
 // Serial.println(encoderValue*359/2400);
  //delay(1); //just here to slow down the output, and show it will work  even during a delay
//for(int k=0;k<6;k++){
  //for(int i=0;i<200;i++){
 String Sett=Serial.readStringUntil('n');
String s0=Sett;
 s0.remove(3);
int S0=s0.toInt()-200;

String s1=Sett;
  s1.remove(0,3);
  State=s1;
  //S1=s1.toInt();

 if(State=="1"){
  //while(State=="l"){
  digitalWrite(In1,HIGH);
digitalWrite(In2,LOW);
//Serial.println(State);
}
   else if(State=="0"){

  digitalWrite(In1,LOW);
digitalWrite(In2,HIGH);
//Serial.println(State);

   }
    else if(State=="3"|State=="4"){

  digitalWrite(In1,HIGH);
digitalWrite(In2,HIGH);
//Serial.println(State);

   }

   else{
    digitalWrite(In1,HIGH);
digitalWrite(In2,HIGH);
//Serial.println("Error");
   }
  
































 
  
 double Time1 = micros();
  //for(int i=0;i<100;i++){
  //Input = analogRead(PIN_INPUT);
 // double myarrey[]={0.0,1.4,2.8,4.3,5.7,7.1,8.5,9.9,11.3,12.7,14.1,15.6,16.9,18.3,19.7,21.1,22.5,23.9,25.2,26.6,27.9,29.3,30.6,32.0,33.3,34.6,35.9,37.2,38.5,39.8,41.0,42.3,43.6,44.8,46.0,47.2,48.4,49.6,50.8,52.0,53.1,54.3,55.4,56.5,57.6,58.7,59.8,60.8,61.9,62.9,63.9,64.9,65.9,66.8,67.8,68.7,69.6,70.5,71.4,72.2,73.1,73.9,74.7,75.5,76.2,77.0,77.7,78.4,79.1,79.8,80.4,81.0,81.7,82.2,82.8,83.4,83.9,84.4,84.9,85.3,85.8,86.2,86.6,87.0,87.3,87.7,88.0,88.3,88.5,88.8,89.0,89.2,89.4,89.5,89.7,89.8,89.9,89.9,90.0,90.0,90.0,90.0,89.9,89.9,89.8,89.7,89.5,89.4,89.2,89.0,88.8,88.5,88.3,88.0,87.7,87.3,87.0,86.6,86.2,85.8,85.3,84.9,84.4,83.9,83.4,82.8,82.2,81.7,81.0,80.4,79.8,79.1,78.4,77.7,77.0,76.2,75.5,74.7,73.9,73.1,72.2,71.4,70.5,69.6,68.7,67.8,66.8,65.9,64.9,63.9,62.9,61.9,60.8,59.8,58.7,57.6,56.5,55.4,54.3,53.1,52.0,50.8,49.6,48.4,47.2,46.0,44.8,43.6,42.3,41.0,39.8,38.5,37.2,35.9,34.6,33.3,32.0,30.6,29.3,27.9,26.6,25.2,23.9,22.5,21.1,19.7,18.3,16.9,15.6,14.1,12.7,11.3,9.9,8.5,7.1,5.7,4.3,2.8,1.4,0.0};
//Setpoint=myarrey[i]*-3/5;
Setpoint=S0;
//Setpoint=analogRead(0);
//Setpoint=map(Setpoint,0,1023,0,90);
// long Pulses = myEnc.read();  // Read Encoder Data
  //Pulses = Pulses % 2000 ;      // Residual Respect to 800
 // newPosition = (double)Pulses / 2000 * 359 ;
  newPosition=encoderValue*359/2400;
  
Speed=(Setpoint-PSetpoint)/dt;
double acc=(Speed-PSpeed)/dt;


    //double gap0 = (Setpoint-newPosition); //distance away from setpoint

  double e = (Setpoint-newPosition);
//de=(PSetpoint-PrePosition)/dt;
de=(e-ep)/dt;
dde=(de-pde)/dt;
ie=ie+e;
  
  if (e < 1)
  {  //we're close to setpoint, use conservative tuning parameters

   Output= 0*acc+consKp*e+consKd*de+consKi*ie;
  }
  else
  {
     Output= 0*acc+aggKp*e+aggKd*de+aggKi*ie;
  }
//Output=map(Output)
  if (Output > 0) { // CW  rotation
//    double OU=abs(Output);
//    OU=map(OU,0,60,0,255);
//if(Output>255){
//  Output
//}
      analogWrite(PinAmotor ,abs(Output)) ; /// PWM for Speed Control
      digitalWrite(PinBmotor , LOW) ;
    }
    else if (Output < 0) { // CCW rotation

//        double OU=abs(Output);
//    OU=map(OU,0,60,0,255);
      analogWrite(PinAmotor , 255 - abs(Output)) ; /// PWM for Speed Control
      digitalWrite(PinBmotor , HIGH) ;
    }

  PrePosition=newPosition;
  PSetpoint=Setpoint;
  PSpeed=Speed;
  ep=e;
  pde=de;
  //analogWrite(PIN_OUTPUT, Output);
  //delay(50);
//}
//Serial.print("Set is=  ");
//Serial.print(Setpoint);
//Serial.print(" Actual is= ");
//Serial.println(newPosition);
//Serial.println(Output);
//Serial.println(Pulses);
Serial.println(State);

 delay(20);
   double Time2 = micros();
// Serial.println(Time2-Time1);
dt=Time2-Time1;
  //}
//delay(1000);
  //}
   /*analogWrite(PinAmotor , 0) ; /// PWM for Speed Control
      digitalWrite(PinBmotor , LOW) ;
  exit(0);*/

}

//else{
//
//   analogWrite(PinAmotor , 0) ; /// PWM for Speed Control
//      digitalWrite(PinBmotor , LOW) ;
//}
//}
 
}
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
 
  lastEncoded = encoded; //store this value for next time
}
