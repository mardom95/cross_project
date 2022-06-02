#include <ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <timerManager.h>
#include <timer.h>

#include <Average.h>


#define speedPinR 9   //  Front Wheel PWM pin connect Right MODEL-X ENA 
#define RightMotorDirPin1  22    //Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define RightMotorDirPin2  24   //Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)                                 
#define LeftMotorDirPin1  26    //Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2  28   //Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)
#define speedPinL 10   //  Front Wheel PWM pin connect Right MODEL-X ENB

#define speedPinRB 11   //  Rear Wheel PWM pin connect Left MODEL-X ENA 
#define RightMotorDirPin1B  5    //Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
#define RightMotorDirPin2B 6    //Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1) 
#define LeftMotorDirPin1B 7    //Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
#define LeftMotorDirPin2B 8  //Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
#define speedPinLB 12    //  Rear Wheel PWM pin connect Left MODEL-X ENB

/*
* CROSS Arduino code
* 
* FRONT LEFT --> Advance -
* REAR LEFT --> Advance -
* FRONT RIGHT --> Advance +
* REAR RIGHT --> Advance +
* 
*/
#define HALLA_FL 30
#define HALLB_FL 31

#define HALLA_RL 32
#define HALLB_RL 33

#define HALLA_FR 34
#define HALLB_FR 35

#define HALLA_RR 36
#define HALLB_RR 37

//1 revolution 151 encoder pulses
#define PULSES_ONEREV 151

#define PERIOD 70

bool doControl = false;

int counter_FL = 0;
int counter_RL = 0;
int counter_FR = 0;
int counter_RR = 0;

int aFLState, aFLLastState;
int aRLState, aRLLastState;
int aFRState, aFRLastState;
int aRRState, aRRLastState;

unsigned long lastTimeFL;

// Create the timer instance
Timer speedTimer;


float speedFL = 0.0, speedRL = 0.0, speedFR = 0.0, speedRR = 0.0;
Average<float> aveFL(2), aveRL(2), aveFR(2), aveRR(2);
// control signal in [-255,255]
int cmdFL = 0, cmdRL = 0, cmdFR = 0, cmdRR = 0;

// PID Stuff
//double Kp=15, Ki=0.015, Kd=0;
double Kp=7, Ki=0.01, Kd=0;
double previousTimeFL = 0.0, previousTimeRL = 0.0, previousTimeFR = 0.0, previousTimeRR = 0.0;
double errorFL=0.0, cumErrorFL=0.0, lasterrorFL=0.0;
double errorRL=0.0, cumErrorRL=0.0, lasterrorRL=0.0;
double errorFR=0.0, cumErrorFR=0.0, lasterrorFR=0.0;
double errorRR=0.0, cumErrorRR=0.0, lasterrorRR=0.0;
// will come a number in max [-9,+9], min [-3,3]
double SetpointFL=0.0, SetpointRL=0.0, SetpointFR=0.0, SetpointRR=0.0;
double prevSetpointFL=0.0, prevSetpointRL=0.0, prevSetpointFR=0.0, prevSetpointRR=0.0;

bool startupFL = false, startupRL = false, startupFR = false, startupRR = false;

//ROS STUFF
ros::NodeHandle nh;
std_msgs::Float32MultiArray feedback_msg;

void motorsCommandCB( const std_msgs::Float32MultiArray& msg){

  SetpointFL = msg.data[0];
  SetpointRL = msg.data[1];
  SetpointFR = msg.data[2];
  SetpointRR = msg.data[3];
   
 }

ros::Publisher pub_feedback("motors_feedback", &feedback_msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub("motors_command", motorsCommandCB );
  
void updatespeedsCB() {

  // Left side negative when advancing
  aveFL.push((-counter_FL*2*PI*1000/151)/PERIOD);
  speedFL = aveFL.mean();
  aveRL.push((-counter_RL*2*PI*1000/151)/PERIOD);
  speedRL = aveRL.mean();
  
  // Right side positive when advancing
  aveFR.push((counter_FR*2*PI*1000/151)/PERIOD);
  speedFR = aveFR.mean();
  aveRR.push((counter_RR*2*PI*1000/151)/PERIOD);
  speedRR = aveRR.mean();

  /*Serial.print("FL SPEED: ");
  Serial.print(speedFL);
  Serial.print(" RL SPEED: ");
  Serial.print(speedRL);
  Serial.print(" FR SPEED: ");
  Serial.print(speedFR);
  Serial.print(" RR SPEED: ");
  Serial.println(speedRR);*/
  
  feedback_msg.data[0] = speedFL;
  feedback_msg.data[1] = speedRL;
  feedback_msg.data[2] = speedFR;
  feedback_msg.data[3] = speedRR;

  //publish feedback
  pub_feedback.publish(&feedback_msg);

  counter_FL = 0;
  counter_RL = 0;
  counter_FR = 0;
  counter_RR = 0;

  doControl = true;
   
 }

void stop_Stop()    //Stop
{
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
}
 
void setup() { 

  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT); 
  pinMode(RightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
  
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);

  //Send 0
  stop_Stop();
  
  pinMode (HALLA_FL,INPUT);
  pinMode (HALLB_FL,INPUT);
  
  pinMode (HALLA_RL,INPUT);
  pinMode (HALLB_RL,INPUT);
  
  pinMode (HALLA_FR,INPUT);
  pinMode (HALLB_FR,INPUT);
  
  pinMode (HALLA_RR,INPUT);
  pinMode (HALLB_RR,INPUT);
  
  // Reads the initial state of the HALLA
  aFLLastState = digitalRead(HALLA_FL);
  
  // Reads the initial state of the HALLA
  aRLLastState = digitalRead(HALLA_RL);   
  
  // Reads the initial state of the HALLA
  aFRLastState = digitalRead(HALLA_FR);   
  
  // Reads the initial state of the HALLA
  aRRLastState = digitalRead(HALLA_RR);
  
  // The timer will repeat every PERIOD ms
  speedTimer.setInterval(PERIOD); 
  
  // The function to be called
  speedTimer.setCallback(updatespeedsCB);
  
  // Start the timer
  speedTimer.start();
  
  //lastTimeFL = millis();
  
  //ROS STUFF
  nh.initNode();
  nh.advertise(pub_feedback);
  nh.subscribe(sub);

  feedback_msg.data = (float*)malloc(sizeof(float) * 4);
  feedback_msg.data_length = 4;
  
  // Wait until ROS connected
  while (!nh.connected())
  {
     nh.spinOnce();
     delay(1000);
  }
   
  nh.loginfo("ROS startup complete");
  //Serial.begin(9600);

 } 

 //Read Halls
 void read(){

   // Current state of HALL A
   aFLState = digitalRead(HALLA_FL);
   aRLState = digitalRead(HALLA_RL);
   aFRState = digitalRead(HALLA_FR);
   aRRState = digitalRead(HALLA_RR);
   
   //Serial.print("Reading A ");
   //Serial.println(aState);
   //Serial.print("Reading B ");
   //Serial.println(digitalRead(outputB));
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aFLState != aFLLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(HALLB_FL) != aFLState) { 
       counter_FL ++;
     } else {
       counter_FL --;
     }
     //Serial.print("FR Position: ");
     //Serial.println(counter_FL);
   }

   if (aRLState != aRLLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(HALLB_RL) != aRLState) { 
       counter_RL ++;
     } else {
       counter_RL --;
     }
     //Serial.print("RL Position: ");
     //Serial.println(counter_RL);
   } 

   if (aFRState != aFRLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(HALLB_FR) != aFRState) { 
       counter_FR ++;
     } else {
       counter_FR --;
     }
     //Serial.print("FR Position: ");
     //Serial.println(counter_FR);
   } 

   if (aRRState != aRRLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(HALLB_RR) != aRRState) { 
       counter_RR ++;
     } else {
       counter_RR --;
     }
     //Serial.print("RR Position: ");
     //Serial.println(counter_RR);
   }
   

   //Update the timer
   speedTimer.update();


   // Update previous States
   aFLLastState = aFLState;
   aRLLastState = aRLState;
   aFRLastState = aFRState;
   aRRLastState = aRRState;
 
 }

 //Write commands to motors
 //On minus sign is reverse, control is 0 -> full speed -255 -> cero
 void write(){

    double cmdFL_,cmdRL_,cmdFR_, cmdRR_;

    if(cmdFL > -1){

      digitalWrite(LeftMotorDirPin1,HIGH);
      digitalWrite(LeftMotorDirPin2,LOW);
      cmdFL_ = cmdFL;
      
    }
    else {

      digitalWrite(LeftMotorDirPin1,LOW);
      digitalWrite(LeftMotorDirPin2,HIGH);
      cmdFL_ = 255 - cmdFL;
      
    }
    if(cmdRL > -1){

      digitalWrite(LeftMotorDirPin1B,HIGH);
      digitalWrite(LeftMotorDirPin2B,LOW);
      cmdRL_ = cmdRL;
      
    }
    else {

      digitalWrite(LeftMotorDirPin1B,LOW);
      digitalWrite(LeftMotorDirPin2B,HIGH);
      cmdRL_ = 255 - cmdRL;
      
    }
    if(cmdFR > -1){

      digitalWrite(RightMotorDirPin1,HIGH);
      digitalWrite(RightMotorDirPin2,LOW);
      cmdFR_ = cmdFR;
      
    }
    else {

      digitalWrite(RightMotorDirPin1,LOW);
      digitalWrite(RightMotorDirPin2,HIGH); 
      cmdFR_ = 255-cmdFR;
      
    }
    if(cmdRR > -1){

      digitalWrite(RightMotorDirPin1B, HIGH);
      digitalWrite(RightMotorDirPin2B,LOW); 
      cmdRR_ = cmdRR;
      
    }
    else {

      digitalWrite(RightMotorDirPin1B, LOW);
      digitalWrite(RightMotorDirPin2B,HIGH); 
      cmdRR_ = 255-cmdRR;
      
    }

    analogWrite(speedPinL,cmdFL_);
    analogWrite(speedPinLB,cmdRL_);
    analogWrite(speedPinR,cmdFR_);
    analogWrite(speedPinRB,cmdRR_);
    
 
 }
 
 void loop() {

   //Read Halls Sensors
   read();

   //Write commands
   if (doControl)
   {
      computePIDFL();
      computePIDRL();
      computePIDFR();
      computePIDRR();

      //Write commands
      write();
      
      doControl = false;
   }
   
   nh.spinOnce();
  
 }

 void computePIDFL(){
       
        double currentTime = millis();                               // obtener el tiempo actual
        double elapsedTime = (double)(currentTime - previousTimeFL);     // calcular el tiempo transcurrido

        errorFL = SetpointFL - speedFL;                             // determinar el error entre la consigna y la medición

        if(SetpointFL >-0.9 && SetpointFL <0.9)
        {
          previousTimeFL = currentTime;
          cmdFL = 0;
          cumErrorFL = 0;
          lasterrorFL = 0;
          prevSetpointFL = SetpointFL;
          return;
          
        } else if(startupFL or ((prevSetpointFL >-0.9 && prevSetpointFL < 0.9) && (SetpointFL >-4.5 && SetpointFL <4.5)))
        {
          if(SetpointFL < 0)
            cmdFL = -110;
          else cmdFL = 110;
          
          previousTimeFL = currentTime;
          if(startupFL && fabs(speedFL) > 0.1)
            startupFL = false;
          else 
          {
            startupFL=true;
            return;
          }
        }
        
        if(fabs(errorFL) > 1.1)
        {
          cumErrorFL += errorFL * elapsedTime;                      // calcular la integral del error
          cmdFL = Kp*errorFL + Ki*cumErrorFL;
          lasterrorFL = errorFL;
          
        } else if(fabs(errorFL) > 0.26)
        {
          cumErrorFL += errorFL * elapsedTime;                      // corregir solo con integral
          cmdFL = Kp*lasterrorFL + Ki*cumErrorFL;
          
        } else
        {
          previousTimeFL = currentTime;                             // almacenar el tiempo anterior
          prevSetpointFL = SetpointFL;
          return;
        }
        
        //double rateError = (errorFL - lastErrorFL) / (elapsedTime/1000.0);         // calcular la derivada del error

        double frac = 0;

        //! Guaratee a minimun signal control for low speeds

        if(SetpointFL > -1.5 && SetpointFL < 1.5)
        {
          frac = fabs(SetpointFL)/1.5;
          if(SetpointFL < -0.05 && cmdFL > -40)
            cmdFL = -40 - 5*frac;
          else if(SetpointFL > 0.05 && cmdFL < 40)
            cmdFL = 40 + 5*frac;
          
        } else if(SetpointFL > -2.5 && SetpointFL < 2.5)
        {
          frac = (fabs(SetpointFL)-1.5);
          if(SetpointFL < 0 && cmdFL > -45)
            cmdFL = -45 - 5*frac;
          else if(SetpointFL > 0 && cmdFL < 45)
            cmdFL = 45 + 5*frac;
          
        } else if(SetpointFL > -3.5 && SetpointFL < 3.5)
        {
          frac = (fabs(SetpointFL)-2.5);
          if(SetpointFL < 0 && cmdFL > -48)
            cmdFL = -48 - 5*frac;
          else if( SetpointFL > 0 && cmdFL < 48)
            cmdFL = 48 + 5*frac;
          
        } else if(SetpointFL > -4.5 && SetpointFL < 4.5)
        {
          frac = (fabs(SetpointFL)-3.5);
          if(SetpointFL < 0 && cmdFL > -50)
            cmdFL = -50 - 5*frac;
          else if( SetpointFL > 0 && cmdFL < 50)
            cmdFL = 50 + 5*frac;
        } else if(SetpointFL > -5.5 && SetpointFL < 5.5)
        {
          frac = (fabs(SetpointFL)-4.5);
          if(SetpointFL < 0 && cmdFL > -55)
            cmdFL = -55 - 5*frac;
          else if( SetpointFL > 0 && cmdFL < 55)
            cmdFL = 55 + 5*frac;
        } else if(SetpointFL > -6.5 && SetpointFL < 6.5)
        {
          frac = (fabs(SetpointFL)-5.5);
          if(SetpointFL < 0 && cmdFL > -60)
            cmdFL = -60 - 5*frac;
          else if( SetpointFL > 0 && cmdFL < 60)
            cmdFL = 60 + 5*frac;
        } else if(SetpointFL > -7.5 && SetpointFL < 7.5)
        {
          frac = (fabs(SetpointFL)-6.5);
          if(SetpointFL < 0 && cmdFL > -65)
            cmdFL = -65 - 5*frac;
          else if( SetpointFL > 0 && cmdFL < 65)
            cmdFL = 65 + 5*frac;
        } else if(SetpointFL > -8.5 && SetpointFL < 8.5)
        {
          frac = (fabs(SetpointFL)-7.5);
          if(SetpointFL < 0 && cmdFL > -70)
            cmdFL = -70 - 5*frac;
          else if( SetpointFL > 0 && cmdFL < 70)
            cmdFL = 70 + 5*frac;
        } else if(SetpointFL > -9.5 && SetpointFL < 9.5)
        {
          frac = (fabs(SetpointFL)-8.5);
          if(SetpointFL < 0 && cmdFL > -75)
            cmdFL = -75 - 5*frac;
          else if( SetpointFL > 0 && cmdFL < 75)
            cmdFL = 75 + 5*frac;
        } else if(SetpointFL > -10.5 && SetpointFL < 10.5)
        {
          frac = (fabs(SetpointFL)-9.5);
          if(SetpointFL < 0 && cmdFL > -80)
            cmdFL = -80 - 5*frac;
          else if( SetpointFL > 0 && cmdFL < 80)
            cmdFL = 80 + 5*frac;
        }

        /*Serial.print("FL Speed: ");
        Serial.print(speedFL);
        Serial.print(" FL cumerror: ");
        Serial.print(cumErrorFL);
        Serial.print(" output: ");
        Serial.println(cmdFL);*/

        //clamp control signal
        if(cmdFL > 255)
        {
          cmdFL = 255;
          cumErrorFL = 0.0;
        }
        else if(cmdFL < -255)
        {
          cmdFL = -255;
          cumErrorFL = 0.0;
        }

        previousTimeFL = currentTime;                             // almacenar el tiempo anterior
        prevSetpointFL = SetpointFL;

}

 void computePIDRL(){
       
        double currentTime = millis();                               // obtener el tiempo actual
        double elapsedTime = (double)(currentTime - previousTimeRL);     // calcular el tiempo transcurrido

        errorRL = SetpointRL - speedRL;                             // determinar el error entre la consigna y la medición

        if(SetpointRL >-0.9 && SetpointRL <0.9)
        {
          previousTimeRL = currentTime;
          cmdRL = 0;
          cumErrorRL = 0;
          lasterrorRL = 0;
          prevSetpointRL = SetpointRL;
          return;
          
        } else if(startupRL or ((prevSetpointRL >-0.9 && prevSetpointRL < 0.9) && (SetpointRL >-4.5 && SetpointRL <4.5)))
        {
          if(SetpointRL < 0)
            cmdRL = -110;
          else cmdRL = 110;
          
          previousTimeRL = currentTime;
          if(startupRL && fabs(speedRL) > 0.1)
            startupRL = false;
          else 
          {
            startupRL=true;
            return;
          }
        }
        
        if(fabs(errorRL) > 1.1)
        {
          cumErrorRL += errorRL * elapsedTime;                      // calcular la integral del error
          cmdRL = Kp*errorRL + Ki*cumErrorRL;
          lasterrorRL = errorRL;
          
        } else if(fabs(errorRL) > 0.26)
        {
          cumErrorRL += errorRL * elapsedTime;                      // corregir solo con integral
          cmdRL = Kp*lasterrorRL + Ki*cumErrorRL;
          
        } else
        {
          previousTimeRL = currentTime;                             // almacenar el tiempo anterior
          prevSetpointRL = SetpointRL;
          return;
        }
        
        //double rateError = (errorRL - lastErrorRL) / (elapsedTime/1000.0);         // calcular la derivada del error

        double frac = 0;

        //! Guaratee a minimun signal control for low speeds

        if(SetpointRL > -1.5 && SetpointRL < 1.5)
        {
          frac = fabs(SetpointRL)/1.5;
          if(SetpointRL < -0.05 && cmdRL > -40)
            cmdRL = -40 - 5*frac;
          else if(SetpointRL > 0.05 && cmdRL < 40)
            cmdRL = 40 + 5*frac;
          
        } else if(SetpointRL > -2.5 && SetpointRL < 2.5)
        {
          frac = (fabs(SetpointRL)-1.5);
          if(SetpointRL < 0 && cmdRL > -45)
            cmdRL = -45 - 5*frac;
          else if(SetpointRL > 0 && cmdRL < 45)
            cmdRL = 45 + 5*frac;
          
        } else if(SetpointRL > -3.5 && SetpointRL < 3.5)
        {
          frac = (fabs(SetpointRL)-2.5);
          if(SetpointRL < 0 && cmdRL > -48)
            cmdRL = -48 - 5*frac;
          else if( SetpointRL > 0 && cmdRL < 48)
            cmdRL = 48 + 5*frac;
          
        } else if(SetpointRL > -4.5 && SetpointRL < 4.5)
        {
          frac = (fabs(SetpointRL)-3.5);
          if(SetpointRL < 0 && cmdRL > -50)
            cmdRL = -50 - 5*frac;
          else if( SetpointRL > 0 && cmdRL < 50)
            cmdRL = 50 + 5*frac;
        } else if(SetpointRL > -5.5 && SetpointRL < 5.5)
        {
          frac = (fabs(SetpointRL)-4.5);
          if(SetpointRL < 0 && cmdRL > -55)
            cmdRL = -55 - 5*frac;
          else if( SetpointRL > 0 && cmdRL < 55)
            cmdRL = 55 + 5*frac;
        } else if(SetpointRL > -6.5 && SetpointRL < 6.5)
        {
          frac = (fabs(SetpointRL)-5.5);
          if(SetpointRL < 0 && cmdRL > -60)
            cmdRL = -60 - 5*frac;
          else if( SetpointRL > 0 && cmdRL < 60)
            cmdRL = 60 + 5*frac;
        } else if(SetpointRL > -7.5 && SetpointRL < 7.5)
        {
          frac = (fabs(SetpointRL)-6.5);
          if(SetpointRL < 0 && cmdRL > -65)
            cmdRL = -65 - 5*frac;
          else if( SetpointRL > 0 && cmdRL < 65)
            cmdRL = 65 + 5*frac;
        } else if(SetpointRL > -8.5 && SetpointRL < 8.5)
        {
          frac = (fabs(SetpointRL)-7.5);
          if(SetpointRL < 0 && cmdRL > -70)
            cmdRL = -70 - 5*frac;
          else if( SetpointRL > 0 && cmdRL < 70)
            cmdRL = 70 + 5*frac;
        } else if(SetpointRL > -9.5 && SetpointRL < 9.5)
        {
          frac = (fabs(SetpointRL)-8.5);
          if(SetpointRL < 0 && cmdRL > -75)
            cmdRL = -75 - 5*frac;
          else if( SetpointRL > 0 && cmdRL < 75)
            cmdRL = 75 + 5*frac;
        } else if(SetpointRL > -10.5 && SetpointRL < 10.5)
        {
          frac = (fabs(SetpointRL)-9.5);
          if(SetpointRL < 0 && cmdRL > -80)
            cmdRL = -80 - 5*frac;
          else if( SetpointRL > 0 && cmdRL < 80)
            cmdRL = 80 + 5*frac;
        }

        //clamp control signal
        if(cmdRL > 255)
        {
          cmdRL = 255;
          cumErrorRL = 0.0;
        }
        else if(cmdRL < -255)
        {
          cmdRL = -255;
          cumErrorRL = 0.0;
        }

        previousTimeRL = currentTime;                             // almacenar el tiempo anterior
        prevSetpointRL = SetpointRL;
}

void computePIDFR(){
       
        double currentTime = millis();                               // obtener el tiempo actual
        double elapsedTime = (double)(currentTime - previousTimeFR);     // calcular el tiempo transcurrido

        errorFR = SetpointFR - speedFR;                             // determinar el error entre la consigna y la medición

        if(SetpointFR >-0.9 && SetpointFR <0.9)
        {
          previousTimeFR = currentTime;
          cmdFR = 0;
          cumErrorFR = 0;
          lasterrorFR = 0;
          prevSetpointFR = SetpointFR;
          return;
          
        }else if(startupFR or ((prevSetpointFR >-0.9 && prevSetpointFR < 0.9) && (SetpointFR >-4.5 && SetpointFR <4.5)))
        {
          if(SetpointFR < 0)
            cmdFR = -110;
          else cmdFR = 110;
          
          previousTimeFR = currentTime;
          if(startupFR && fabs(speedFR) > 0.1)
            startupFR = false;
          else 
          {
            startupFR=true;
            return;
          }
        }
        
        if(fabs(errorFR) > 1.1)
        {
          cumErrorFR += errorFR * elapsedTime;                      // calcular la integral del error
          cmdFR = Kp*errorFR + Ki*cumErrorFR;
          lasterrorFR = errorFR;
          
        } else if(fabs(errorFR) > 0.26)
        {
          cumErrorFR += errorFR * elapsedTime;                      // corregir solo con integral
          cmdFR = Kp*lasterrorFR + Ki*cumErrorFR;
          
        } else
        {
          previousTimeFR = currentTime;                             // almacenar el tiempo anterior
          prevSetpointFR = SetpointFR;
          return;
        }
        
        //double rateError = (errorFR - lastErrorFR) / (elapsedTime/1000.0);         // calcular la derivada del error

        double frac = 0;

        //! Guaratee a minimun signal control for low speeds

        if(SetpointFR > -1.5 && SetpointFR < 1.5)
        {
          frac = fabs(SetpointFR)/1.5;
          if(SetpointFR < -0.05 && cmdFR > -40)
            cmdFR = -40 - 5*frac;
          else if(SetpointFR > 0.05 && cmdFR < 40)
            cmdFR = 40 + 5*frac;
          
        } else if(SetpointFR > -2.5 && SetpointFR < 2.5)
        {
          frac = (fabs(SetpointFR)-1.5);
          if(SetpointFR < 0 && cmdFR > -45)
            cmdFR = -45 - 5*frac;
          else if(SetpointFR > 0 && cmdFR < 45)
            cmdFR = 45 + 5*frac;
          
        } else if(SetpointFR > -3.5 && SetpointFR < 3.5)
        {
          frac = (fabs(SetpointFR)-2.5);
          if(SetpointFR < 0 && cmdFR > -48)
            cmdFR = -48 - 5*frac;
          else if( SetpointFR > 0 && cmdFR < 48)
            cmdFR = 48 + 5*frac;
          
        } else if(SetpointFR > -4.5 && SetpointFR < 4.5)
        {
          frac = (fabs(SetpointFR)-3.5);
          if(SetpointFR < 0 && cmdFR > -50)
            cmdFR = -50 - 5*frac;
          else if( SetpointFR > 0 && cmdFR < 50)
            cmdFR = 50 + 5*frac;
        } else if(SetpointFR > -5.5 && SetpointFR < 5.5)
        {
          frac = (fabs(SetpointFR)-4.5);
          if(SetpointFR < 0 && cmdFR > -55)
            cmdFR = -55 - 5*frac;
          else if( SetpointFR > 0 && cmdFR < 55)
            cmdFR = 55 + 5*frac;
        } else if(SetpointFR > -6.5 && SetpointFR < 6.5)
        {
          frac = (fabs(SetpointFR)-5.5);
          if(SetpointFR < 0 && cmdFR > -60)
            cmdFR = -60 - 5*frac;
          else if( SetpointFR > 0 && cmdFR < 60)
            cmdFR = 60 + 5*frac;
        } else if(SetpointFR > -7.5 && SetpointFR < 7.5)
        {
          frac = (fabs(SetpointFR)-6.5);
          if(SetpointFR < 0 && cmdFR > -65)
            cmdFR = -65 - 5*frac;
          else if( SetpointFR > 0 && cmdFR < 65)
            cmdFR = 65 + 5*frac;
        } else if(SetpointFR > -8.5 && SetpointFR < 8.5)
        {
          frac = (fabs(SetpointFR)-7.5);
          if(SetpointFR < 0 && cmdFR > -70)
            cmdFR = -70 - 5*frac;
          else if( SetpointFR > 0 && cmdFR < 70)
            cmdFR = 70 + 5*frac;
        } else if(SetpointFR > -9.5 && SetpointFR < 9.5)
        {
          frac = (fabs(SetpointFR)-8.5);
          if(SetpointFR < 0 && cmdFR > -75)
            cmdFR = -75 - 5*frac;
          else if( SetpointFR > 0 && cmdFR < 75)
            cmdFR = 75 + 5*frac;
        } else if(SetpointFR > -10.5 && SetpointFR < 10.5)
        {
          frac = (fabs(SetpointFR)-9.5);
          if(SetpointFR < 0 && cmdFR > -80)
            cmdFR = -80 - 5*frac;
          else if( SetpointFR > 0 && cmdFR < 80)
            cmdFR = 80 + 5*frac;
        }

        
        //clamp control signal
        if(cmdFR > 255)
        {
          cmdFR = 255;
          cumErrorFR = 0.0;
        }
        else if(cmdFR < -255)
        {
          cmdFR = -255;
          cumErrorFR = 0.0;
        }

        previousTimeFR = currentTime;                             // almacenar el tiempo anterior
        prevSetpointFR = SetpointFR;
}


void computePIDRR(){
       
        double currentTime = millis();                               // obtener el tiempo actual
        double elapsedTime = (double)(currentTime - previousTimeRR);     // calcular el tiempo transcurrido

        errorRR = SetpointRR - speedRR;                             // determinar el error entre la consigna y la medición

        if(SetpointRR >-0.9 && SetpointRR <0.9)
        {
          previousTimeRR = currentTime;
          cmdRR = 0;
          cumErrorRR = 0;
          lasterrorRR = 0;
          prevSetpointRR = SetpointRR;
          return;
          
        }else if(startupRR or ((prevSetpointRR >-0.9 && prevSetpointRR < 0.9) && (SetpointRR >-4.5 && SetpointRR <4.5)))
        {
          if(SetpointRR < 0)
            cmdRR = -110;
          else cmdRR = 110;
          
          previousTimeRR = currentTime;
          if(startupRR && fabs(speedRR) > 0.1)
            startupRR = false;
          else 
          {
            startupRR=true;
            return;
          }
        }
        
        if(fabs(errorRR) > 1.1)
        {
          cumErrorRR += errorRR * elapsedTime;                      // calcular la integral del error
          cmdRR = Kp*errorRR + Ki*cumErrorRR;
          lasterrorRR = errorRR;
          
        } else if(fabs(errorRR) > 0.26)
        {
          cumErrorRR += errorRR * elapsedTime;                      // corregir solo con integral
          cmdRR = Kp*lasterrorRR + Ki*cumErrorRR;
          
        } else
        {
          previousTimeRR = currentTime;                             // almacenar el tiempo anterior
          prevSetpointRR = SetpointRR;
          return;
        }
        
        //double rateError = (errorRR - lastErrorRR) / (elapsedTime/1000.0);         // calcular la derivada del error

        double frac = 0;

        //! Guaratee a minimun signal control for low speeds

        if(SetpointRR > -1.5 && SetpointRR < 1.5)
        {
          frac = fabs(SetpointRR)/1.5;
          if(SetpointRR < -0.05 && cmdRR > -40)
            cmdRR = -40 - 5*frac;
          else if(SetpointRR > 0.05 && cmdRR < 40)
            cmdRR = 40 + 5*frac;
          
        } else if(SetpointRR > -2.5 && SetpointRR < 2.5)
        {
          frac = (fabs(SetpointRR)-1.5);
          if(SetpointRR < 0 && cmdRR > -45)
            cmdRR = -45 - 5*frac;
          else if(SetpointRR > 0 && cmdRR < 45)
            cmdRR = 45 + 5*frac;
          
        } else if(SetpointRR > -3.5 && SetpointRR < 3.5)
        {
          frac = (fabs(SetpointRR)-2.5);
          if(SetpointRR < 0 && cmdRR > -48)
            cmdRR = -48 - 5*frac;
          else if( SetpointRR > 0 && cmdRR < 48)
            cmdRR = 48 + 5*frac;
          
        } else if(SetpointRR > -4.5 && SetpointRR < 4.5)
        {
          frac = (fabs(SetpointRR)-3.5);
          if(SetpointRR < 0 && cmdRR > -50)
            cmdRR = -50 - 5*frac;
          else if( SetpointRR > 0 && cmdRR < 50)
            cmdRR = 50 + 5*frac;
        } else if(SetpointRR > -5.5 && SetpointRR < 5.5)
        {
          frac = (fabs(SetpointRR)-4.5);
          if(SetpointRR < 0 && cmdRR > -55)
            cmdRR = -55 - 5*frac;
          else if( SetpointRR > 0 && cmdRR < 55)
            cmdRR = 55 + 5*frac;
        } else if(SetpointRR > -6.5 && SetpointRR < 6.5)
        {
          frac = (fabs(SetpointRR)-5.5);
          if(SetpointRR < 0 && cmdRR > -60)
            cmdRR = -60 - 5*frac;
          else if( SetpointRR > 0 && cmdRR < 60)
            cmdRR = 60 + 5*frac;
        } else if(SetpointRR > -7.5 && SetpointRR < 7.5)
        {
          frac = (fabs(SetpointRR)-6.5);
          if(SetpointRR < 0 && cmdRR > -65)
            cmdRR = -65 - 5*frac;
          else if( SetpointRR > 0 && cmdRR < 65)
            cmdRR = 65 + 5*frac;
        } else if(SetpointRR > -8.5 && SetpointRR < 8.5)
        {
          frac = (fabs(SetpointRR)-7.5);
          if(SetpointRR < 0 && cmdRR > -70)
            cmdRR = -70 - 5*frac;
          else if( SetpointRR > 0 && cmdRR < 70)
            cmdRR = 70 + 5*frac;
        } else if(SetpointRR > -9.5 && SetpointRR < 9.5)
        {
          frac = (fabs(SetpointRR)-8.5);
          if(SetpointRR < 0 && cmdRR > -75)
            cmdRR = -75 - 5*frac;
          else if( SetpointRR > 0 && cmdRR < 75)
            cmdRR = 75 + 5*frac;
        } else if(SetpointRR > -10.5 && SetpointRR < 10.5)
        {
          frac = (fabs(SetpointRR)-9.5);
          if(SetpointRR < 0 && cmdRR > -80)
            cmdRR = -80 - 5*frac;
          else if( SetpointRR > 0 && cmdRR < 80)
            cmdRR = 80 + 5*frac;
        }

        //clamp control signal
        if(cmdRR > 255)
        {
          cmdRR = 255;
          cumErrorRR = 0.0;
        }
        else if(cmdRR < -255)
        {
          cmdRR = -255;
          cumErrorRR = 0.0;
        }

        previousTimeRR = currentTime;                             // almacenar el tiempo anterior
        prevSetpointRR = SetpointRR;
}
 
