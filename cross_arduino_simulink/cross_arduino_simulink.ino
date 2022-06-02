#include <Average.h>

#include <ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <timerManager.h>
#include <timer.h>


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
* when u = 255 output 10 rad/s (9.57 / 10.82)
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

#define PERIOD 100

bool sent = false, doControl = false;

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
Average<float> aveFL(5);
int cmdFL = 0, cmdRL = 0, cmdFR = 0, cmdRR = 0;

//PID STUFF

// Constantes del controlador
double Kp=15, Ki=0.015, Kd=0;
 
// variables externas del controlador
double Input, Output, Setpoint = 4.0;
 
// variables internas del controlador
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;

//ROS STUFF
ros::NodeHandle nh;
std_msgs::Float32MultiArray feedback_msg;

void motorsCommandCB( const std_msgs::Int32MultiArray& msg){

  // will come a number in [-255,255]

  cmdFL = msg.data[0];
  cmdRL = msg.data[1];
  cmdFR = msg.data[2];
  cmdRR = msg.data[3];
   
 }

ros::Publisher pub_feedback("motors_feedback", &feedback_msg);
ros::Subscriber<std_msgs::Int32MultiArray> sub("motors_command", motorsCommandCB );
  
void updatespeedsCB() {

  // Left side negative when advancing
  aveFL.push((-counter_FL*2*PI*1000/151)/PERIOD);
  speedFL = aveFL.mean();
  speedRL = (-counter_RL*2*PI*1000/151)/PERIOD;
  
  // Right side positive when advancing
  speedFR = (counter_FR*2*PI*1000/151)/PERIOD;
  speedRR = (counter_RR*2*PI*1000/151)/PERIOD;

  /*Serial.print("FL SPEED: ");
  Serial.print(speedFL);
  Serial.print(" RL SPEED: ");
  Serial.print(speedRL);
  Serial.print(" FR SPEED: ");
  Serial.print(speedFR);
  Serial.print(" RR SPEED: ");
  Serial.println(speedRR);*/
  
  Serial.print(speedFL);
  Serial.print(",");
  /*Serial.print(speedRL);
  Serial.print(",");
  Serial.print(speedFR);
  Serial.print(",");
  Serial.println(speedRR);*/
  
  feedback_msg.data[0] = speedFL;
  feedback_msg.data[1] = speedRL;
  feedback_msg.data[2] = speedFR;
  feedback_msg.data[3] = speedRR;

  //publish feedback
  //pub_feedback.publish(&feedback_msg);

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
  
  // The timer will repeat every 400 ms
  speedTimer.setInterval(PERIOD); 
  
  // The function to be called
  speedTimer.setCallback(updatespeedsCB);
  
  // Start the timer
  speedTimer.start();
  
  //lastTimeFL = millis();
  
  //ROS STUFF
  //nh.initNode();
  //nh.advertise(pub_feedback);
  //nh.subscribe(sub);

  feedback_msg.data = (float*)malloc(sizeof(float) * 4);
  feedback_msg.data_length = 4;
  
  // Wait until connected
  /*while (!nh.connected())
  {
     nh.spinOnce();
    delay(500);
  }*/
   
  //nh.loginfo("ROS startup complete");
  Serial.begin(9600);

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

    if(cmdFL > -1){

      digitalWrite(LeftMotorDirPin1,HIGH);
      digitalWrite(LeftMotorDirPin2,LOW);
      analogWrite(speedPinL,cmdFL);
      
    }
    else {

      digitalWrite(LeftMotorDirPin1,LOW);
      digitalWrite(LeftMotorDirPin2,HIGH);
      analogWrite(speedPinL,255-cmdFL);
      
    }
    if(cmdRL > -1){

      digitalWrite(LeftMotorDirPin1B,HIGH);
      digitalWrite(LeftMotorDirPin2B,LOW);
      analogWrite(speedPinLB,cmdRL);
      
    }
    else {

      digitalWrite(LeftMotorDirPin1B,LOW);
      digitalWrite(LeftMotorDirPin2B,HIGH);
      analogWrite(speedPinLB,255-cmdRL);
      
    }
    if(cmdFR > -1){

      digitalWrite(RightMotorDirPin1,HIGH);
      digitalWrite(RightMotorDirPin2,LOW); 
      analogWrite(speedPinR,cmdFR);
      
    }
    else {

      digitalWrite(RightMotorDirPin1,LOW);
      digitalWrite(RightMotorDirPin2,HIGH); 
      analogWrite(speedPinR,255-cmdFR);
      
    }
    if(cmdRR > -1){

      digitalWrite(RightMotorDirPin1B, HIGH);
      digitalWrite(RightMotorDirPin2B,LOW); 
      analogWrite(speedPinRB,cmdRR);
      
    }
    else {

      digitalWrite(RightMotorDirPin1B, LOW);
      digitalWrite(RightMotorDirPin2B,HIGH); 
      analogWrite(speedPinRB,255-cmdRR);
      
    }
 
 }
 
 void loop() {

   //Read Halls Sensors
   read();

   //Write commands
   if (doControl)
   {
      cmdFL = computePID(speedFL);
      //Serial.print(cumError);
      //Serial.print(",");
      Serial.println(cmdFL);
      write();
      doControl = false;
   }

   if(millis()>15000 && !sent)
   {
     Setpoint = 7.0;
     sent = true;
   }

   //Serial.println(millis() - lastTimeFL);
   //lastTimeFL = millis();
   
   //nh.spinOnce();
  
 }

 double computePID(double inp){     
        currentTime = millis();                               // obtener el tiempo actual
        elapsedTime = (double)(currentTime - previousTime);     // calcular el tiempo transcurrido

        error = Setpoint - inp;                             // determinar el error entre la consigna y la medición
        
        if(fabs(error) > 0.02)
          cumError += error * elapsedTime;                      // calcular la integral del error
          
        //rateError = (error - lastError) / elapsedTime;         // calcular la derivada del error
 
        double output;     // calcular la salida del PID

        //if(Setpoint < 6.5 && Setpoint > -6.5)
        //  output = 12*error + 0.015*cumError;// + kd*rateError;
        //else output = Kp*error + Ki*cumError;// + kd*rateError;
        output = Kp*error + Ki*cumError;

        //clamp control signal
        if(output > 255)
        {
          output = 255;
          cumError -= error * elapsedTime;
        }
        else if(output < -255)
        {
          output = -255;
          cumError -= error * elapsedTime;
        }

        //lastError = error;                                      // almacenar error anterior
        previousTime = currentTime;                             // almacenar el tiempo anterior
 
        return output;
}
 
