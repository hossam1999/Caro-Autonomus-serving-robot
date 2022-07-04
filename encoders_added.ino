#include <Wire.h>
#include <PID_v2.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <geometry_msgs/Twist.h>

#include <ros/time.h>

#define left_forward_rpwm 5
#define left_forward_lpwm 4
#define right_forward_rpwm 8
#define right_forward_lpwm 9


#define left_backward_rpwm 6
#define left_backward_lpwm 7
#define right_backward_rpwm 10
#define right_backward_lpwm 11

#define tpr  142
//initializing all the variables
#define LOOPTIME                      50     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

char log_msg[50];
char result[8];
double speed_cmd_left2 = 0;      

const int PIN_ENCOD_A_MOTOR_LEFT = 21;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 20;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 18;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 19;              //B channel for encoder of right motor 

const int PIN_SIDE_LIGHT_LED = 46;                  //Side light blinking led pin

unsigned long lastMilli = 0;
const double radius = 0.03;                   //Wheel radius, in m
const double wheelbase = 0.42;               //Wheelbase, in m

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double distance = 0;
double desiredDistance=100;

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 
                        
const double max_speed = 0.4;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 
                                                      
// PID Parameters
const double PID_left_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position
volatile float pos_temp = 0; 
PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

//Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create the motor shield object with the default I2C address
//Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);      //Create left motor object
//Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);     //Create right motor object
//  
ros::NodeHandle nh;
int Changedir=0;
//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

int flag=0;
/*void handle_dist(std_msgs::String msg){
 if(flag==0){
  desiredDistance = String(msg.data).toFloat() - 0.1;
  distance=0;
  flag=1;
 }
}*/

//ros::Subscriber<std_msgs::String> dist("ObjectDepth", handle_dist);

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
geometry_msgs::Vector3Stamped encoders;                                //create a "speed_msg" ROS message
//ros::Publisher encoders_pub("Encoder", &encoders);
const int lightIncNumber = 30;                                                                                                                                       //Number of lightIncrements for side light blinking
int lightInc = 0;                                                                                                                                                    //Init increment for side light blinking
int lightValue [lightIncNumber]= { 10, 40, 80, 160, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 160, 80, 40, 10, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
int lightValueNoComm [25]= { 255, 0, 255, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //side light increment values
int lightT = 0; //init light period

//____

void setup() {

  //Serial3.begin(19200);
  //Serial3.setTimeout(3);
  
  //pinMode(PIN_SIDE_LIGHT_LED, OUTPUT);      //set pin for side light leds as output
  //analogWrite(PIN_SIDE_LIGHT_LED, 255);     //light up side lights
  
  nh.initNode();                            //init ROS node
    nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel); 
   nh.advertise(speed_pub);  //suscribe to ROS topic for velocity commands
 // nh.subscribe(dist);
                 //prepare to publish speed in ROS topic
 
///  AFMS.begin();
  
  //setting motor speeds to zero
//  leftMotor->setSpeed(0);
//  leftMotor->run(BRAKE);
//  rightMotor->setSpeed(0);
//  rightMotor->run(BRAKE);
 
  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
    
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);
  delay(2000);
}

//___

void loop() {
  
    nh.spinOnce();
    if(1){
      // enter timed loop
    //lastMilli = millis();
    
    
//    if (!nh.connected()){
//      analogWrite(PIN_SIDE_LIGHT_LED, lightValueNoComm[lightInc]);
//      lightInc=lightInc+1;
//      if (lightInc >= 25){
//        lightInc=0;
//      }
//    }
//    else{
//      analogWrite(PIN_SIDE_LIGHT_LED, lightValue [lightInc]);
//      lightT = 3000 - ((2625/max_speed)*((abs(speed_req_left)+abs(speed_req_right))/2));
//      lightInc=lightInc+(30/(lightT/LOOPTIME));
//      if (lightInc >= lightIncNumber){
//        lightInc=0;
//      }
//    }
    
    

      speed_act_left=((pos_left/tpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
      distance = ((pos_temp/142)*2*PI)*radius;
      /*Serial3.println("Distance=");
      Serial3.println(distance); 
      Serial3.println("Desired distance=");
      Serial3.println(desiredDistance); 
      Serial3.println("Speed Req=");
      Serial3.println(speed_req); */
    

    
     
    speed_act_right=((pos_right/tpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    
    pos_temp=pos_temp+pos_left;
    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 // compute PWM value for left motor
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*0.0882)/0.00235) + (speed_cmd_left/0.00235), -255, 255); //
    
//    if (distance > desiredDistance) {                   //Stopping if too much time without command
//      analogWrite(left_forward_rpwm,0);
//      analogWrite(left_forward_lpwm,0);
//    }
//    else 
     if (speed_req_left == 0){                        //Stopping
     analogWrite(left_forward_rpwm,0);
     analogWrite(left_forward_lpwm,0);
     analogWrite(left_backward_rpwm,0);
     analogWrite(left_backward_lpwm,0);
     
    }
    else if (PWM_leftMotor > 0){
      analogWrite(left_forward_lpwm,0);
      analogWrite(left_forward_rpwm,abs(PWM_leftMotor));//Going forward
      
      analogWrite(left_backward_lpwm,0);
      analogWrite(left_backward_rpwm,abs(PWM_leftMotor));//Going forward

     
  
    }
    else {                                               //Going backward
     analogWrite(left_forward_lpwm,abs(PWM_leftMotor));
     analogWrite(left_forward_rpwm,0);
     analogWrite(left_backward_lpwm,abs(PWM_leftMotor));
     analogWrite(left_backward_rpwm,0);
    }
    
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 // compute PWM value for right motor
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*0.0882)/0.00235) + (speed_cmd_right/0.00235), -255, 255); // 

   /* if (distance > desiredDistance) {                   //Stopping if too much time without command
       analogWrite(right_forward,0);
      analogWrite(right_backward,0);
    }*/
      if (speed_req_right == 0){                        //Stopping
     analogWrite(right_forward_rpwm,0);
     analogWrite(right_forward_lpwm,0);
     analogWrite(right_backward_rpwm,0);
     analogWrite(right_backward_lpwm,0);
     
    }
    else if (PWM_rightMotor > 0){
      analogWrite(right_forward_lpwm,0);
      analogWrite(right_forward_rpwm,abs(PWM_rightMotor));//Going forward
      
      analogWrite(right_backward_lpwm,0);
      analogWrite(right_backward_rpwm,abs(PWM_rightMotor));//Going forward

     
  
    }
    else {                                               //Going backward
     analogWrite(right_forward_lpwm,abs(PWM_rightMotor));
     analogWrite(right_forward_rpwm,0);
     analogWrite(right_backward_lpwm,abs(PWM_rightMotor));
     analogWrite(right_backward_rpwm,0);
    }
//     if (speed_req_right == 0){                       //Stopping
//       analogWrite(right_forward,0);
//      analogWrite(right_backward,0);
//    }
//    else if (PWM_rightMotor > 0){                         //Going forward
//      analogWrite(right_forward,abs(PWM_rightMotor));
//     analogWrite(right_backward,0);
//    }
//    else {                                                //Going backward
//      analogWrite(right_forward,0);
//     analogWrite(right_backward,abs(PWM_rightMotor));
//    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    
    publishSpeed(LOOPTIME); 
    //Publish odometry on ROS topic
    //publishEncoders(LOOPTIME);
  
 }
 
else{
//
//      analogWrite(right_forward,0);
//      analogWrite(right_backward,0);
//
//      analogWrite(left_forward,0);
//      analogWrite(left_backward,0);


}

}
//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;   
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}



/*void publishEncoders(double time) {
  encoders.header.stamp = nh.now();      //timestamp for odometry data
  encoders.vector.x = speed_act_left;    //left wheel speed (in m/s)
  encoders.vector.y = speed_act_right;  //right wheel speed (in m/s)
  encoders.vector.z = distance;         //looptime, should be the same as specified in LOOPTIME (in s)
  encoders_pub.publish(&encoders);
  nh.spinOnce();
  nh.loginfo("Publishing Encoders");
}*/





//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;

}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  else pos_right++;

}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
