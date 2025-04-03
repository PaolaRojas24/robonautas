// Include Libraries to be used
#include <micro_ros_arduino.h>    //micro-ros-arduino library
#include <rcl/rcl.h>              //Core ROS 2 Client Library (RCL) for node management.
#include <rcl/error_handling.h>   //Error handling utilities for Micro-ROS.
#include <rclc/rclc.h>            //Micro-ROS Client library for embedded devices.
#include <rclc/executor.h>        //Micro-ROS Executor to manage callbacks
#include <std_msgs/msg/float32.h>   //Predefined ROS 2 message type
#include <stdio.h>                //Standard I/O library for debugging.
#include <math.h>


//Declare nodes to be used
rcl_node_t node;            //Represents a ROS 2 Node running on the microcontroller.

//Instantiate executor and its support classes
rclc_executor_t executor;   //Manages task execution (timers, callbacks, etc.).
rclc_support_t support;     //Data structure that holds the execution context of Micro-ROS, including its communication state, memory management, and initialization data.
rcl_allocator_t allocator;  //Manages memory allocation.

//Declare Subscribers to be used
rcl_subscription_t subscriber;

//Declare Publishers to be used
rcl_publisher_t pwm_publisher;  //Declares a ROS 2 publisher for sending messages.

//Declare timers to be used
rcl_timer_t timer;          //Creates a timer to execute functions at intervals.

//Declare Messages to be used
std_msgs__msg__Float32 msg;  //Defines a message of type float32.
std_msgs__msg__Float32 pwm_msg;  //Defines a message of type int32.

//Define Macros to be used
//Executes fn and goes to error_loop() function if fn fails.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Specifies GPIO pin 13 for controlling an LED
#define ENA 26
#define In1 14
#define In2 27

#define PWM_FRQ 980 //Define PWM Frequency
#define PWM_RES 8  //Define PWM Resolution
#define PWM_CHNL 0    //Define Channel
#define MSG_MIN_VAL -1 //Define min input value
#define MSG_MAX_VAL 1 //Define max input value

//Variables to be used
float pwm_set_point = 0.0;

//Define Error Functions
void error_loop(){
  while(1){
    delay(100); // Wait 100 milliseconds
  }
}

//Define callbacks
void subscription_callback(const void * cmd_motor_argos)//el nombre del mensaje
{  
  //Get the message received and store it on the message msg
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)cmd_motor_argos;//se guarda xd
  pwm_set_point = constrain(msg->data, MSG_MIN_VAL, MSG_MAX_VAL);//saca los valores, entre los valores minimos y maximos
  
  if (pwm_set_point >0){
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  }else if (pwm_set_point < 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  }else {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
  }
  //ledcWrite(PWM_CHNL, (uint32_t) (pow(2, PWM_RES) * (pwm_set_point  / 1.0)));
  int pwm_value = (int)(fabs(pwm_set_point) * 255);  // El valor de PWM se mapea entre 0 y 255
  ledcWrite(PWM_CHNL, pwm_value);
  //Publica el valor de PWM 
  pwm_msg.data = pwm_set_point;
  RCSOFTCHECK(rcl_publish(&pwm_publisher, &pwm_msg, NULL));
  
}


//Setup
void setup() {
  set_microros_transports(); // Initializes communication between ESP32 and the ROS 2 agent (Serial).
  //Setup Microcontroller Pins
  pinMode(ENA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);  //Setup the PWM
  ledcAttachPin(ENA, PWM_CHNL);       //Setup Attach the Pin to the Channel   
  //Connection delay
  delay(2000);
  //Initializes memory allocation for Micro-ROS operations.
  allocator = rcl_get_default_allocator();

  //Creates a ROS 2 support structure to manage the execution context.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_pwm_argos"));

    // Create PWM publisher
  RCCHECK(rclc_publisher_init_default(
    &pwm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_argos"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));//solo se usara el subcriber
  // Register suscription with executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}//cada que tiene un dato nuevo  ejectua el callback function

void loop() {
  //Executor Spin
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  //Executor Spin
}