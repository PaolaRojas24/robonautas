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
rcl_publisher_t motor_output_publisher;  //Declares a ROS 2 publisher for sending messages.

//Declare timers to be used
rcl_timer_t timer;          //Creates a timer to execute functions at intervals.

//Declare Messages to be used
std_msgs__msg__Float32 msg;  //Defines a message of type float32.
std_msgs__msg__Float32 pwm_msg;  //Defines a message of type int32.
std_msgs__msg__Float32 motor_speed_msg; //Defines a message for motor speed

uint64_t timer_timeout = 1000;  // Valor en milisegundos (1000 ms = 1 segundo)

//Define Macros to be used
//Executes fn and goes to error_loop() function if fn fails.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Specifies GPIO pin 13 for controlling an LED
#define ENA 27
#define In1 12
#define In2 14

#define PWM_FRQ 980 //Define PWM Frequency
#define PWM_RES 8  //Define PWM Resolution
#define PWM_CHNL 0    //Define Channel
#define EncA 25 //Encoder signal A pin
#define EncB 26 //Encoder signal B pin

//Variables to be used
float pwm_set_point = 0.0;
float motor_speed = 0.01;
float vel = 0.0;
int pwm_value = 0.0;
float velocidad = 0.0;  // Velocidad angular en rad/s
float velocidad_rad = 0.0;  // Declarar la variable para almacenar la velocidad angular

int32_t tiempo_act = 0, tiempo_ant = 0.0, delta_tiempo = 2e9;
float resolucion = 0.10233;  //Definir resolución del encoder
int pulsos = 3518;      //Número de pulsos a la salida del motorreductor
volatile int contador = 0; //Contador de pulsos
volatile bool BSet = 0;
volatile bool ASet = 0;
volatile bool encoderDirection = false;

// PID variables
float Kp = 0.575202065929138;           // Proportional Gain
float Ki = 1.592025076156;           // Integral Gain
float Kd = 1.28540119408164e-05;           // Derivative Gain
float set_point = 0.1;    // Desired set point
float prev_error = 0.0;   // Previous error for derivative calculation
float integral = 0.0;     // Integral term
float control_signal = 0.0;  // PID control signal
float posicion = 0.0;       // Almacena la posición calculada
int revoluciones = 0;       // Contador de revoluciones
float filt =0.0;
float velocidad_prev=0.0;
float p1 =0.0;
volatile int contador_ISR = 0;      // Almacena el valor de contador en la ISR
volatile uint32_t tiempo_act_ISR = 0; // Almacena el tiempo de la ISR
volatile bool direccion_ISR = false;  // Dirección del encoder
int contador_safe = 0;               // Copia segura del contador
uint32_t tiempo_safe = 0;            // Copia segura del tiempo
bool direccion_safe = false;         // Copia segura de la dirección


//Define Error Functions
void error_loop(){
  while(1){
    delay(100); // Wait 100 milliseconds
  }
}

void IRAM_ATTR Encoder() {
  BSet = digitalRead(EncB);
  ASet = digitalRead(EncA);

  if (BSet == ASet) {
    contador_ISR++;
    direccion_ISR = true;  // Giro en sentido horario
  } else {
    contador_ISR--;
    direccion_ISR = false; // Giro en sentido antihorario
  }

  tiempo_act_ISR = micros();  // Almacenar tiempo actual en la ISR
}


void PID(float setvalue){
  float error= setvalue- motor_speed;
  float derivative =(error - prev_error)/ 0.02;

  control_signal = Kp * error + Ki * integral + Kd * derivative;
  prev_error= error;
  
  control_signal = constrain(control_signal, -60, 60);
  pwm_value = map(abs(control_signal), 0, 60, 0, 255);

  if(control_signal >= 0 && control_signal <=255){
    digitalWrite(In1,LOW);
    digitalWrite(In2, HIGH);
    p1 = -1;
    //analogWrite(ENA, abs(control_signal));
  } else if (control_signal >= -255 && control_signal <=0){
    digitalWrite(In1,HIGH);
    digitalWrite(In2, LOW);
    p1 = +1;
    //analogWrite(ENA, abs(control_signal));
  }else {
    digitalWrite(In1,LOW);
    digitalWrite(In2, LOW);
  }

  ledcWrite(PWM_CHNL, pwm_value);


}
// Función para calcular posición y velocidad angular
void pose() {
  // Desactivar interrupciones temporalmente para copiar valores
  noInterrupts();
  contador_safe = contador_ISR;
  tiempo_safe = tiempo_act_ISR;
  direccion_safe = direccion_ISR;
  interrupts();  // Reactivar interrupciones

  // Calcular la posición
  if (direccion_safe) {
    posicion = contador_safe * resolucion;
    if (contador_safe >= pulsos) {
      revoluciones++;
      contador_ISR = 0;  // Reiniciar el contador
    }
  } else {
    posicion = contador_safe * resolucion;
    if (contador_safe <= -pulsos) {
      revoluciones--;
      contador_ISR = 0;  // Reiniciar el contador
    }
  }

  // Calcular velocidad angular (sin interferir con la ISR)
  delta_tiempo = tiempo_safe - tiempo_ant;
  tiempo_ant = tiempo_safe;

  if (delta_tiempo > 0) {
    velocidad = 60000000.0 / (pulsos * delta_tiempo);  // RPM
    //if (direccion_safe) velocidad = -velocidad;  // Ajustar signo
  }

  // Aplicar filtro para eliminar ruido
  if (abs(velocidad) < 1000) {
    velocidad_prev = velocidad;
    filt = 0.2 * velocidad + (1 - 0.2) * filt;
  }

  // Convertir velocidad a rad/s
  motor_speed = (2 * M_PI * contador_safe) / (pulsos * (delta_tiempo / 1e6));
  if (direccion_safe) motor_speed = -motor_speed;
  vel = motor_speed * (60 / (2 * M_PI));
  vel = constrain(vel, -60, 60);
  motor_speed_msg.data = vel;
  RCSOFTCHECK(rcl_publish(&motor_output_publisher, &motor_speed_msg, NULL));
}


//Define callbacks
void subscription_callback(const void * cmd_motor_argos)//el nombre del mensaje
{  
  //Get the message received and store it on the message msg
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)cmd_motor_argos;//se guarda xd
  set_point= msg->data;
  PID(set_point);
  
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  pose();
  if (timer != NULL) {
    //int pwm = int(setpoint_value * 255);
    msg.data = set_point;
    RCSOFTCHECK(rcl_publish(&pwm_publisher, &msg, NULL));
    
  }
}

//Setup
void setup() {
  set_microros_transports(); // Initializes communication between ESP32 and the ROS 2 agent (Serial).
  //Setup Microcontroller Pins
  pinMode(ENA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);

  //Setup PWM
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);  //Setup the PWM
  ledcAttachPin(ENA, PWM_CHNL);       //Setup Attach the Pin to the Channel   
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(EncB), Encoder, RISING);
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
    "set_point_argos"));

    // Create PWM publisher
  RCCHECK(rclc_publisher_init_default(
    &pwm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_argos"));

        // Create PWM publisher
  RCCHECK(rclc_publisher_init_default(
    &motor_output_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output_argos"));
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));//solo se usara el subcriber
  // Register suscription with executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}//cada que tiene un dato nuevo  ejectua el callback function

void loop() {
  //Executor Spin
  
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  //Executor Spin
  
}