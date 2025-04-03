// Incluir bibliotecas necesarias
#include <micro_ros_arduino.h>    // Biblioteca Micro-ROS para Arduino
#include <rcl/rcl.h>              // Biblioteca principal de ROS 2 para gestión de nodos
#include <rcl/error_handling.h>   // Funciones de manejo de errores para Micro-ROS
#include <rclc/rclc.h>            // Biblioteca Micro-ROS para dispositivos embebidos
#include <rclc/executor.h>        // Ejecutador de Micro-ROS para gestionar callbacks
#include <std_msgs/msg/float32.h>   // Tipo de mensaje estándar Float32 en ROS 2
#include <stdio.h>                // Biblioteca estándar de I/O para depuración
#include <math.h>                 // Funciones matemáticas estándar

// Declaración de nodos
rcl_node_t node;            // Nodo ROS 2 ejecutándose en el microcontrolador

// Declaración de ejecutador y clases de soporte
rclc_executor_t executor;   // Ejecuta tareas (timers, callbacks, etc.)
rclc_support_t support;     // Estructura de datos que mantiene el estado de la ejecución de Micro-ROS
rcl_allocator_t allocator;  // Gestión de memoria

// Declaración de suscriptores
rcl_subscription_t subscriber;  // Suscriptor para recibir mensajes

// Declaración de publicadores
rcl_publisher_t pwm_publisher;  // Publicador de mensajes para PWM
rcl_publisher_t motor_output_publisher;  // Publicador de mensajes para la salida del motor

// Declaración de temporizadores
rcl_timer_t timer;          // Temporizador para ejecutar funciones a intervalos

// Declaración de mensajes
std_msgs__msg__Float32 msg;  // Mensaje de tipo Float32
std_msgs__msg__Float32 pwm_msg;  // Mensaje de tipo Float32 para PWM
std_msgs__msg__Float32 motor_speed_msg; // Mensaje de tipo Float32 para velocidad del motor

uint64_t timer_timeout = 1000;  // Tiempo del temporizador en milisegundos (1 segundo)

// Definición de macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Definición de pines y otros valores de hardware
#define ENA 27
#define In1 12
#define In2 14
#define PWM_FRQ 980  // Frecuencia PWM
#define PWM_RES 8    // Resolución PWM
#define PWM_CHNL 0   // Canal PWM
#define EncA 25      // Pin de señal A del encoder
#define EncB 26      // Pin de señal B del encoder

// Variables globales
float pwm_set_point = 0.0;
float motor_speed = 0.01;
float vel = 0.0;
int pwm_value = 0;
float velocidad = 0.0;
float velocidad_rad = 0.0;  // Velocidad angular en rad/s

// Variables de control PID
float Kp = 0.575202065929138;    // Ganancia proporcional
float Ki = 1.592025076156;      // Ganancia integral
float Kd = 1.28540119408164e-05; // Ganancia derivativa
float set_point = 0.1;          // Punto de referencia deseado
float prev_error = 0.0;         // Error previo para cálculo derivativo
float integral = 0.0;           // Término integral
float control_signal = 0.0;     // Señal de control PID
float posicion = 0.0;           // Posición calculada
int revoluciones = 0;           // Contador de revoluciones

volatile int contador = 0;      // Contador de pulsos
volatile bool BSet = 0;
volatile bool ASet = 0;
volatile bool encoderDirection = false;

// Funciones de error
void error_loop(){
  while(1){
    delay(100); // Espera en caso de error
  }
}

// Interrupción del encoder
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

// Función PID para control del motor
void PID(float setvalue){
  float error = setvalue - motor_speed;
  float derivative = (error - prev_error) / 0.02;

  control_signal = Kp * error + Ki * integral + Kd * derivative;
  prev_error = error;
  
  control_signal = constrain(control_signal, -60, 60);
  pwm_value = map(abs(control_signal), 0, 60, 0, 255);

  if (control_signal >= 0 && control_signal <= 255) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  } else if (control_signal >= -255 && control_signal <= 0) {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  } else {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
  }

  ledcWrite(PWM_CHNL, pwm_value);
}

// Función para calcular la posición y velocidad angular
void pose() {
  noInterrupts();  // Desactivar interrupciones temporalmente
  contador_safe = contador_ISR;
  tiempo_safe = tiempo_act_ISR;
  direccion_safe = direccion_ISR;
  interrupts();    // Reactivar interrupciones

  // Calcular la posición y la velocidad angular
  if (direccion_safe) {
    posicion = contador_safe * resolucion;
    if (contador_safe >= pulsos) {
      revoluciones++;
      contador_ISR = 0;  // Reiniciar contador
    }
  } else {
    posicion = contador_safe * resolucion;
    if (contador_safe <= -pulsos) {
      revoluciones--;
      contador_ISR = 0;  // Reiniciar contador
    }
  }

  delta_tiempo = tiempo_safe - tiempo_ant;
  tiempo_ant = tiempo_safe;

  if (delta_tiempo > 0) {
    velocidad = 60000000.0 / (pulsos * delta_tiempo);  // RPM
  }

  if (abs(velocidad) < 1000) {
    velocidad_prev = velocidad;
    filt = 0.2 * velocidad + (1 - 0.2) * filt;
  }

  motor_speed = (2 * M_PI * contador_safe) / (pulsos * (delta_tiempo / 1e6));
  if (direccion_safe) motor_speed = -motor_speed;
  vel = motor_speed * (60 / (2 * M_PI));
  vel = constrain(vel, -60, 60);
  motor_speed_msg.data = vel;
  RCSOFTCHECK(rcl_publish(&motor_output_publisher, &motor_speed_msg, NULL));
}

// Callback para suscripción
void subscription_callback(const void * cmd_motor_argos) {  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)cmd_motor_argos;
  set_point = msg->data;
  PID(set_point);
}

// Callback para temporizador
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  pose();
  if (timer != NULL) {
    msg.data = set_point;
    RCSOFTCHECK(rcl_publish(&pwm_publisher, &msg, NULL));
  }
}

// Configuración inicial
void setup() {
  set_microros_transports(); // Inicializa la comunicación entre el ESP32 y el agente ROS 2 (por Serial)
  
  // Configuración de pines
  pinMode(ENA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(EncA, INPUT_PULLUP);
  pinMode(EncB, INPUT_PULLUP);

  // Configuración de PWM
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CHNL);
  
  // Configuración de interrupciones del encoder
  attachInterrupt(digitalPinToInterrupt(EncA), Encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(EncB), Encoder, RISING);

  delay(2000); // Espera para asegurar la conexión
  
  // Inicializa la memoria para Micro-ROS
  allocator = rcl_get_default_allocator();

  // Crea la estructura de soporte de ROS 2
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crea el nodo de ROS 2
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  // Crea el suscriptor
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "set_point_argos"));

  // Crea el publicador de PWM
  RCCHECK(rclc_publisher_init_default(
    &pwm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_argos"));

  // Crea el publicador de salida del motor
  RCCHECK(rclc_publisher_init_default(
    &motor_output_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output_argos"));

  // Crea el temporizador
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Crea el ejecutador
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  // Registra la suscripción con el ejecutador
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

// Loop principal
void loop() {
  delay(100);  // Espera de 100 ms
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  // Ejecuta el ciclo del ejecutador
}
