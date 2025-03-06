#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
//Declaracion de motor
int In1 =19;
int In2 =21;
int Ena = 18;
int pinChannelB = 4;
int pinChannelA = 2;
int count = 0;

float timeValue=0;
float timeValue_1=0;
float w=0;


rcl_subscription_t subscriber;
std_msgs__msg__Float32 pwm;
std_msgs__msg__Float32 dato;
std_msgs__msg__Float32 vel_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_publisher_t publisher;


#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void IRAM_ATTR isr_Bf(){
  if(digitalRead(pinChannelA)==1){ 
    count++;
    
  }
  else{
    count--;
  }
  //count_msg.data=count;
  //RCSOFTCHECK(rcl_publish(&publisher, &count_msg, NULL));
}

void IRAM_ATTR isr_Af(){
  if(digitalRead(pinChannelB)==0){ 
    count++;
    
  }
  else{
    count--;
  }
  //count_msg.data=count;
  //RCSOFTCHECK(rcl_publish(&publisher, &count_msg, NULL));
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    timeValue=micros();
    if(timeValue-timeValue_1 > 0){
      w=count*(1e6/(timeValue-timeValue_1))/(30*22);
      vel_msg.data=w;
    }
    else{
      w=0;
      vel_msg.data=w;
    }
    count=0;
    timeValue_1=timeValue;
    RCSOFTCHECK(rcl_publish(&publisher, &vel_msg, NULL));   //Se puede modificar---------------------------------------
    //msg_pub.data++;                                         //Se puede modificar---------------------------------------

  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * dato = (const std_msgs__msg__Float32 *)msgin;
  //PWM de -1 a 1
  pwm.data = max(-1.0,min(double(dato->data),1.0));
  
  if (pwm.data > 0){
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);

  }
  else {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  }
  analogWrite(Ena, int(255*abs(pwm.data)));

}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(Ena, OUTPUT);

  Serial.begin(115200);
  
  delay(2000);

  attachInterrupt(digitalPinToInterrupt(pinChannelA), isr_Af, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinChannelB), isr_Bf, FALLING);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_input_u"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,                                         //Se puede modificar---------------------------------------
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),      //Se puede modificar tipo de mensaje---------------------------------------
    "motor_output_y"));                             //Se puede modificar---------------------------------------

  const unsigned int timer_timeout = 25;
  RCCHECK(rclc_timer_init_default(
    &timer,                         //Se puede modificar---------------------------------------
    &support,
    RCL_MS_TO_NS(timer_timeout),    //Se puede modificar---------------------------------------
    timer_callback));

  
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &dato, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
