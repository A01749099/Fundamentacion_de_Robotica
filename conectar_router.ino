#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>               // Wi-Fi library for ESP32
#include <WiFiUdp.h>            // UDP communication over Wi-Fi
#include <std_msgs/msg/float32.h>

#define BEST_EFFORT         //Select between BEST_EFFORT or RELIABLE
#define WIFI           //Select between SERIAL and WIFI Communication

// Pines motores
int In1 = 19, In2 = 21, Ena = 18, pinChannelB = 4, pinChannelA = 2; //Derecha
int In3 = 5, In4 = 17, En2 = 16, pinChannelB2 = 22, pinChannelA2 = 23; //Izquierda

volatile int count_derecha = 0, count_izquierda = 0;
float timeValue = 0, timeValue_1 = 0;

// Variables PID por motor

float Kp_d = 0.9030, Ki_d = 0.915, Kd_d = 0.0001665;//Izquierda
float Kp_i = 0.8030, Ki_i = 0.915, Kd_i = 0.0001665;//Derecho
float Ts = 0.002; 

// Estado PID motor derecha
float Ie_d = 0, e_1_d = 0, De_d = 0;
// Estado PID motor izquierda
float Ie_i = 0, e_1_i = 0, De_i = 0;


#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

#ifdef WIFI
const char* ssid     = "V2D2";
const char* password = "12345678";
micro_ros_agent_locator locator;

// Micro-ROS Agent configuration (host machine)
const char* agent_ip = "10.42.0.1";
const int agent_port = 9999;
#endif

// Enum representing different connection states of the microcontroller
enum states {
  WAITING_AGENT,       // Waiting for a connection to the Micro-ROS agent
  AGENT_AVAILABLE,     // Agent found, trying to establish communication
  AGENT_CONNECTED,     // Connected to the agent, publishing messages
  AGENT_DISCONNECTED   // Lost connection, trying to reconnect
} state;

// ROS2
rcl_subscription_t sub_setpoint_d, sub_feedback_d, sub_setpoint_i, sub_feedback_i;
rcl_publisher_t pub_control_d, pub_feedback_d, pub_control_i, pub_feedback_i;
std_msgs__msg__Float32 control_msg_d, control_msg_i, vel_msg_d, vel_msg_i, setpoint_msg_d, setpoint_msg_i;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void IRAM_ATTR isr_Bf_d() { count_derecha += (digitalRead(pinChannelA) == 0) ? 1 : -1; }
void IRAM_ATTR isr_Af_d() { count_derecha += (digitalRead(pinChannelB) == 1) ? 1 : -1; }
void IRAM_ATTR isr_Bf_i() { count_izquierda += (digitalRead(pinChannelA2) == 1) ? 1 : -1; }
void IRAM_ATTR isr_Af_i() { count_izquierda += (digitalRead(pinChannelB2) == 0) ? 1 : -1; }

// Cálculo de velocidades y publicación
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer != NULL) {
    timeValue = micros();
    float w_d = count_derecha * (1e6 / (timeValue - timeValue_1)) / (30 * 22);
    float w_i = count_izquierda * (1e6 / (timeValue - timeValue_1)) / (30 * 22);
    vel_msg_d.data = w_d;
    vel_msg_i.data = w_i;
    //Serial.print("Velocidad Izquierda: ");
    //Serial.println(vel_msg_i.data);

    count_derecha = count_izquierda = 0;
    timeValue_1 = timeValue;
    rcl_publish(&pub_feedback_d, &vel_msg_d, NULL);
    rcl_publish(&pub_feedback_i, &vel_msg_i, NULL);
  }
}

void subscription_setpoint_callback_d(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  setpoint_msg_d.data = msg->data;
  
}
void subscription_setpoint_callback_i(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  setpoint_msg_i.data = msg->data;
  //Serial.print("Setpoint Izquierdo: ");
  //Serial.println(setpoint_msg_i.data);

}
// PID Motor derecha
void subscription_feedback_callback_d(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float e = setpoint_msg_d.data - msg->data;
  Ie_d += (e + e_1_d) * Ki_d * Ts / 2;
  De_d = (e - e_1_d) / Ts;
  float u2 = constrain(Kp_d * e + Ki_d * Ie_d + Kd_d * De_d, -1.0, 1.0);
  control_msg_d.data = u2;
  rcl_publish(&pub_control_d, &control_msg_d, NULL);
  digitalWrite(In2, u2 > 0 ? LOW : HIGH);
  digitalWrite(In1, u2 > 0 ? HIGH : LOW);
  analogWrite(Ena, int(255 * abs(u2)));
  e_1_d = e;
}

// PID Motor izquierda
void subscription_feedback_callback_i(const void *msgin) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  float e = setpoint_msg_i.data - msg->data;
  Ie_i += (e + e_1_i) * Ki_i * Ts / 2;
  De_i = (e - e_1_i) / Ts;
  float u = constrain(Kp_i * e + Ki_i * Ie_i + Kd_i * De_i, -1.0, 1.0);
  control_msg_i.data = u;
  rcl_publish(&pub_control_i, &control_msg_i, NULL);
  digitalWrite(In3, u > 0 ? LOW : HIGH);
  digitalWrite(In4, u > 0 ? HIGH : LOW);
  analogWrite(En2, int(255 * abs(u)));
  e_1_i = e;
}

bool create_entities()
{
  
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "motor_controller", "", &support);

  //rclc_subscription_init_default(&subscriber_setpoint, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"set_point");

  rclc_subscription_init_default(&sub_setpoint_d, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "set_point_d");
  rclc_subscription_init_default(&sub_feedback_d, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_output_y_d");
  rclc_publisher_init_default(&pub_control_d, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_input_u_d");
  rclc_publisher_init_default(&pub_feedback_d, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_output_y_d");

  rclc_subscription_init_default(&sub_setpoint_i, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "set_point_i");
  rclc_subscription_init_default(&sub_feedback_i, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_output_y_i");
  rclc_publisher_init_default(&pub_control_i, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_input_u_i");
  rclc_publisher_init_default(&pub_feedback_i, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_output_y_i");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2), timer_callback);
  rclc_executor_init(&executor, &support.context, 6, &allocator);

  rclc_executor_add_subscription(&executor, &sub_setpoint_d, &setpoint_msg_d, &subscription_setpoint_callback_d, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_setpoint_i, &setpoint_msg_i, &subscription_setpoint_callback_i, ON_NEW_DATA);

  rclc_executor_add_subscription(&executor, &sub_feedback_d, &vel_msg_d, &subscription_feedback_callback_d, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_feedback_i, &vel_msg_i, &subscription_feedback_callback_i, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;  // Return true if all entities are successfully created
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub_control_i, &node);
  rcl_publisher_fini(&pub_feedback_i, &node);
  rcl_publisher_fini(&pub_control_d, &node);
  rcl_publisher_fini(&pub_feedback_d, &node);

  rcl_subscription_fini(&sub_setpoint_i, &node);
  rcl_subscription_fini(&sub_setpoint_d, &node);
  rcl_subscription_fini(&sub_feedback_i, &node);
  rcl_subscription_fini(&sub_feedback_d, &node);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}



void setup() {
  allocator = rcl_get_default_allocator();
  #ifdef WIFI
  Serial.begin(115200);
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //delay(1000);
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  
  locator.address.fromString(agent_ip);
  locator.port = agent_port;
  //WiFi.softAPConfig(local_ip, gateway, subnet);

  rmw_uros_set_custom_transport(
    false,
    (void *) &locator,
    arduino_wifi_transport_open,
    arduino_wifi_transport_close,
    arduino_wifi_transport_write,
    arduino_wifi_transport_read
  );

  #else

  set_microros_transports();

  #endif

  // Set initial state to waiting for ROS 2 Agent
  state = WAITING_AGENT;

  
  pinMode(In1, OUTPUT); pinMode(In2, OUTPUT); pinMode(Ena, OUTPUT);
  pinMode(In3, OUTPUT); pinMode(In4, OUTPUT); pinMode(En2, OUTPUT);
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(pinChannelA), isr_Af_d, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinChannelB), isr_Bf_d, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinChannelA2), isr_Af_i, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinChannelB2), isr_Bf_i, FALLING);

  
}

void loop() {
  switch (state) {

    case WAITING_AGENT:
      // Try to ping the Micro-ROS agent every second
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(1000, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      // Try to create ROS entities, move to connected state if successful
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      // Check connection every second, if lost move to disconnected state
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(1000, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      }
      break;

    case AGENT_DISCONNECTED:
      // Destroy entities and try reconnecting
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }

}
