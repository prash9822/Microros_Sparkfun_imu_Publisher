#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include "ICM_20948.h" 

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>



#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM;

#include <sensor_msgs/msg/imu.h>

sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{   
    
    imu_msg.linear_acceleration.x = float(sensor->accX());
    imu_msg.linear_acceleration.y = float(sensor->accY());
    imu_msg.linear_acceleration.z = float(sensor->accZ());

    imu_msg.angular_velocity.x = float(sensor->gyrX());
    imu_msg.angular_velocity.y = float(sensor->gyrY());
    imu_msg.angular_velocity.z = float(sensor->gyrZ());

    imu_msg.orientation.x = float(sensor->magX());
    imu_msg.orientation.y = float(sensor->magY());
    imu_msg.orientation.z = float(sensor->magZ());
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    printScaledAGMT(&myICM);     
    delay(30);
  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }

  if (timer != NULL) {

    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
   
  }
}

void setup(){

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  while (!Serial)
  {
  };

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(WIRE_PORT, AD0_VAL);


    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "imu_publisher_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_info_topic"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

}
void loop() {
  delay(100);
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}

