#include "Kinematics.h"
#include "Pid_control.h"
#include "Motor_control.h"
extern odom_t odom;
// 引入microros和wifi相关的库函数------------------------------------------------------------------------
#include "WiFi.h"
#include "rcl/rcl.h"
#include "micro_ros_platformio.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include <geometry_msgs/msg/twist.h> //运动指令的消息接口
#include "nav_msgs/msg/odometry.h"//里程计的消息接口
#include "micro_ros_utilities/string_utilities.h"//引入字符串内存分配初始化工具

// 声明一些结构体对象
rcl_allocator_t allocator;             // 用于动态内存分配
rclc_support_t support;                // 用于存储时钟
rclc_executor_t executor;              // 执行器，用于管理订阅和计时器的回调执行
rcl_node_t node;                       // 创建单片机上的节点
rcl_subscription_t sub_cmd_vel;        // 创建一个订阅者
geometry_msgs__msg__Twist msg_cmd_vel; // 创建一个消息存放数据
//里程计
rcl_publisher_t pub_odom;              // 创建一个里程计发布者
nav_msgs__msg__Odometry msg_odom;       //存储里程计消息
rcl_timer_t timer1;                  //创建一个定时器

float out_MotorL = 0, out_MotorR = 0;
float target_linear = 0;
float target_angular = 0;


// 定时器回调函数
void timer_callback(rcl_timer_t *timer1, int64_t last_call_time)
{ //完成里程计的发布
    odom_update();
    int64_t stamp =rmw_uros_epoch_millis();
    msg_odom.header.stamp.sec = static_cast<int32_t>(stamp / 1000);
    msg_odom.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000 )*1e6;
    msg_odom.pose.pose.position.x = odom.x;
    msg_odom.pose.pose.position.y = odom.y;
    msg_odom.pose.pose.orientation.w = cos(odom.angle/2);
    msg_odom.pose.pose.orientation.x =0; 
    msg_odom.pose.pose.orientation.y =0;
    msg_odom.pose.pose.orientation.z=sin(odom.angle/2);
    msg_odom.twist.twist.linear.x = (odom.linear_speed/1000);
    msg_odom.twist.twist.angular.z = odom.angular_speed;

    if(rcl_publish(&pub_odom, &msg_odom, NULL)!=RCL_RET_OK)
    {
        Serial.println("publish failed");
    }
    
}



// 话题收到数据的回调函数
void twist_callback(const void *msg_in)
{
    Serial.println("=== 收到 /cmd_vel 消息 ===");

    // 将收到的消息指针转换为指向geometry_msgs__msg__Twist的指针
    const geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist *)msg_in;
    // 将话题中的角速度赋值给目标值
    target_angular = msg->angular.z;
    // 将话题中的线速度赋值给目标值
    target_linear = msg->linear.x * 1000;
    // 运动学逆解
    Kinematics_down(target_linear, target_angular, &out_MotorL, &out_MotorR);
   
    if (fabs(target_linear) < 0.1 && fabs(target_angular) < 0.01)
    { // 收到停止指令
        for (int i = 0; i < 4; i++)
        {   
            run[i].Pid_setgoal(i + 1, 0);
        }
         
    }
    else
    {
        run[0].Pid_setgoal(1, out_MotorL); // 加载给电机解算后的速度
        run[1].Pid_setgoal(2, out_MotorL);
        run[2].Pid_setgoal(3, out_MotorR);
        run[3].Pid_setgoal(4, out_MotorR);
    }

    
}

void micro_ros_task(void *arg)
{
    // 1. 连接 WiFi
    Serial.println("Connecting to WiFi...");
    WiFi.begin("doomsday", "123123123");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        if (millis() > 15000)
        {
            Serial.println("\nWiFi timeout!");
            vTaskDelete(NULL);
        }
    }
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());

    // 2. 设置 transport
    IPAddress agent_ip;
    uint16_t agent_port = 8888;
    agent_ip.fromString("10.19.168.51"); //-------------------------------------------------------------------------------------------------上位机地址
    set_microros_wifi_transports("doomsday", "123123123", agent_ip, agent_port);
    delay(2000); // ⚠️ 关键：给 transport 充分时间初始化

    // 3. 初始化 micro-ROS（带错误检查！）
    allocator = rcl_get_default_allocator();

    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK)
    {
        Serial.println("rclc_support_init failed!");
        vTaskDelete(NULL);
    }

    ret = rclc_node_init_default(&node, "fish_node", "", &support);
    if (ret != RCL_RET_OK)
    {
        Serial.println("rclc_node_init_default failed!");
        vTaskDelete(NULL);
    }

    ret = rclc_subscription_init_best_effort(
        &sub_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");
    if (ret != RCL_RET_OK)
    {
        Serial.println("subscription init failed!");
        vTaskDelete(NULL);
    }

    ret = rclc_executor_init(&executor, &support.context, 2, &allocator);//-----------------------------------------------------------------------------
    if (ret != RCL_RET_OK)
    {
        Serial.println("executor init failed!");
        vTaskDelete(NULL);
    }

    ret = rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &twist_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK)
    {
        Serial.println("add subscription failed!");
        vTaskDelete(NULL);
    }
    //初始化msg
    msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id,"odom");
    msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id,"base_footprint");
    //初始化odom发布着和定时器
    rclc_publisher_init_best_effort(&pub_odom, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),"/odom");
   
    
    //每间隔5ms调用一次回调函数
    rclc_timer_init_default(&timer1, &support, RCL_MS_TO_NS(50), timer_callback);
    rclc_executor_add_timer(&executor, &timer1);
    //同步时间
    while (!rmw_uros_epoch_synchronized() )
    {
        rmw_uros_sync_session(1000);
        delay(10);
    }
    
    Serial.println("✅ micro-ROS node 'node' ready!");
    // 4. 主循环
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10)); // 稍微延长让出时间
    }

    vTaskDelete(NULL);
}