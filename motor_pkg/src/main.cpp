
#include <ros/ros.h>
#include <motor_test/motor_node.h>
#include <my_msgs/CameraData.h>
#include <my_msgs/SensorData.h>
#include <my_msgs/Signal.h>
#include <stdio.h>

#define X_CENTER_1 -100
#define X_CENTER_2 100
#define BIG_SIZE 500
#define SENSOR_DANGER 50

float x_point = 0;
float size = 0;

float FrontSensor = 0;
float LeftSensor = 0;
float RightSensor = 0;

int sequence = 1;

void NumberCallback(const my_msgs::CameraData &msg){
    x_point = msg.x;
    size = msg.size;
}

/*
void NumberCallback2(const my_msgs::SensorData &msg){
    FrontSensor = msg.front;
    LeftSensor = msg.left;
    RightSensor = msg.right;
}
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  Initialize();
  ros::Publisher pub_number = nh.advertise<my_msgs::Signal>("/signal/topic",10); // give signal to Display
  ros::Rate loop_rate(Control_cycle);
  my_msgs::Signal signal;

  while(ros::ok())
  {
    if(sequence == 1)
    {
        ros::Subscriber sub_number = nh.subscribe("/camera/topic", 10, NumberCallback); //camera topic sub
    
        Motor_Controller(1, true, 50);
        Motor_Controller(2, true, 50);
    
        if(x_point <= X_CENTER_1)
        {
            //turn left
        }
    
        else if(x_point >= X_CENTER_2)
        {
            //tun right
        }
        
        if(size >= BIG_SIZE)
        {
            Motor_Controller(1, true, 0);
            Motor_Controller(2, true, 0);
            if(FrontSensor < SENSOR_DANGER)
            {
                pub_number.publish(signal);
                sequence = 2;
                break;
            }
        }
    }
      
    else if(sequence == 2)
    {
        
        if() //end signal input
        {
            sequence = 1;
        }
    }
    
    Motor_View();
    ros::spinOnce();
    loop_rate.sleep();
  }
  Motor_Controller(1, true, 0);
  Motor_Controller(2, true, 0);
  return 0;
}

//Accel_Controller(1, true, 100);
//Accel_Controller(2, true, 100);
//Switch_Turn_Example(100, 100);
//Theta_Distance(180,100,30,110);
