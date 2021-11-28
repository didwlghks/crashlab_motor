
#include <ros/ros.h>
#include <motor_test/motor_node.h>
#include <my_msgs/CameraData.h>
#include <my_msgs/SensorData.h>
#include <my_msgs/SignalData.h>
#include <iostream>

#define X_CENTER_1 -100
#define X_CENTER_2 100
#define BIG_SIZE 500
#define SENSOR_LIMIT 50


float person_x = 0;
float person_size = 0; //cam data

float FrontSensor = 0;
float LeftSensor = 0;
float RightSensor = 0; //ultrasonic sensor

int sequence = 1;
int state = 0; //signal state


void CamDataCallback(const my_msgs::CameraData &msg){
    person_x = msg.p_x;
    person_size = msg.p_size;
    //sign_x = msg.s_x; 
    //sign_size = msg.s_size; ////////if use sign, activiate
}

void SensorDataCallback(const my_msgs::SensorData &msg){
    FrontSensor = msg.front;
    LeftSensor = msg.left;
    RightSensor = msg.right;
}

void SignalDataCallback(const my_msgs::SignalData &msg){
    state = msg.data;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  Initialize();
  ros::Publisher signal_pub = nh.advertise<my_msgs::SignalData>("/signal/topic",10);
  ros::Rate loop_rate(Control_cycle);
  my_msgs::SignalData data;

  while(ros::ok())
  {
    ros::Subscriber sensor_sub = nh.subscribe("/sensor/topic", 10, SensorDataCallback); //ultrasonic sensor topic sub
      
    if(FrontSensor < SENSOR_LIMIT || LeftSensor < SENSOR_LIMIT || RightSensor < SENSOR_LIMIT)
    {
        Motor_Controller(1, true, 0);
        Motor_Controller(2, true, 0);
    }
      
    if(sequence == 1)
    {
        ros::Subscriber camera_sub = nh.subscribe("/camera/topic", 10, CamDataCallback); //camera topic sub
        
        if(person_x <= X_CENTER_1)
        {
            //turn left
            Motor_Controller(1, true, 30);
            Motor_Controller(2, true, 50);
        }
    
        else if(person_x >= X_CENTER_2)
        {
            //tun right
            Motor_Controller(1, true, 50);
            Motor_Controller(2, true, 30);
        }
        
        else
        {
            Motor_Controller(1, true, 50); //go foward normally
            Motor_Controller(2, true, 50);
        }
        
        if(person_size >= BIG_SIZE)
        {
            Motor_Controller(1, true, 20); //move slow not to be bumped
            Motor_Controller(2, true, 20);
            
            if(FrontSensor < SENSOR_LIMIT)
            {
                Motor_Controller(1, true, 0);
                Motor_Controller(2, true, 0);
                data = 1; //to publish SignalData.data to display
                signal_pub.publish(data);
                sequence++;
            }
        }
    }
      
    else if(sequence == 2)
    {
        ros::Subscriber signal_sub = nh.subscribe("/signal/topic2", 10, SignalDataCallback); //state = subscribe display's signal
        
        if(state == 1) //if sub display's signal
        {
            state = 0;
            data = 0; //reset state and data
            sequence = 1;
        }
    }
      
    /*  //////////come back using sign
    else if(sequence == 2)
    {
        ros::Subscriber signal_sub = nh.subscribe("/signal/topic2", 10, SignalDataCallback); //state = subscribe display's signal
        
        if(state == 1) //if sub display's signal
        {
            state = 0;
            data = 0; //reset state and data
            sequence++;
        }
    }
      
    else if(sequence == 3)
    {
       ros::Subscriber camera_sub = nh.subscribe("/camera/topic", 10, CamDataCallback);
        
       if(sign_x <= X_CENTER_1)
        {
            //turn left
            Motor_Controller(1, true, 30);
            Motor_Controller(2, true, 50);
        }
    
        else if(sign_x >= X_CENTER_2)
        {
            //tun right
            Motor_Controller(1, true, 50);
            Motor_Controller(2, true, 30);
        }
        
        else
        {
            Motor_Controller(1, true, 50); //go foward normally
            Motor_Controller(2, true, 50);
        }
        
        if(sign_size >= BIG_SIZE)
        {
            Motor_Controller(1, true, 0);
            Motor_Controller(2, true, 0);
            sequence = 1;
        }
        
    }
    */
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
