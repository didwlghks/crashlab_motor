
/*
#include<ros/ros.h>
#include<my_msgs/RequestOfMove.h>

void NumberCallback(const my_msgs::RequestOfMove &msg){
    ROS_INFO("sub MR: %d, ML: %d\n", msg.MR, msg.ML);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sub_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_number = nh.subscribe("/Topic/Move", 10, NumberCallback);
    
    ros::spin();
}


*/

#include <ros/ros.h>
#include <motor_test/motor_node.h>
#include <motor_test/pid.h>
#include <my_msgs/CameraData.h>
#include <stdio.h>

#define X_CENTER_1 -100
#define X_CENTER_2 100

float x_point = 0;
float size = 0;
int sequence = 1;

void NumberCallback(const my_msgs::CameraData &msg){
    x_point = msg.x;
    size = msg.size;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  Initialize();
  ros::Rate loop_rate(Control_cycle);

  while(ros::ok() && sequence == 1)
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

/*
int state = 0;
//0 : 홍보 종료상태
//1 : 홍보 진행중



While(ros::ok) 

if (사람의 좌표가 안 들어올때 && state == 0)
{
제자리회전 // 기본주행 어떻게 할지, 정지해있을지?
} 

// 여러 사람중 한 사람 선택했을때부터 시작해서 그 사람 바운딩박스값만 줄 수 있는지?


While(통신으로 사람의 좌표가 들어올때 && state == 0)
{
  if (사람의 위치가 카메라상의 왼편일때)
  오른쪽바퀴 턴 

  if (사람의 위치가 카메라상의 오른편일 때)
  왼쪽바퀴 턴 

  if (사람의 위치가 카메라상의 가운데편)
  직진 

  if (바운딩 박스의 크기가 기준값이상 커졌을때 or 초음파센서로 근접인식 시) 

모터정지 

// 바운딩박스 크기가 기준값보다 크다 = 사람에게 근접했다 -> 모터정지 필요 

//초음파센서값으로 충돌고려해서 모터정지? 장애물 우회 필요 -> 장애물 우회코드 함수로 만들기! 

  if (일정시간동안 바운딩박스 크기가 기준값보다 클때) //사람에 근접 도달 후 사람이 도망 안가면 홍보진행 

모터정지 후 디스플레이 출력신호주기?
state = 1;
break; 

홍보 과정 종료시 신호받아서 state = 0으로 초기화해줘야
  */
