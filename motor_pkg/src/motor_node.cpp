/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <ros/ros.h>
#include <motor_test/motor_node.h>
#include <my_msgs/CameraData.h>
#include <my_msgs/SensorData.h>
#include <my_msgs/SignalData.h>
#include <iostream>
#include <fstream>

////////////////////////////////////
#define X_BOUND_1 -100
#define X_BOUND_2 100
#define BIG_SIZE 500
#define SENSOR_LIMIT 50
///////////////////////////////////// define value, 일단 임의값으로 해놓음

/////////////////////////////
bool person_detect = false;
bool sign_detect = false;
float person_x = 0;
float person_size = 0;
float sign_x = 0;
float sign_size = 0;
/////////////////////////////// cam data

/////////////////////////// 
float FrontSensor = 0;
float LeftSensor = 0;
float RightSensor = 0;
float BackSensor = 0;
////////////////////////// sensor data

///////////////
int state = 0; // display data
///////////////
int sequence = 0; // sequence 0 to 3

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/motor_node/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Acceleration_ratio = atof(line.substr(found+2).c_str()); break;
      case 5: Wheel_radius = atof(line.substr(found+2).c_str()); break;
      case 6: Robot_radius = atof(line.substr(found+2).c_str()); break;
      case 7: Encoder_resolution = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}
int Motor_Setup(void)
{
  pinum=pigpio_start(NULL, NULL);
  
  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_DIR, PI_OUTPUT);
  set_mode(pinum, motor2_DIR, PI_OUTPUT);
  set_mode(pinum, motor1_PWM, PI_OUTPUT);
  set_mode(pinum, motor2_PWM, PI_OUTPUT);
  set_mode(pinum, motor1_ENA, PI_INPUT);
  set_mode(pinum, motor1_ENB, PI_INPUT);
  set_mode(pinum, motor2_ENA, PI_INPUT);
  set_mode(pinum, motor2_ENB, PI_INPUT);

  gpio_write(pinum, motor1_DIR, PI_LOW);
  gpio_write(pinum, motor2_DIR, PI_LOW);

  set_PWM_range(pinum, motor1_PWM, PWM_range);
  set_PWM_range(pinum, motor2_PWM, PWM_range);
  set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
  set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

  set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

  current_PWM1 = 0;
  current_PWM2 = 0;

  current_Direction1 = true;
  current_Direction2 = true;

  acceleration = PWM_limit/(Acceleration_ratio);

  ROS_INFO("Setup Fin");
  return 0;
}
void Interrupt_Setting(void)
{
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1A ++;
  else EncoderCounter1A --;
  EncoderSpeedCounter1 ++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1B ++;
  else EncoderCounter1B --;
  EncoderSpeedCounter1 ++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A --;
  else EncoderCounter2A ++;
  EncoderSpeedCounter2 ++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B --;
  else EncoderCounter2B ++;
  EncoderSpeedCounter2 ++;
}
int Motor1_Encoder_Sum()
{
  EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
  return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
  EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
  return EncoderCounter2;
}
void Init_Encoder(void)
{
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  Init_Encoder();
  Interrupt_Setting();

  Wheel_round = 2*PI*Wheel_radius;
  Robot_round = 2*PI*Robot_radius;

  switch_direction = true;
  Theta_Distance_Flag = 0;

  ROS_INFO("PWM_range %d", PWM_range);
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
  ROS_INFO("Initialize Complete");

  printf("\033[2J");  
}

void Motor_Controller(int motor_num, bool direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);
	
  if(motor_num == 1)
  {
    if(direction == true)
    {
      gpio_write(pinum, motor1_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = true;
    }
    else if(direction == false)
    {
      gpio_write(pinum, motor1_DIR, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = false;
    }
  }
  
  else if(motor_num == 2)
  {
   if(direction == true)
   {
     gpio_write(pinum, motor2_DIR, PI_LOW);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = true;
   }
   else if(direction == false)
   {
     gpio_write(pinum, motor2_DIR, PI_HIGH);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = false;
   }
  }
}
void Accel_Controller(int motor_num, bool direction, int desired_pwm)
{
  bool local_current_direction;
  int local_PWM;
  int local_current_PWM;

  if(motor_num == 1)
  {
    local_current_direction = current_Direction1;
    local_current_PWM = current_PWM1;
  }
  else if(motor_num == 2)
  {
    local_current_direction = current_Direction2;
    local_current_PWM = current_PWM2;
  }

  if(direction == local_current_direction)
  {
    if(desired_pwm > local_current_PWM)
    {
      local_PWM = local_current_PWM + acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else if(desired_pwm < local_current_PWM)
    {
      local_PWM = local_current_PWM - acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
  else
  {
	  if(desired_pwm >= 0)
	  {
      local_PWM = local_current_PWM - acceleration;
      if(local_PWM <= 0)
      {
        local_PWM = 0;
        Motor_Controller(motor_num, direction, local_PWM);
      }
      else Motor_Controller(motor_num, local_current_direction, local_PWM);
	  }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
}

void Switch_Turn_Example(int PWM1, int PWM2)
{
  int local_PWM1 = Limit_Function(PWM1);
  int local_PWM2 = Limit_Function(PWM2);
  if(switch_direction == true)
  {
    Motor_Controller(1, switch_direction, local_PWM1);
    Motor_Controller(2, switch_direction, local_PWM2);
    switch_direction = false;
    ROS_INFO("true");
  }
  else
  {
    Motor_Controller(1, switch_direction, local_PWM1);
    Motor_Controller(2, switch_direction, local_PWM2);
    switch_direction = true;
    ROS_INFO("false");
  }
}
void Theta_Turn(double Theta, int PWM)
{
  double local_encoder;
  int local_PWM = Limit_Function(PWM);
  if(Theta_Distance_Flag == 1)
  {
      Init_Encoder();
      Theta_Distance_Flag = 2;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if(Theta > 0)
  {
    local_encoder = (Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
    Motor_Controller(1, false, local_PWM);
    Motor_Controller(2, false, local_PWM);
    //Accel_Controller(1, false, local_PWM);
    //Accel_Controller(2, false, local_PWM);
  }
  else
  {
    local_encoder = -(Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
    Motor_Controller(1, true, local_PWM);
    Motor_Controller(2, true, local_PWM);
    //Accel_Controller(1, true, local_PWM);
    //Accel_Controller(2, true, local_PWM);
  }

  if(EncoderCounter1 > local_encoder)
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    Theta_Distance_Flag = 3;
  }
}
void Distance_Go(double Distance, int PWM)
{
  double local_encoder = (Encoder_resolution*4*Distance)/Wheel_round;
  int local_PWM = Limit_Function(PWM);
  bool Direction = true;
  if(Distance < 0)
  {
    Direction = false;
    local_encoder = -local_encoder;
  }
  if(Theta_Distance_Flag == 3)
  {
      Init_Encoder();
      Theta_Distance_Flag = 4;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if(EncoderCounter1 < local_encoder)
  {
    if(Direction==true)
    {
      Motor_Controller(1, false, local_PWM);
      Motor_Controller(2, true, local_PWM);
      //Accel_Controller(1, false, local_PWM);
      //Accel_Controller(2, true, local_PWM);
    }
    else
    {
      Motor_Controller(1, true, local_PWM);
      Motor_Controller(2, false, local_PWM);
      //Accel_Controller(1, true, local_PWM);
      //Accel_Controller(2, false, local_PWM);
    }
  }
  else
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    //Accel_Controller(1, true, 0);
    //Accel_Controller(2, true, 0);
    Theta_Distance_Flag = 0;
  }
}
void Theta_Distance(double Theta, int Turn_PWM, double Distance, int Go_PWM)
{
  if(Theta_Distance_Flag == 0)
  {
    Theta_Distance_Flag = 1;
  }
  else if(Theta_Distance_Flag == 1 || Theta_Distance_Flag == 2)
  {
    Theta_Turn(Theta, Turn_PWM);
  }
  else if(Theta_Distance_Flag == 3 || Theta_Distance_Flag == 4)
  {
    Distance_Go(Distance, Go_PWM);
  }
}

int Limit_Function(int pwm)
{
  int output;
  if (pwm > PWM_limit*2)
  {
    output = PWM_limit;
    ROS_WARN("PWM too fast!!!");
  }
  else if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)
  {
	output = 0;
    ROS_WARN("trash value!!!");
  }
  else output = pwm;
  return output; 
}
void RPM_Calculator()
{
  RPM_Value1 = (EncoderSpeedCounter1*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter1 = 0;
  RPM_Value2 = (EncoderSpeedCounter2*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter2 = 0;
}
void Motor_View()
{
	RPM_Calculator();
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
	printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
	printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
	printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
	printf("Acc  :%10.0d\n", acceleration);
	printf("\n");
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////

void CamDataCallback(const my_msgs::CameraData& msg) { // function which subscribes from camera
    person_detect = msg.p_detect;
    person_x = msg.p_x;
    person_size = msg.p_size;
    sign_detect = msg.s_detect;
    sign_x = msg.s_x;
    sign_size = msg.s_size; //동현이형이랑 메시지명 맞춰야되는 부분 p_detect, p_x 등 
}

void SensorDataCallback(const my_msgs::SensorData& msg) {
    FrontSensor = msg.front;
    LeftSensor = msg.left;
    RightSensor = msg.right;
    BackSensor = msg.back;
}

void SignalDataCallback(const my_msgs::SignalData& msg) {
    state = msg.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;
    Initialize();
    ros::Publisher signal_pub = nh.advertise<my_msgs::SignalData>("/signal/topic", 10);
    ros::Subscriber signal_sub = nh.subscribe("/signal/topic2", 10, SignalDataCallback);
    ros::Subscriber sensor_sub = nh.subscribe("/sensor/topic", 10, SensorDataCallback);
    ros::Subscriber camera_sub = nh.subscribe("/camera/topic", 10, CamDataCallback); //camera topic sub
    ros::Rate loop_rate(Control_cycle);
    my_msgs::SignalData signal_msg;

    while (ros::ok())
    {

        if (sequence == 0)
        {
            camera_sub;
            sensor_sub; //subscribe camera and sensor data

            if (person_detect) {        // object-detection 여부에 대한 bool형 - 사람
                Motor_Controller(1, true, 0);
                Motor_Controller(2, true, 0);
                sequence++;    // 다음 단계
            }
            Motor_Controller(1, true, 30);
            Motor_Controller(2, false, 30);
        }

        if (sequence == 1)
        {
            camera_sub;
            sensor_sub;
            if (FrontSensor < SENSOR_LIMIT || person_size > BIG_SIZE) {      // 전진 센서 작동 or 사이즈 커졌을 때
                Motor_Controller(1, true, 0);
                Motor_Controller(2, true, 0);
                signal_msg.data = 1 // 사람에게 도달했을 때 디스플레이에 데이터 1로 신호 pub
                signal_pub.publish(signal_msg);      // pub~~~ -> '스크린 Rpi'로 데이터 넘겨줌(interaction start)

                person_detect = 0;
                person_x = 0;
                person_size = 0; //camera data 초기화
                sequence++;    // 다음 단계
            }

            else
            {
                if (person_x > X_BOUND_1 && person_x < X_BOUND_2) {     // x_person: detection된 사람 화면에서 x값, v1,v2는 바운더리, 내에 있을 때 전진
                    Motor_Controller(1, true, 50);
                    Motor_Controller(2, true, 50);
                }
                else if (person_x <= X_BOUND_1) {    // 왼쪽으로 회전
                    Motor_Controller(1, true, 30);
                    Motor_Controller(2, true, 50);      // pid제어를 통해 값 변화에 따라 회전값 증가 필요
                }
                else {
                    Motor_Controller(1, true, 50);
                    Motor_Controller(2, true, 30);
                }
            }
        }
        if (sequence == 2)     // Screen interaction  나중에 없애도 됌.
        {
            signal_sub;
            if (state == 9999) // 9999 = display's end_signal
            {
                sequence++;; // 다음단계
                break;
            }

            /*
            if (x < V1) {    // 제자리 왼쪽으로 회전
                Motor_Controller(1, true, 30);
                Motor_Controller(2, false, 30);      // pid제어를 통해 값 변화에 따라 회전값 증가 필요
            }

            else(x > V2){   // 제자리 오른쪽 회전
                Motor_Controller(1, false, 30);
                Motor_Controller(2, true, 30);      // 반복문이라 생각 조금 더
             */
        }
        if(sequence == 3)
        {
            camera_sub;
            sensor_sub;
            if (sign_detect)     // 표지판 detection
            {
                if (x_sign == V)   // 표지판 정면(V)에 두기 //지환 : V가 무슨 값 말하는건지 모르겠어어 일단 놔둠, 필요하면 위에 define으로 임의값 정의
                {
                    if (size_sign > V)
                    {
                        Motor_Controller(1, true, 30);
                        Motor_Controller(2, true, 30);
                    }
                    else       // 크기가 작다, 거리가 멀다 => 향해서 전진
                    {
                        Motor_Controller(1, true, 0);
                        Motor_Controller(2, true, 0);
                        person_detect = 0;
                        person_x = 0;
                        person_size = 0; //camera data 초기화
                        sequence = 0; //sequence 초기화
                    }
                }
                else
                {
                    Motor_Controller(1, true, 20);
                    Motor_Controller(2, false, 20);     // 움직이던 방향대로 조금 회전
                }
            }
            else
            {
                Motor_Controller(1, true, 30);
                Motor_Controller(2, false, 30);     // detection 하기 전까지 한쪽 방향으로 회전
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0); //노드 종료시 무조건 모터정지
    return 0;
}
