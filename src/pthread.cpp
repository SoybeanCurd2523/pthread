#ifdef __linux__
#include <ros/ros.h> ////
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif


#include <iostream>
#include "std_msgs/Int32.h"
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <time.h>


#include "dynamixel_sdk/dynamixel_sdk.h"     
#define Control_Cycle 1
bool pthread_is_running = true;

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     1                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define PI 3.141592




  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  double t=0.0;
  int T=20;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//   int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
  // int dxl_goal_position[2] = { 1024, 3072 };
  int dxl_goal_position = 0;

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position



void process();

void *p_function(void * data)
{
  pid_t pid; //process id
  pthread_t tid; // thread id

  pid = getpid(); //4
  tid = pthread_self();

  char* thread_name = (char *)data;
  int i = 0;

  static struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(pthread_is_running)
  {
    next_time.tv_sec += (next_time.tv_nsec + Control_Cycle * 1000000) / 1000000000;
    next_time.tv_nsec += (next_time.tv_nsec + Control_Cycle * 1000000) % 1000000000;

    process();

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}


int main(int argc, char **argv)
{ 

  ros::init(argc, argv, "pthread_read_write");
  ros::NodeHandle nh;

  ros::Publisher pthread_read_write_pub = nh.advertise<std_msgs::Int32>("dxl_goal_position", 1000);


//////////////////




  // Open port
  // portHandler->openPort()  :  성공하면 1, 실패하면 0
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    // printf("Press any key to terminate...\n");
    // getch();
    return 0;
  }

  // Set port baudrate
  // portHandler->setBaudRate(BAUDRATE) : 성공하면 1, 실패하면 0
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    // printf("Press any key to terminate...\n");
    // getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);  // dxl_error : 이상이 없으면 0
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }


///////////////////


  pthread_t pthread;
  int thr_id;
  int status;

   char p1[] = "thread_1";
  
  sleep(1);

  thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1);  // thread 진입

  
  if(thr_id < 0)
  {
    perror("pthread0 create error");
    exit(EXIT_FAILURE);
  }


//////////////////////


  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std_msgs::Int32 msg;
    msg.data = dxl_goal_position;
    ROS_INFO("I speak : [%d]", msg.data);
    pthread_read_write_pub.publish(msg); 
    ros::spinOnce();

    loop_rate.sleep();
  }

//////////////


  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler-> getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler-> getRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();


//////////////////////



  pthread_is_running = false;
  return 0;
}

void process(){

    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &dxl_error); //잘되면 0을 반환
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    sleep(0.1);
    // do
    // {
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position, dxl_present_position);



    t += 0.1;
    if(t > T) {t=0;}
    
    dxl_goal_position = ((4095-0)/2) * (1 - cos(2*PI*t/ T));
    // 2*PI 인 이유는 4095 에 갔다가 0으로 다시 돌아와야 하기 때문
//   int index = 0;
}