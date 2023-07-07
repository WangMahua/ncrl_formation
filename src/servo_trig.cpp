#ifdef __cplusplus
extern "C"
{
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pigpio.h>
#include <time.h>
}
#endif

#include <ros/ros.h>
#include <termios.h>
#include <sys/select.h>
//#include <stropts.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <std_msgs/Int32.h>

#define NUM_GPIO   32
#define MIN_WIDTH  500
#define MAX_WIDTH  2500

#define TRIG_GPIO_NUM 4  
int run=1;

int step[NUM_GPIO];
int width[NUM_GPIO];
int used[NUM_GPIO];
//===========
#define TRIG_PERIOD   400000000 //nsec
#define MID_WIDTH     1500      //usec
#define OFFSET_WIDTH  400       //usec

#define TRIG_DURATION 350000000 //nsec
#define TRIG_WIDTH    1050	//usec
#define UNTRIG_WIDTH  1420	//usec
#define CMD_LEN	      1
struct timespec trig_prev, t_now;
//===========
static int peek_character = -1;

int readch(void);
int _kbhit(void);

int randint(int from, int to)
{
  return (random() % (to - from + 1)) + from;
}

void stop(int signum)
{
  run = 0;
}

int readch()
{
  char ch;
  if(peek_character != -1)
  {
    ch = peek_character;
    peek_character = -1;
    return ch;
  }
  read(0,&ch,1);
  return ch;
}

int _kbhit()
{
  static const int STDIN = 0;
  static bool initialized = false;
  if (!initialized)
  {
    termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN,TCSANOW, &term);
    setbuf(stdin, NULL);
    initialized = true;
  }
  int bytesWaiting;
  ioctl(STDIN, FIONREAD, &bytesWaiting);
  return bytesWaiting;
}

int fire = 0;

void fire_cb(const std_msgs::Int32 msg)
{
  fire = msg.data;
  ROS_INFO("fire recieved");
  gpioServo(4, TRIG_WIDTH);
}

int main(int argc, char *argv[])
{
  int i, g;
  if(gpioInitialise() < 0) return -1;

  gpioSetSignalFunc(SIGINT, stop);
  gpioServo(4, UNTRIG_WIDTH);

  ros::init(argc, argv, "fire");
  ros::NodeHandle nh;
  ros::Subscriber fire_sub = nh.subscribe<std_msgs::Int32>("/fire", 10, fire_cb);

  if(argc==1) used[TRIG_GPIO_NUM]=1;
  else
  {
    for(i=1;i<argc;i++)
    {
      g = atoi(argv[i]);
      if((g>=0) && (g<NUM_GPIO)) used[g] = 1;
    }
  }

  ros::spin();
  gpioTerminate();
  return 0;
}
