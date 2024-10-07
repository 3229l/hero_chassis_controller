#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h> //包含与操作系统交互的底层函数，如文件描述符控制
#include <termios.h> //用于修改终端的输入模式，允许读取键盘的非阻塞输入

#include <map> //map 容器，用于存储按键与移动/速度绑定关系

//移动按键
std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

// 加速倍率
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

)";

float speed(1.0); // 线速度初始值 (m/s)
float turn(1.0); // 角速度初始值 (rad/s)
float x(0), y(0), z(0), th(0); //  控制机器人方向
char key(' '); //存储按下的键值

// 非阻塞式键盘输入
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt); //从STDIN_FILENO获取信息，并存储到oldt
  newt = oldt;
  //ICANON:关闭了规范模式，程序可以立即读取用户按下的字符
  //ECHO:关闭回显,避免在终端上显示按下的字符
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1; //最少读取一个字符才能返回输入
  newt.c_cc[VTIME] = 0; //设置没有读取超时时间
  tcsetattr(fileno(stdin), TCSANOW, &newt); //修改后的 newt 设置应用到终端

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); //恢复终端的原始设置

  return ch;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist twist;
  printf("%s", msg);
  printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r", speed, turn);

  while(ros::ok) {
    key = getch();

    if (moveBindings.count(key) == 1) {
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];
      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    } else if (speedBindings.count(key) == 1) {
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];
      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    } else {
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      if (key == '\x03')// If ctrl-C
      break;
      printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
    }

    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    pub.publish(twist);
    ros::spinOnce();
  }
  return 0;
}