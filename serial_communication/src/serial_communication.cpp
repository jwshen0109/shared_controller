#include <string>
#include "ros/ros.h" // 包含ROS的头文件
#include <boost/asio.hpp> //包含boost库函数
#include <boost/bind.hpp>
#include "std_msgs/Float64MultiArray.h" //ros定义的String数据类型
#include "std_msgs/String.h"
#include "fstream"

using namespace std;
using namespace boost::asio; //定义一个命名空间，用于后面的读写操作

unsigned char buf[120]; //定义字符串长度
unsigned char buf_tmp[120];

//ofstream output;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "serial_communication"); //初始化节点
  ros::NodeHandle n;

  ros::Publisher touch_force_pub = n.advertise<std_msgs::Float64MultiArray>("/touch_force", 1000); //定义发布消息的名称及sulv
  ros::Rate loop_rate(100);

  io_service iosev;
  serial_port sp(iosev, "/dev/ttyUSB0"); //定义传输的串口
  sp.set_option(serial_port::baud_rate(115200));
  sp.set_option(serial_port::flow_control());
  sp.set_option(serial_port::parity());
  sp.set_option(serial_port::stop_bits());
  sp.set_option(serial_port::character_size(8));

  int count = 0;
  int loop = 0;

  //output.open("/home/jwshen-310/nt_1.txt");

  while (ros::ok())
  {
    // if(count == 0){
    //     write(sp, buffer("0", 1));
    // }
    // if(count == 100){
    //     write(sp, buffer("1", 1));
    // }
    // if(count <= 120){
    //     count++;
    // }
 
    //write(sp, buffer(buf1, 6));  //write the speed for cmd_val
    //write(sp, buffer("Hellq world", 12));
    read(sp, buffer(buf));
    int x;
    string str;
    for(int i=0; i<120; i++){
      if(buf[i]==0x0a){
        //string str(&buf[i+1],&buf[i+18]);
        x = i+1;
        break;
      }
    }


    // string str1(&buf[x],&buf[x+8]);
    // string str2(&buf[x+10],&buf[x+18]);
    // string str3(&buf[x+20],&buf[x+27]);
    // string str4(&buf[x+29],&buf[x+36]);
    // string str5(&buf[x+38],&buf[x+45]);
    // string str6(&buf[x+47],&buf[x+54]);

    string str1(&buf[x],&buf[x+7]);
    string str2(&buf[x+9],&buf[x+16]);
    string str3(&buf[x+18],&buf[x+25]);
    string str4(&buf[x+27],&buf[x+34]);
    string str5(&buf[x+36],&buf[x+43]);
    string str6(&buf[x+45],&buf[x+52]);

    float x1 = atof(str1.c_str());
    float x2 = atof(str2.c_str());
    float x3 = atof(str3.c_str());
    float x4 = atof(str4.c_str());
    float x5 = atof(str5.c_str());
    float x6 = atof(str6.c_str());
    // output << x1 << "\t";
    // output << x2 << "\t";
    // output << x3 << "\t";
    // output << x4 << "\t";
    // output << x5 << "\t";
    // output << x6 << endl;
    //string str(&buf[0], &buf[39]); //将数组转化为字符串
    // output << x1 << "\t";
    // output << x2 << endl;
    //if (buf[10] == '\n')
    std_msgs::Float64MultiArray msg_data;
    //std::stringstream ss;
    //ss << str;

    msg_data.data.push_back(x1);
    msg_data.data.push_back(x2);
    msg_data.data.push_back(x3);
    msg_data.data.push_back(x4);
    msg_data.data.push_back(x5);
    msg_data.data.push_back(x6);
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str()); //打印接受到的字符串
    touch_force_pub.publish(msg_data);  //发布消息
    // if(loop == 0) {
    //   ROS_INFO("start loop"); //
    // }else{
    //   ROS_INFO("loop: %i", loop); //
    // }
    // loop++;
    ros::spinOnce();

    loop_rate.sleep();
  }

  iosev.run();
  return 0;
}
