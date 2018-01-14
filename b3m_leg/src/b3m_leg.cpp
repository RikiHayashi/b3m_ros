#include <b3m_leg/B3M.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <signal.h>

class B3MLeg {
 public:
  B3MLeg() : b3m_("/dev/ttyUSB0") {
    for (int id = 1; id < 6; id++) {
      b3m_.set_mode(id, B3M::MODE::Free);
    }
    b3m_.servo_sleep(1);
    for (int id = 1; id < 6; id++) {
      b3m_.set_mode(id, B3M::MODE::Normal);
    }
    b3m_.servo_sleep(1);
    init_leg();
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("b3m_leg_angle", 1);
    sub_ = nh_.subscribe("/b3m_leg_setangle", 1, &B3MLeg::callback, this);
  }

  void spin() {
    ros::Rate rate(100);
    ros::Rate rate2(1000);
    while (nh_.ok()) {
//    std_msgs::Float64MultiArray angledata;
//    for (int id = 1; id < 6; id++) {
//        double data = b3m_.get_angle(id) / 100.0;
//        angledata.data.push_back(data);
//        rate2.sleep();
 //     }
   //   pub_.publish(angledata);
      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  void callback(const std_msgs::Float64MultiArrayConstPtr &msg) {
    for (int i = 0; i < 5; i++) {
      b3m_.set_angle(i + 1, msg->data[i]);
      b3m_.servo_usleep(300);
    }
  }

  void init_leg() {
    for (int id = 1; id < 6; id++) {
      b3m_.set_angle(id, 0.0);
      b3m_.servo_usleep(100);
    }
    b3m_.servo_sleep(100);
  }

  B3M::B3M b3m_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "b3m_leg");
  B3MLeg b3mleg;
  b3mleg.spin();
  return 0;
}
