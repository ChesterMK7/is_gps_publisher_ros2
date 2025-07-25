// Inertial Sense GPS & IMU data parser ROS node file for Scout 2.0 Autonomous Navigation Project
// Version 1.0.6 (2025-07-01)

#include "gps_parser.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ParsedOutputPublisher : public rclcpp::Node
{
  public:
    ParsedOutputPublisher(int s, sockaddr_in c)
    : Node("gps_output_publisher"), count_(0) {
      sockfd = s; cliaddr = c;
      publisherGPS_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("nav_sat_fix", 10);
      publisherIMU_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
      timer_ = this->create_wall_timer(10ms, std::bind(&ParsedOutputPublisher::timer_callback, this));
    }
  private:
    rclcpp::Clock clock;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisherGPS_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisherIMU_;
    size_t count_;
    sockaddr_in cliaddr;
    int sockfd;
    char buffer[MAXLINE];
    void timer_callback() {
      socklen_t len;
      int n;
      msgdata output;
      len = sizeof(cliaddr);  //len is value/result 
      n = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len); 
      buffer[n] = '\0'; 
      printf("Recieved: %s\n", buffer);
      output = coordParser(buffer);
      std_msgs::msg::Header hd = makeHeader(clock.now());
      if (1 == output.dataType()) {
        sensor_msgs::msg::NavSatFix messageGPS = output.GPSoutputROS();
        hd.frame_id = GPS_FRAME;
        messageGPS.header = hd;
        publisherGPS_->publish(messageGPS);
      }
      else if (2 == output.dataType()) {
        sensor_msgs::msg::Imu messageIMU = output.IMUoutputROS();
        hd.frame_id = IMU_FRAME;
        messageIMU.header = hd;
        publisherIMU_->publish(messageIMU);
      }
    }
};

int main(int argc, char * argv[]) {
  int sockfd = socket(AF_INET,SOCK_DGRAM,0);
  struct sockaddr_in cliaddr = setup(sockfd);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParsedOutputPublisher>(sockfd,cliaddr));
  rclcpp::shutdown();
  return 0;
}
