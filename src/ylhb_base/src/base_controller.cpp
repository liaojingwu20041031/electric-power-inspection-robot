#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Linux Serial communication
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <iomanip>

class BaseController : public rclcpp::Node
{
public:
    BaseController() : Node("base_controller"), x_(0.0), y_(0.0), th_(0.0)
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyS1"); 
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<bool>("require_fresh_scan", false);
        this->declare_parameter<double>("scan_timeout_sec", 0.3);
        this->declare_parameter<double>("cmd_timeout_sec", 0.5);
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->declare_parameter<double>("wheel_track", 0.25); 
        this->declare_parameter<bool>("publish_tf", true);

        this->get_parameter("serial_port", serial_port_);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
        this->get_parameter("scan_topic", scan_topic_);
        this->get_parameter("require_fresh_scan", require_fresh_scan_);
        this->get_parameter("scan_timeout_sec", scan_timeout_sec_);
        this->get_parameter("cmd_timeout_sec", cmd_timeout_sec_);
        this->get_parameter("baud_rate", baud_rate_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("wheel_track", wheel_track_);
        this->get_parameter("publish_tf", publish_tf_);

        if (!initSerial()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port_.c_str());
            return;
        }

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10,
            std::bind(&BaseController::cmdVelCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&BaseController::scanCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&BaseController::updateLoop, this));

        last_time_ = this->now();
        last_cmd_received_sec_ = last_time_.seconds();
        RCLCPP_INFO(this->get_logger(), "Base Controller Unit Started Successfully on %s !", serial_port_.c_str());
    }

    ~BaseController()
    {
        if (serial_fd_ > 0) {
            close(serial_fd_);
        }
    }

private:
    std::string serial_port_;
    std::string cmd_vel_topic_ = "cmd_vel";
    std::string scan_topic_ = "/scan";
    bool require_fresh_scan_ = false;
    double scan_timeout_sec_ = 0.3;
    double cmd_timeout_sec_ = 0.5;
    int baud_rate_;
    std::string odom_frame_;
    std::string base_frame_;
    double wheel_track_;
    bool publish_tf_;
    int serial_fd_ = -1;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_, y_, th_;
    rclcpp::Time last_time_;
    double last_cmd_received_sec_ = 0.0;
    double last_scan_received_sec_ = 0.0;
    double last_scan_stamp_sec_ = 0.0;
    bool cmd_timed_out_ = false;

    bool initSerial()
    {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            return false;
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        options.c_cflag |= (CLOCAL | CREAD); 
        options.c_cflag &= ~PARENB;          
        options.c_cflag &= ~CSTOPB;          
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;              

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
        options.c_oflag &= ~OPOST;                          
        options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        tcsetattr(serial_fd_, TCSANOW, &options);
        return true;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (serial_fd_ < 0) return;

        last_cmd_received_sec_ = this->now().seconds();
        cmd_timed_out_ = false;
        const bool nonzero = std::abs(msg->linear.x) > 1e-6 || std::abs(msg->angular.z) > 1e-6;
        if (require_fresh_scan_ && nonzero && !scanIsFresh()) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejecting non-zero %s: %s is stale or missing",
                cmd_vel_topic_.c_str(), scan_topic_.c_str());
            sendVelocityCommand(0.0, 0.0);
            return;
        }

        sendVelocityCommand(msg->linear.x, msg->angular.z);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_received_sec_ = this->now().seconds();
        last_scan_stamp_sec_ = rclcpp::Time(msg->header.stamp).seconds();
    }

    bool scanIsFresh() const
    {
        const double now_sec = this->now().seconds();
        const double stamp_age = now_sec - last_scan_stamp_sec_;
        return last_scan_received_sec_ > 0.0 && last_scan_stamp_sec_ > 0.0 &&
               now_sec - last_scan_received_sec_ <= scan_timeout_sec_ &&
               stamp_age >= -0.1 && stamp_age <= scan_timeout_sec_;
    }

    void sendVelocityCommand(double vx, double vth)
    {
        if (serial_fd_ < 0) return;

        double vl_target = vx - vth * (wheel_track_ / 2.0);
        double vr_target = vx + vth * (wheel_track_ / 2.0);

        int16_t vl_send = static_cast<int16_t>(vl_target * 1000.0);
        int16_t vr_send = static_cast<int16_t>(vr_target * 1000.0);

        uint8_t buffer[7];
        buffer[0] = 0xAA;
        buffer[1] = 0x55;
        buffer[2] = (vl_send >> 8) & 0xFF; 
        buffer[3] = vl_send & 0xFF;        
        buffer[4] = (vr_send >> 8) & 0xFF; 
        buffer[5] = vr_send & 0xFF;        
        buffer[6] = (buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5]) & 0xFF;

        write(serial_fd_, buffer, sizeof(buffer));

        RCLCPP_INFO(this->get_logger(), "Sending to STM32: L_vel=%d, R_vel=%d, Hex: AA 55 %02X %02X %02X %02X %02X", 
                     vl_send, vr_send, buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);
    }

    void updateLoop()
    {
        if (serial_fd_ < 0) return;

        const double command_age = this->now().seconds() - last_cmd_received_sec_;
        if (command_age > cmd_timeout_sec_ && !cmd_timed_out_) {
            RCLCPP_WARN(
                this->get_logger(), "%s timeout %.3fs; sending zero velocity",
                cmd_vel_topic_.c_str(), command_age);
            sendVelocityCommand(0.0, 0.0);
            cmd_timed_out_ = true;
        }

        double dt_vx = 0.0;
        double dt_vth = 0.0;

        uint8_t rx_buf[64];
        int n = read(serial_fd_, rx_buf, sizeof(rx_buf));

        if (n >= 7) { 
            for (int i = 0; i < n - 6; i++) {
                if (rx_buf[i] == 0xAA && rx_buf[i+1] == 0x55) { 
                    uint8_t chksum = (rx_buf[i] + rx_buf[i+1] + rx_buf[i+2] + rx_buf[i+3] + rx_buf[i+4] + rx_buf[i+5]) & 0xFF;
                    if (chksum == rx_buf[i+6]) { 
                        int16_t real_vl_mm = (rx_buf[i+2] << 8) | rx_buf[i+3];
                        int16_t real_vr_mm = (rx_buf[i+4] << 8) | rx_buf[i+5];
                        
                        double vl = static_cast<double>(real_vl_mm) / 1000.0; 
                        double vr = static_cast<double>(real_vr_mm) / 1000.0; 

                        dt_vx = (vr + vl) / 2.0; 
                        dt_vth = (vr - vl) / wheel_track_; 

                        // RCLCPP_INFO_ONCE(this->get_logger(), "Received data from STM32: VL=%.3f, VR=%.3f", vl, vr);
                        break; 
                    }
                }
            }
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double delta_x = (dt_vx * cos(th_)) * dt;
        double delta_y = (dt_vx * sin(th_)) * dt;
        double delta_th = dt_vth * dt;

        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = odom_frame_;
        odom_tf.child_frame_id = base_frame_;
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom_tf.transform.rotation = tf2::toMsg(q);
        
        if (publish_tf_) {
            tf_broadcaster_->sendTransform(odom_tf);
        }

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom.twist.twist.linear.x = dt_vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = dt_vth;

        // 设置里程计的基础协方差值，不然 EKF 等滤波算法无法正常融合
        odom.pose.covariance[0] = 0.001;
        odom.pose.covariance[7] = 0.001;
        odom.pose.covariance[14] = 1e6;
        odom.pose.covariance[21] = 1e6;
        odom.pose.covariance[28] = 1e6;
        odom.pose.covariance[35] = 0.01;

        odom.twist.covariance[0] = 0.001;
        odom.twist.covariance[7] = 1e6;
        odom.twist.covariance[14] = 1e6;
        odom.twist.covariance[21] = 1e6;
        odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = 0.01;

        odom_pub_->publish(odom);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
