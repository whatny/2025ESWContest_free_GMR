#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <regex>
#include <cstdio>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <Eigen/Dense>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class UWBReceiver : public rclcpp::Node 
{
public:
  UWBReceiver() : Node("uwb_position"), 
                  tf_buffer_(this->get_clock()), 
                  tf_listener_(tf_buffer_)
  {
    // 설정값들
    port_name_ = "/dev/ttyACM0";
    baud_rate_ = B9600;
    min_calibration_points_ = 5;
    min_uwb_distance_ = 0.1;
    min_map_distance_ = 0.1;
    time_sync_threshold_ms_ = 5000;
    estimate_scale_ = false;
    degenerate_threshold_ = 1e-4;
    uwb_scale_factor_ = 0.01;
    
    // 시리얼 포트 열기
    serial_fd_ = open_serial_port(port_name_);
    if (serial_fd_ >= 0) {
      RCLCPP_INFO(this->get_logger(), "시리얼 포트 %s 연결 성공", port_name_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "시리얼 포트 %s 연결 실패", port_name_.c_str());
    }
    
    // 퍼블리셔 생성 (2개만)
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/bt_position_uwb", 10);
    uwb_aligned_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/bt_position", 10);
    
    // 서브스크라이버 생성
    rtab_localization_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/rtabmap/localization_pose", 10, 
      std::bind(&UWBReceiver::localization_callback, this, std::placeholders::_1));
    
    // Static TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // 타이머 설정
    timer_ = this->create_wall_timer(
      60ms, std::bind(&UWBReceiver::read_serial, this));
    
    connection_timer_ = this->create_wall_timer(
      60000ms, std::bind(&UWBReceiver::check_connection, this));
    
    calibration_timer_ = this->create_wall_timer(
      5000ms, std::bind(&UWBReceiver::check_calibration_progress, this));
    
    // 상태 변수 초기화
    start_time_ = std::chrono::system_clock::now();
    last_data_time_ = std::chrono::system_clock::time_point();
    last_uwb_ros_time_ = rclcpp::Time(0);
    last_rtab_ros_time_ = rclcpp::Time(0);
    have_x_value_ = false;
    have_y_value_ = false;
    x_value_ = 0.0;
    y_value_ = 0.0;
    z_value_ = 0.0;
    
    // 캘리브레이션 상태 초기화
    calibration_complete_ = false;
    tf_published_ = false;
    localization_available_ = false;
    has_latest_uwb_ = false;
    estimated_scale_ = 1.0;
    
    RCLCPP_INFO(this->get_logger(), "UWB 위치 데이터 수신 노드가 시작되었습니다");
    RCLCPP_INFO(this->get_logger(), "설정:");
    RCLCPP_INFO(this->get_logger(), "  - 최소 캘리브레이션 포인트: %d개", min_calibration_points_);
    RCLCPP_INFO(this->get_logger(), "  - UWB 최소 이동거리: %.1fm", min_uwb_distance_);
    RCLCPP_INFO(this->get_logger(), "  - Map 최소 이동거리: %.1fm", min_map_distance_);
    RCLCPP_INFO(this->get_logger(), "  - UWB 스케일: %.3f (cm→m)", uwb_scale_factor_);
    RCLCPP_INFO(this->get_logger(), "  - 스케일 추정: %s", estimate_scale_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), "RTAB-Map localization 모드를 기다리는 중...");
  }
  
  ~UWBReceiver() 
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  // 시리얼 포트 관련 함수들
  int open_serial_port(const std::string& port_name) 
  {
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      return -1;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
      close(fd);
      return -1;
    }
    
    cfsetospeed(&tty, baud_rate_);
    cfsetispeed(&tty, baud_rate_);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    tty.c_iflag &= ~IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL);
    
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      close(fd);
      return -1;
    }
    
    return fd;
  }
  
  void check_connection() 
  {
    auto now = std::chrono::system_clock::now();
    
    const bool have_last_data = last_data_time_.time_since_epoch().count() != 0;
    const bool data_timeout = have_last_data && 
      std::chrono::duration_cast<std::chrono::seconds>(now - last_data_time_).count() > 10;
    
    if (serial_fd_ < 0 || data_timeout) {
      if (serial_fd_ >= 0) {
        close(serial_fd_);
      }
      
      serial_fd_ = open_serial_port(port_name_);
      if (serial_fd_ >= 0) {
        RCLCPP_INFO(this->get_logger(), "시리얼 포트 %s 재연결 성공", port_name_.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "시리얼 포트 %s 재연결 실패", port_name_.c_str());
      }
    }
  }
  
  void localization_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "localization_callback 호출됨");
    
    if (msg->header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "localization_pose frame_id=%s (기대값: map)", msg->header.frame_id.c_str());
    }
    
    if (!localization_available_) {
      localization_available_ = true;
      RCLCPP_INFO(this->get_logger(), "RTAB-Map localization 모드 감지! 정렬 시작합니다");
      RCLCPP_INFO(this->get_logger(), "로봇을 서로 다른 위치로 움직여서 %d개 대응쌍을 수집해주세요", min_calibration_points_);
    }
    
    if (calibration_complete_) {
      return;
    }
    
    last_rtab_position_.x = msg->pose.pose.position.x;
    last_rtab_position_.y = msg->pose.pose.position.y;
    last_rtab_position_.z = msg->pose.pose.position.z;
    last_rtab_ros_time_ = rclcpp::Time(msg->header.stamp);
    
    RCLCPP_INFO(this->get_logger(), "RTAB 위치: (%.2f,%.2f,%.2f)", 
                last_rtab_position_.x, last_rtab_position_.y, last_rtab_position_.z);
    
    if (has_latest_uwb_) {
      RCLCPP_INFO(this->get_logger(), "add_calibration_point() 호출 시도");
      add_calibration_point();
    } else {
      RCLCPP_WARN(this->get_logger(), "UWB 데이터 없음");
    }
  }
  
  // UWB 데이터 처리 (시리얼에서 받은 후 호출)
  void process_uwb_data(double x, double y, double z)
  {
    // UWB 스케일 적용 (cm → m 변환)
    x *= uwb_scale_factor_;
    y *= uwb_scale_factor_;
    z *= uwb_scale_factor_;
    
    // 기존 호환성을 위한 Point 메시지
    geometry_msgs::msg::Point point_msg;
    point_msg.x = x;
    point_msg.y = y;
    point_msg.z = z;
    publisher_->publish(point_msg);  // /bt_position 발행
    
    // UWB 원본 위치 생성 (내부 처리용)
    geometry_msgs::msg::PointStamped uwb_raw;
    uwb_raw.header.stamp = this->get_clock()->now();
    uwb_raw.header.frame_id = "uwb_world";
    uwb_raw.point.x = x;
    uwb_raw.point.y = y;
    uwb_raw.point.z = z;
    
    // 최신 UWB 데이터 저장
    latest_uwb_ = uwb_raw;
    has_latest_uwb_ = true;
    last_uwb_ros_time_ = this->get_clock()->now();
    
    // 캘리브레이션이 완료되면 변환된 좌표 발행
    if (calibration_complete_) {
      try {
        geometry_msgs::msg::PointStamped transformed = uwb_raw;
        
        if (estimate_scale_ && estimated_scale_ != 1.0) {
          transformed.point.x *= estimated_scale_;
          transformed.point.y *= estimated_scale_;
          transformed.point.z *= estimated_scale_;
        }
        
        if (tf_buffer_.canTransform("map", "uwb_world", tf2::TimePointZero)) {
          tf_buffer_.transform(transformed, transformed, "map", tf2::durationFromSec(0.1));
          uwb_aligned_publisher_->publish(transformed);  // /bt_position 발행
          
          RCLCPP_INFO(this->get_logger(), 
                      "UWB | Raw:(%.2f,%.2f,%.2f) → Map:(%.2f,%.2f,%.2f) [완료]", 
                      uwb_raw.point.x, uwb_raw.point.y, uwb_raw.point.z,
                      transformed.point.x, transformed.point.y, transformed.point.z);
        }
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "UWB 변환 실패: %s", ex.what());
      }
    } else {
      std::string status = !localization_available_ ? "대기중" : "수집중";
      
      RCLCPP_INFO(this->get_logger(), 
                  "UWB | Raw:(%.2f,%.2f,%.2f) [%s]", 
                  uwb_raw.point.x, uwb_raw.point.y, uwb_raw.point.z, status.c_str());
    }
    
    last_data_time_ = std::chrono::system_clock::now();
  }
  
  void add_calibration_point()
  {
    if (!localization_available_ || calibration_complete_ || !has_latest_uwb_) {
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "add_calibration_point() 실행");
    
    Eigen::Vector3d current_uwb(latest_uwb_.point.x, latest_uwb_.point.y, latest_uwb_.point.z);
    Eigen::Vector3d current_map(last_rtab_position_.x, last_rtab_position_.y, last_rtab_position_.z);
    
    // UWB 공간에서 거리 필터링
    for (const auto& uwb_point : uwb_points_) {
      if ((current_uwb - uwb_point).norm() < min_uwb_distance_) {
        return;
      }
    }
    
    // Map 공간에서도 이동량 체크
    if (!map_points_.empty()) {
      if ((current_map - map_points_.back()).norm() < min_map_distance_) {
        return;
      }
    }
    
    // 퇴화 감지
    if (uwb_points_.size() >= 2) {
      const auto& a = uwb_points_[uwb_points_.size()-2];
      const auto& b = uwb_points_[uwb_points_.size()-1];
      const auto& c = current_uwb;
      double area_squared = (b-a).cross(c-a).squaredNorm();
      if (area_squared < degenerate_threshold_) {
        return;
      }
    }
    
    // 새로운 대응쌍 추가
    uwb_points_.push_back(current_uwb);
    map_points_.push_back(current_map);
    
    RCLCPP_INFO(this->get_logger(), 
                "대응쌍 %zu/%d | UWB:(%.2f,%.2f,%.2f) ↔ Map:(%.2f,%.2f,%.2f)", 
                uwb_points_.size(), min_calibration_points_,
                current_uwb.x(), current_uwb.y(), current_uwb.z(),
                current_map.x(), current_map.y(), current_map.z());
    
    if (uwb_points_.size() >= min_calibration_points_) {
      perform_alignment();
    }
  }
  
  void perform_alignment()
  {
    if (uwb_points_.size() < 3 || tf_published_) {
      return;
    }
    
    auto tf = compute_alignment();
    
    if (!tf_published_) {
      tf_broadcaster_->sendTransform(tf);
      tf_published_ = true;
      calibration_complete_ = true;
      
      RCLCPP_INFO(this->get_logger(), "SVD 기반 3D 정렬 완료! Static TF 발행됨");
      RCLCPP_INFO(this->get_logger(), "   Translation: (%.3f, %.3f, %.3f)", 
                  tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
      RCLCPP_INFO(this->get_logger(), "   Rotation: (%.3f, %.3f, %.3f, %.3f)", 
                  tf.transform.rotation.x, tf.transform.rotation.y, 
                  tf.transform.rotation.z, tf.transform.rotation.w);
      
      if (estimate_scale_) {
        RCLCPP_INFO(this->get_logger(), "   Estimated Scale: %.6f", estimated_scale_);
      }
    }
  }
  
  geometry_msgs::msg::TransformStamped compute_alignment()
  {
    size_t N = map_points_.size();
    Eigen::MatrixXd M(3, N), U(3, N);
    
    for (size_t i = 0; i < N; ++i) {
      M.col(i) = map_points_[i];
      U.col(i) = uwb_points_[i];
    }
    
    Eigen::Vector3d map_centroid = M.rowwise().mean();
    Eigen::Vector3d uwb_centroid = U.rowwise().mean();
    
    M.colwise() -= map_centroid;
    U.colwise() -= uwb_centroid;
    
    Eigen::Matrix3d H = U * M.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    
    if (R.determinant() < 0) {
      Eigen::Matrix3d V = svd.matrixV();
      V.col(2) *= -1;
      R = V * svd.matrixU().transpose();
    }
    
    double s = 1.0;
    if (estimate_scale_) {
      double varU = 0.0;
      for (size_t i = 0; i < N; ++i) {
        varU += U.col(i).squaredNorm();
      }
      varU /= static_cast<double>(N);
      
      if (varU > 1e-12) {
        s = (svd.singularValues().sum()) / varU;
        estimated_scale_ = s;
      }
    }
    
    Eigen::Vector3d t = map_centroid - s * R * uwb_centroid;
    
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "uwb_world";
    
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    
    Eigen::Quaterniond q(R);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    
    return tf;
  }
  
  void check_calibration_progress()
  {
    if (!localization_available_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                           "RTAB-Map mapping 모드 - localization_pose 대기 중...");
    } else if (!calibration_complete_ && uwb_points_.size() > 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "SVD 정렬 진행 중: %zu/%d 대응쌍 수집됨", 
                  uwb_points_.size(), min_calibration_points_);
    } else if (calibration_complete_) {
      RCLCPP_INFO_ONCE(this->get_logger(), "3D 정렬 완료! UWB 좌표 자동 변환 중...");
    }
  }
  
  // 시리얼 읽기 함수들
  void read_serial() 
  {
    if (serial_fd_ < 0) {
      return;
    }
    
    char buf[256];
    int n = read(serial_fd_, buf, sizeof(buf) - 1);
    
    if (n > 0) {
      buf[n] = '\0';
      buffer_ += buf;
      process_buffer();
    }
  }
  
  void process_buffer() 
  {
    size_t pos = 0;
    while ((pos = buffer_.find('\n')) != std::string::npos) {
      std::string line = buffer_.substr(0, pos);
      buffer_.erase(0, pos + 1);
      process_line(line);
    }
  }
  
  void process_line(const std::string& line) 
  {
    std::string trimmed_line = line;
    trimmed_line.erase(0, trimmed_line.find_first_not_of(" \t\r\n"));
    trimmed_line.erase(trimmed_line.find_last_not_of(" \t\r\n") + 1);
    
    if (trimmed_line.empty()) {
      return;
    }
    
    if (trimmed_line.find("📥 X:") != std::string::npos) {
      try {
        std::regex x_regex("📥 X:\\s*(-?\\d+\\.\\d+)");
        std::smatch match;
        if (std::regex_search(trimmed_line, match, x_regex) && match.size() > 1) {
          x_value_ = std::stof(match[1].str());
          have_x_value_ = true;
        } else {
          size_t pos = trimmed_line.find("X:");
          if (pos != std::string::npos) {
            std::string value_str = trimmed_line.substr(pos + 2);
            x_value_ = std::stof(value_str);
            have_x_value_ = true;
          }
        }
      } catch (const std::exception& e) {
        have_x_value_ = false;
      }
    }
    else if (trimmed_line.find("📥 Y:") != std::string::npos) {
      try {
        std::regex y_regex("📥 Y:\\s*(-?\\d+\\.\\d+)");
        std::smatch match;
        if (std::regex_search(trimmed_line, match, y_regex) && match.size() > 1) {
          y_value_ = std::stof(match[1].str());
          have_y_value_ = true;
        } else {
          size_t pos = trimmed_line.find("Y:");
          if (pos != std::string::npos) {
            std::string value_str = trimmed_line.substr(pos + 2);
            y_value_ = std::stof(value_str);
            have_y_value_ = true;
          }
        }
      } catch (const std::exception& e) {
        have_y_value_ = false;
      }
    }
    else if (trimmed_line.find("📥 Z:") != std::string::npos) {
      try {
        std::regex z_regex("📥 Z:\\s*(-?\\d+\\.\\d+)");
        std::smatch match;
        if (std::regex_search(trimmed_line, match, z_regex) && match.size() > 1) {
          z_value_ = std::stof(match[1].str());
        } else {
          size_t pos = trimmed_line.find("Z:");
          if (pos != std::string::npos) {
            std::string value_str = trimmed_line.substr(pos + 2);
            z_value_ = std::stof(value_str);
          } else {
            return;
          }
        }
        
        if (have_x_value_ && have_y_value_) {
          process_uwb_data(x_value_, y_value_, z_value_);
          
          have_x_value_ = false;
          have_y_value_ = false;
        }
      } catch (const std::exception& e) {
        have_x_value_ = false;
        have_y_value_ = false;
      }
    }
  }
  
  // 멤버 변수들
  std::string port_name_;
  speed_t baud_rate_;
  int min_calibration_points_;
  double min_uwb_distance_;
  double min_map_distance_;
  int time_sync_threshold_ms_;
  bool estimate_scale_;
  double degenerate_threshold_;
  double uwb_scale_factor_;
  
  int serial_fd_;
  std::string buffer_;
  
  // ROS 관련 (2개 퍼블리셔만)
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;  // /bt_position_uwb
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr uwb_aligned_publisher_;  // /bt_position
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rtab_localization_subscriber_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;
  
  std::chrono::system_clock::time_point start_time_;
  std::chrono::system_clock::time_point last_data_time_;
  rclcpp::Time last_uwb_ros_time_;
  rclcpp::Time last_rtab_ros_time_;
  bool have_x_value_;
  bool have_y_value_;
  float x_value_;
  float y_value_;
  float z_value_;
  
  std::vector<Eigen::Vector3d> uwb_points_;
  std::vector<Eigen::Vector3d> map_points_;
  
  geometry_msgs::msg::PointStamped latest_uwb_;
  geometry_msgs::msg::Point last_rtab_position_;
  
  bool calibration_complete_ = false;
  bool tf_published_ = false;
  bool localization_available_ = false;
  bool has_latest_uwb_ = false;
  double estimated_scale_ = 1.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UWBReceiver>());
  rclcpp::shutdown();
  return 0;
}
