#include <chrono>
#include <memory>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>
#include <algorithm>
//#include <optional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

//#include <Eigen/Dense>
#pragma once


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

//interface package srv include...
#include "interfaces/srv/imu_reset.hpp"

//만든 메시지 타입
#include "iahrs_driver/msg/imu_array.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

//네비게이션
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace dynamixel;

#define PROTOCOL_VERSION      1.0
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_GOAL_VELOCITY    32
#define ADDR_PRESENT_POSITION 36
#define DXL1_ID               14              // Motor 1 ID Joint Pitch 106+ (몸통 1 관절)   잠시 바꿈 1, 14
#define DXL2_ID               24              // Motor 2 ID Joint Yaw   RX64+ (역방향?) - (머리 관절) - 안씀
#define DXL3_ID               4              // Motor 3 ID Screw - 몸통 1 스크류 - 원래 11
#define DXL4_ID               27              // Motor 4 ID Screw - 몸통 3 스크류 - 원래 27
#define DXL5_ID               28              // Motor 5 ID Joint Pitch  106+ 기준 뒤 모터(몸통 3 상하)  잠시 바꿈 21, 28
#define DXL6_ID               26              // Motor 6 ID Joint Yaw  106+ (정방향) - 기준 앞 모터(몸통 1 좌우)
#define DXL7_ID               12              // Motor 7 ID Joint Yaw  106+ (역방향) - 기준 뒤 모터(몸통 3 좌우)
#define DXL8_ID               13              // Motor 8 ID Screw  - 중간부 스크류 - 원래 13
#define DXL9_ID               5              // Motor 9 ID Joint Yaw  - (꼬리) - mx 64+?  - 임시 실험용 5 - 원래 25인데 그거 어디감?
#define BAUDRATE              1000000
#define DEVICE_NAME 		"/dev/motor"

#define SERIAL_PORT1	"/dev/ttyUSB1"
#define SERIAL_PORT2	"/dev/ttyUSB2"
#define SERIAL_SPEED	B115200

// Dynamixel SDK objects
PortHandler *portHandler;
PacketHandler *packetHandler;

typedef struct IMU_DATA
{
	double dQuaternion_x = 0.0;
	double dQuaternion_y = 0.0;
	double dQuaternion_z = 0.0;
	double dQuaternion_w = 1.0;

	double dAngular_velocity_x = 0.0;
	double dAngular_velocity_y = 0.0;
	double dAngular_velocity_z = 0.0;
	
	double dLinear_acceleration_x = 0.0;
	double dLinear_acceleration_y = 0.0;
	double dLinear_acceleration_z = 0.0;
    
	double dEuler_angle_Roll = 0.0;
	double dEuler_angle_Pitch = 0.0;
	double dEuler_angle_Yaw = 0.0;

}IMU_DATA;
IMU_DATA _pIMU_data_1, _pIMU_data_2;





int serial_fd_1 = -1;
int serial_fd_2 = -1;
double time_offset_in_seconds;
double dSend_Data[10];
double m_dRoll, m_dPitch, m_dYaw;
//single_used TF
bool m_bSingle_TF_option = true; //false;

using namespace std::chrono_literals;

std::atomic<bool> stop_requested(false);
void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}


//std::shared_ptr<IAHRS> node = nullptr;


// 최적 각도 찾아주는 코드래
static inline double normalize_angle(double a){
  while(a > M_PI)  a -= 2.0*M_PI;
  while(a < -M_PI) a += 2.0*M_PI;
  return a;
}

//mpc 관련 클래스
	class MPCController {
	public:
		std::pair<double, double> computeControls(const std::vector<double>& current_state, const nav_msgs::msg::Path& path) {
			// 실제로는 상태 예측 모델과 비용 함수 최적화를 통해 계산
			// 여기서는 개념을 보여주기 위한 가상 로직
			double target_v = 0.3; // 목표 선속도 (m/s)
			double target_w = 0.0; // 목표 각속도 (rad/s)

			if (!path.poses.empty()) {
				double dx = path.poses.front().pose.position.x - current_state[0];
				double dy = path.poses.front().pose.position.y - current_state[1];
				double dist_to_target = std::hypot(dx, dy);
				
				target_v = std::min(0.3, dist_to_target * 0.5); 
				
				double target_yaw = std::atan2(dy, dx);
				// 'normalize_angle' 함수가 이 클래스 외부에 있으므로 직접 호출
				double yaw_error = normalize_angle(target_yaw - current_state[2]);
				target_w = yaw_error * 1.5; 
			}
			return {target_v, target_w};
		}
};


//iahrs_driver class define
class IAHRS : public rclcpp::Node
{
public:


	struct ImuData {
    double roll_1;
    double pitch_1;
    double yaw_1;

    double roll_2;
    double pitch_2;
    double yaw_2;
	};

	ImuData imu_data;

	struct DRState2D {
	double vx{0.0}, vy{0.0};  // m/s
	double px{0.0}, py{0.0};  // m
	};
		
	// === IMU1/IMU2 DR 상태 ===
	DRState2D dr1_, dr2_;

	
	////value//////////////////////////////////////////////////////////////////////////////
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	geometry_msgs::msg::TransformStamped transformStamped_1;
	geometry_msgs::msg::TransformStamped transformStamped_2;
	geometry_msgs::msg::TransformStamped imu_transform_1, imu_transform_2;
	sensor_msgs::msg::Imu imu_data_msg;

	//  ImuArray 메시지 생성
	iahrs_driver::msg::ImuArray imu_array_msg;

	rclcpp::Publisher<iahrs_driver::msg::ImuArray>::SharedPtr imu_array_pub;
	rclcpp::Parameter m_bSingle_TF_option_param;

	std::vector<geometry_msgs::msg::Point> position_list;
	size_t current_index_ = 0;
	geometry_msgs::msg::PoseStamped current; // 현재 위치 저장용
	
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr bt_position_sub; //uwb 좌표값

	//카메라 안들어올때 uwb 값 가져오기
	std::vector<geometry_msgs::msg::PoseStamped> dynamic_position_list;
	rclcpp::Time last_camera_time;
	rclcpp::Duration timeout_threshold = rclcpp::Duration::from_seconds(1.0);

	//타이머 선언 및 변수처리
	rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;

	// 모든 타이머를 추적
	std::vector<rclcpp::TimerBase::SharedPtr> timers_;

	//nav 관련
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
	

	size_t current_index = 0;  // 현재 목표 좌표 인덱스

	//루프에서 타이머 실행 한 번만
	bool is_mode1_running = false;


	bool shutting_down_ = false; // 클래스 변수 추가



	bool link1_safe = true;
  	bool link2_safe = true;

	double to_hill_yaw_rad = 0.0;  // 로봇→언덕 방위각 (라디안)
	double yaw_error_rad   = 0.0;  // 회전해야 할 각(라디안)
	double dist_xy_m       = 0.0;  // 수평거리
	double dist_3d_m       = 0.0;  // 3D 거리


	

	IAHRS() : Node("iahrs_driver")
	{
		imu_array_msg.header.stamp = this->now();

		tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		//  메시지 타입 변경 (배열 기반)
        imu_array_pub = this->create_publisher<iahrs_driver::msg::ImuArray>("imu/data_combined", rclcpp::SensorDataQoS());

        //  서비스 생성 (IMU 데이터 리셋)
        euler_angle_reset_srv_ = create_service<interfaces::srv::ImuReset>(
            "all_data_reset",
            std::bind(&IAHRS::Euler_angle_reset_callback, this, std::placeholders::_1, std::placeholders::_2));

		
		 // 맵, pose 구독 (람다 콜백)-> pose 삭제 해도 ㄱㅊ 이미 카메라 sub 있음
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                mapCallback(msg);
            }
        );
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                poseCallback(msg);
            }
        );

		// hill_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    	// "/detected_hill_pose", 10,
    	// [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 			this->hillCallback(msg);
    	// });

		 // 파라미터로 토픽명 설정 (기본: /sand_hills, /robot_location)
    	declare_parameter<std::string>("hill_topic", "/sand_hills");
    	declare_parameter<std::string>("location_topic", "/rtabmap/localization_pose");

		get_parameter("hill_topic", hill_topic_);
		get_parameter("location_topic", location_topic_);

		RCLCPP_INFO(get_logger(), "Subscribing hill_topic   : %s", hill_topic_.c_str());
		RCLCPP_INFO(get_logger(), "Subscribing location_topic: %s", location_topic_.c_str());

		
		hill_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			hill_topic_, rclcpp::QoS(10),
			[this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
				// <<-- 수정: onHill 대신 hillCallback을 호출합니다. -->>
				this->hillCallback(msg); 
			});

		loc_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		location_topic_, rclcpp::QoS(10),
		[this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg){
			geometry_msgs::msg::PoseStamped ps;
			ps.header = msg->header;
			ps.pose   = msg->pose.pose;   // ← 위치/자세만 꺼내서
			this->onLocation(std::make_shared<geometry_msgs::msg::PoseStamped>(ps)); // ← 기존 흐름 재사용
		}
		);

        // 경로 퍼블리셔
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

		

		// 타이머 → 주기적으로 경로 업데이트
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { plannerLoop(); }
        );


		// TF2 설정
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


		// last_hill_update_time_을 아주 오래된 시간으로 초기화
    	last_hill_update_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

		

        //  파라미터 설정
        this->declare_parameter("m_bSingle_TF_option", false);
        m_bSingle_TF_option_param = this->get_parameter("m_bSingle_TF_option");
        m_bSingle_TF_option = m_bSingle_TF_option_param.as_bool();

		portHandler = PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
            rclcpp::shutdown();
            return;
        }

        if (!portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
            rclcpp::shutdown();
            return;
        }

      
        uint8_t dxl_error = 0;
        
        packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL6_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL7_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL8_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL9_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);

        RCLCPP_INFO(this->get_logger(), "Motor controller initialized.");

        
		//PoseStamped = 위치 + 자세 + 시간정보 
		aruco_posesub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/camera/aruco_pose", 10, 
			[this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
			  this->pose_callback(msg);
			});

			

			bt_position_sub = this->create_subscription<geometry_msgs::msg::Point>(
			  "/bt_position", 10,
			  [this](const geometry_msgs::msg::Point::SharedPtr msg) {
				this->bt_position_callback(*msg);
			  }
			);	

		rclcpp::on_shutdown([this]() {
        
        shutting_down_ = true;   // 반드시 가장 먼저!
        if (timer_) timer_->cancel();  // 타이머 즉시 중단 요청

		for (auto& t : timers_) {
                if (t) t->cancel();
            }

			timers_.clear();

        // 모든 모터 정지
        //stop_all_motors();
		stop_motors();

        // Torque disable
        uint8_t dxl_error = 0;
        packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL6_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL7_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL8_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);

        //안전하게 포트 닫기 전 잠시 대기
        //rclcpp::sleep_for(std::chrono::milliseconds(100));

		
        portHandler->closePort();

        
    	});	
       
	}
	~IAHRS()
	{
		// 1. 모든 모터 속도를 0으로 설정
    	stop_motors();

		// 안전을 위해 잠시 대기
    	rclcpp::sleep_for(std::chrono::milliseconds(100));

		uint8_t dxl_error = 0;
        packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL6_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL7_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL8_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        //packetHandler->write1ByteTxRx(portHandler, DXL9_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);

		
        portHandler->closePort();

	}


	////function//////////////////////////////////////////////////////////////////////////////
	int open_serial_1(const char* port)
    {
        RCLCPP_INFO(this->get_logger(), "Try to open serial: %s", port);

		
	

        serial_fd_1 = open(port, O_RDWR | O_NOCTTY);
        if (serial_fd_1 < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Unable to open serial port: %s", port);
            return -1;
        }else {
			RCLCPP_INFO(this->get_logger(), "!!Successfully opened serial port: %s", port);
		}

		//RCLCPP_INFO(this->get_logger(), "1Test [%d] serial %d", serial_fd_1, serial_fd_2);

        struct termios tio;
        tcgetattr(serial_fd_1, &tio);
        cfmakeraw(&tio);
        tio.c_cflag = CS8 | CLOCAL | CREAD;
        tio.c_iflag &= ~(IXON | IXOFF);
        cfsetspeed(&tio, SERIAL_SPEED);
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN] = 0;

        if (tcsetattr(serial_fd_1, TCSAFLUSH, &tio) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to configure serial port: %s", port);
            close(serial_fd_1);
            return -1;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully: %s", port);
        return 0;
    }

	int open_serial_2(const char* port)
    {
        RCLCPP_INFO(this->get_logger(), "Try to open serial: %s", port);

		
	

        serial_fd_2 = open(port, O_RDWR | O_NOCTTY);
        if (serial_fd_2 < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Unable to open serial port: %s", port);
            return -1;
        }else {
			RCLCPP_INFO(this->get_logger(), "!!Successfully opened serial port: %s", port);
		}

		
        struct termios tio;
        tcgetattr(serial_fd_2, &tio);
        cfmakeraw(&tio);
        tio.c_cflag = CS8 | CLOCAL | CREAD;
        tio.c_iflag &= ~(IXON | IXOFF);
        cfsetspeed(&tio, SERIAL_SPEED);
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN] = 0;

        if (tcsetattr(serial_fd_2, TCSAFLUSH, &tio) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to configure serial port: %s", port);
            close(serial_fd_2);
            return -1;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully: %s", port);
        return 0;
    }

	static unsigned long GetTickCount() 
	{
		struct timespec ts;
	
		clock_gettime (CLOCK_MONOTONIC, &ts);

		return ts.tv_sec*1000 + ts.tv_nsec/1000000;
	}

	
	int SendRecv(const char* command, double* returned_data, int data_length, int fd)
	{
		// 하나의 포트만 처리
		char temp_buff[256];
		read(fd, temp_buff, 256);  // 버퍼 비우기
		write(fd, command, strlen(command));

		#define COMM_RECV_TIMEOUT	30	

		const int buff_size = 1024;
		int recv_len = 0;
		char recv_buff[buff_size + 1] = {0};

		unsigned long time_start = GetTickCount();

		while (recv_len < buff_size) {
			int n = read(fd, recv_buff + recv_len, buff_size - recv_len);
			if (n < 0) return -1;
			else if (n == 0) usleep(1000);
			else {
				recv_len += n;
				if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n')
					break;
			}
			if (GetTickCount() - time_start >= COMM_RECV_TIMEOUT) return -1;
		}
		recv_buff[recv_len] = '\0';

		// 응답 파싱
		if (strncmp(command, recv_buff, strlen(command) - 1) == 0 && recv_buff[strlen(command) - 1] == '=') {
			int data_count = 0;
			char* p = &recv_buff[strlen(command)];
			char* pp = NULL;
			for (int i = 0; i < data_length; ++i) {
				if (p[0] == '0' && p[1] == 'x')
					returned_data[i] = strtol(p+2, &pp, 16);
				else
					returned_data[i] = strtod(p, &pp);
				data_count++;
				if (*pp == ',') p = pp + 1;
				else break;
			}
			return data_count;
		}
		RCLCPP_INFO(this->get_logger(), "Received [%s] from serial %d", recv_buff, fd);
		return 0;
	}

	// ── 추가: 부모 프레임 파라미터(기본 camera_link)
	std::string imu_parent_frame_ = "camera_link";

	// vslam은 map으로 보낼 가능성 높으니 나중에 map으로 바꿔야 한다?
	void publish_transforms_worlds()
	{

		// base_link_1을 world에 연결
		geometry_msgs::msg::TransformStamped transform_base_1;
		transform_base_1.header.stamp = rclcpp::Clock().now();
		transform_base_1.header.frame_id = "world";
		transform_base_1.child_frame_id = "base_link_1";
		transform_base_1.transform.translation.x = 0.0;
		transform_base_1.transform.translation.y = 0.0;
		transform_base_1.transform.translation.z = 0.0;
		transform_base_1.transform.rotation.w = 1.0;

		// base_link_2를 world에 연결
		geometry_msgs::msg::TransformStamped transform_base_2;
		transform_base_2.header.stamp = rclcpp::Clock().now();
		transform_base_2.header.frame_id = "world";
		transform_base_2.child_frame_id = "base_link_2";
		transform_base_2.transform.translation.x = 1.0;
		transform_base_2.transform.translation.y = 0.0;
		transform_base_2.transform.translation.z = 0.0;
		transform_base_2.transform.rotation.w = 1.0;

		//  TF 브로드캐스트 실행
		tf_broadcaster->sendTransform(transform_base_1);
		tf_broadcaster->sendTransform(transform_base_2);
		

	}

	void publish_transforms_1()
	{
		tf2::Quaternion q;
		q.setRPY(_pIMU_data_1.dEuler_angle_Roll , _pIMU_data_1.dEuler_angle_Pitch, _pIMU_data_1.dEuler_angle_Yaw);
		
		// Update the timestamp of the transform
		transformStamped_1.header.stamp = this->now();
		transformStamped_1.header.frame_id = imu_parent_frame_;   // Parent frame ID
		transformStamped_1.child_frame_id = "imu_link_1";       // IMU frame ID

		// Set the transformation translation (position)
		transformStamped_1.transform.translation.x = -0.20;
		transformStamped_1.transform.translation.y =  0.13;
		transformStamped_1.transform.translation.z =  0.00;


		
		transformStamped_1.transform.rotation.x = q.x();
		transformStamped_1.transform.rotation.y = q.y();
		transformStamped_1.transform.rotation.z = q.z();
		transformStamped_1.transform.rotation.w = q.w();


		// Publish the transform
		tf_broadcaster->sendTransform(transformStamped_1);
		

	}

	void publish_transforms_2()
	{
		tf2::Quaternion q;
		q.setRPY(_pIMU_data_2.dEuler_angle_Roll , _pIMU_data_2.dEuler_angle_Pitch, _pIMU_data_2.dEuler_angle_Yaw);
		
		// Update the timestamp of the transform
		transformStamped_2.header.stamp = this->now();
		transformStamped_2.header.frame_id = imu_parent_frame_;;   // Parent frame ID
		transformStamped_2.child_frame_id = "imu_link_2";       // IMU frame ID

		// Set the transformation translation (position)
		transformStamped_2.transform.translation.x = -0.20;
		transformStamped_2.transform.translation.y = -0.15;
		transformStamped_2.transform.translation.z =  0.00;

		
		transformStamped_2.transform.rotation.x = q.x();
		transformStamped_2.transform.rotation.y = q.y();
		transformStamped_2.transform.rotation.z = q.z();
		transformStamped_2.transform.rotation.w = q.w();


		// Publish the transform
		tf_broadcaster->sendTransform(transformStamped_2);
		

	}

	void imu_data_1(sensor_msgs::msg::Imu& imu_data_msg_1)
	{
		imu_data_msg_1.header.stamp = this->now();
		imu_data_msg_1.header.frame_id = "imu_1";

		if (serial_fd_1 >= 0)
		{
			const int max_data = 10;
			double data[max_data];
			int no_data = 0;

			no_data = SendRecv("g\n", data, max_data, serial_fd_1);  // Angular velocity
			if (no_data >= 3)
			{
				imu_data_msg_1.angular_velocity.x = _pIMU_data_1.dAngular_velocity_x = data[0] * (M_PI / 180.0);
				imu_data_msg_1.angular_velocity.y = _pIMU_data_1.dAngular_velocity_y = data[1] * (M_PI / 180.0);
				imu_data_msg_1.angular_velocity.z = _pIMU_data_1.dAngular_velocity_z = data[2] * (M_PI / 180.0);

			}

			no_data = SendRecv("a\n", data, max_data, serial_fd_1);  // Linear acceleration
			if (no_data >= 3)
			{
				imu_data_msg_1.linear_acceleration.x = _pIMU_data_1.dLinear_acceleration_x = data[0] * 9.80665;
				imu_data_msg_1.linear_acceleration.y = _pIMU_data_1.dLinear_acceleration_y = data[1] * 9.80665;
				imu_data_msg_1.linear_acceleration.z = _pIMU_data_1.dLinear_acceleration_z = data[2] * 9.80665;
			}

			no_data = SendRecv("e\n", data, max_data, serial_fd_1);  // Euler angles
			if (no_data >= 3)
			{
				_pIMU_data_1.dEuler_angle_Roll  = data[0] * (M_PI / 180.0);
				_pIMU_data_1.dEuler_angle_Pitch = data[1] * (M_PI / 180.0);
				_pIMU_data_1.dEuler_angle_Yaw   = data[2] * (M_PI / 180.0);
			}

			tf2::Quaternion q;
			q.setRPY(_pIMU_data_1.dEuler_angle_Roll,
					_pIMU_data_1.dEuler_angle_Pitch,
					_pIMU_data_1.dEuler_angle_Yaw);

			imu_data_msg_1.orientation.x = q.x();
			imu_data_msg_1.orientation.y = q.y();
			imu_data_msg_1.orientation.z = q.z();
			imu_data_msg_1.orientation.w = q.w();

			imu_array_msg.imus.push_back(imu_data_msg_1);

			
			if (m_bSingle_TF_option)
			{
				publish_transforms_1();
			}
		}
	}

	void imu_data_2(sensor_msgs::msg::Imu& imu_data_msg_2)
	{
		imu_data_msg_2.header.stamp = this->now();
		imu_data_msg_2.header.frame_id = "imu_2";

		if (serial_fd_2 >= 0) 
		{
			const int max_data = 10;
			double data[max_data];
			int no_data = 0;

			no_data = this->SendRecv("g\n", data, max_data, serial_fd_2);  // angular velocity
			if (no_data >= 3) 
			{
				imu_data_msg_2.angular_velocity.x = _pIMU_data_2.dAngular_velocity_x = data[0] * (M_PI / 180.0);
				imu_data_msg_2.angular_velocity.y = _pIMU_data_2.dAngular_velocity_y = data[1] * (M_PI / 180.0);
				imu_data_msg_2.angular_velocity.z = _pIMU_data_2.dAngular_velocity_z = data[2] * (M_PI / 180.0);
			}

			no_data = this->SendRecv("a\n", data, max_data, serial_fd_2);  // linear acceleration
			if (no_data >= 3) 
			{
				imu_data_msg_2.linear_acceleration.x = _pIMU_data_2.dLinear_acceleration_x = data[0] * 9.80665;
				imu_data_msg_2.linear_acceleration.y = _pIMU_data_2.dLinear_acceleration_y = data[1] * 9.80665;
				imu_data_msg_2.linear_acceleration.z = _pIMU_data_2.dLinear_acceleration_z = data[2] * 9.80665;
			}

			no_data = this->SendRecv("e\n", data, max_data, serial_fd_2);  // Euler angle
			if (no_data >= 3) 
			{
				_pIMU_data_2.dEuler_angle_Roll  = data[0] * (M_PI / 180.0);
				_pIMU_data_2.dEuler_angle_Pitch = data[1] * (M_PI / 180.0);
				_pIMU_data_2.dEuler_angle_Yaw   = data[2] * (M_PI / 180.0);
			}

			tf2::Quaternion q2;
			q2.setRPY(_pIMU_data_2.dEuler_angle_Roll, _pIMU_data_2.dEuler_angle_Pitch, _pIMU_data_2.dEuler_angle_Yaw);

			imu_data_msg_2.orientation.x = q2.x();
			imu_data_msg_2.orientation.y = q2.y();
			imu_data_msg_2.orientation.z = q2.z();
			imu_data_msg_2.orientation.w = q2.w();

			this->imu_array_msg.imus.push_back(imu_data_msg_2);

			
			if (m_bSingle_TF_option)
			{
				publish_transforms_2();
			}
		}
	}

	void getRPYFromImu(const sensor_msgs::msg::Imu& imu_msg, double& roll, double& pitch, double& yaw)
	{
		tf2::Quaternion q(
			imu_msg.orientation.x,
			imu_msg.orientation.y,
			imu_msg.orientation.z,
			imu_msg.orientation.w);

		tf2::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);  // 라디안 값

		//RCLCPP_INFO(this->get_logger(), "IMU Data Received - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll* 180.0 / M_PI, pitch* 180.0 / M_PI, yaw* 180.0 / M_PI);
	}

		double saved_x = 0.0;
		double saved_y = 0.0;
		double saved_z = 0.0;

		double x_ = 0.0;
		double y_ = 0.0;
		double z_ = 0.0;

	//카메라 좌표	
	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
	{
		// 콜백 내에서 멤버 변수에 저장
		
		
		x_ = msg->pose.position.x;
		y_ = msg->pose.position.y;
		z_ = msg->pose.position.z;

		save_pose(x_,y_,z_);
		last_camera_time = this->now();

		RCLCPP_INFO(this->get_logger(), " Received pose: (%.2f, %.2f, %.2f)", x_, y_, z_);
	}

	void save_pose(double x, double y, double z)
	{
		saved_x = x;
		saved_y = y;
		saved_z = z;
	}

	//tolerance 가 단위가 m 라는 얘기가 있네요 - gpt라 확실하진 않음
	bool has_arrived(const geometry_msgs::msg::PoseStamped& current, const geometry_msgs::msg::PoseStamped& target, double tolerance = 0.3)
	{
		return hypot(current.pose.position.x - target.pose.position.x, current.pose.position.y - target.pose.position.y) <= tolerance;
	}

	double x_uwb = 0.0;
	double y_uwb = 0.0;
	double z_uwb = 0.0;

	void bt_position_callback(const geometry_msgs::msg::Point& msg)
	{
		x_uwb = msg.x;
		y_uwb = msg.y;
		z_uwb = msg.z;

		RCLCPP_INFO(this->get_logger(), "UWB 위치 업데이트됨: x=%.2f, y=%.2f, z=%.2f", x_uwb, y_uwb, z_uwb);
	}

	void position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		current = *msg;
	}

	//uwb 좌표 저장
	void save_position(const geometry_msgs::msg::Point& p)
	{
		geometry_msgs::msg::Point temp = p;
		// temp.z 유지 (나중에 쓸 수도 있음)
		position_list.push_back(temp);
	}

	void dynamic_position_update()
	{
		geometry_msgs::msg::PoseStamped pose_msg;
		rclcpp::Time now = this->now();

		pose_msg.header.stamp = now;
		pose_msg.header.frame_id = "map";  // 필요시 수정

		if ((now - last_camera_time) < timeout_threshold) {
			// 카메라 좌표 사용
			pose_msg.pose.position.x = x_;
			pose_msg.pose.position.y = y_;
			pose_msg.pose.position.z = z_;
			// orientation도 카메라 Pose가 있다면 저장 가능
			pose_msg.pose.orientation.x = 0.0;
			pose_msg.pose.orientation.y = 0.0;
			pose_msg.pose.orientation.z = 0.0;
			pose_msg.pose.orientation.w = 1.0;
			RCLCPP_INFO(this->get_logger(), "카메라 좌표 저장");
		} 
		else {
			// UWB 좌표 사용 → orientation은 기본값
			pose_msg.pose.position.x = x_uwb;
			pose_msg.pose.position.y = y_uwb;
			pose_msg.pose.position.z = z_uwb;
			pose_msg.pose.orientation.x = 0.0;
			pose_msg.pose.orientation.y = 0.0;
			pose_msg.pose.orientation.z = 0.0;
			pose_msg.pose.orientation.w = 1.0;
			RCLCPP_WARN(this->get_logger(), "카메라 신호 없음 → UWB 좌표 저장");
    	}

		// PoseStamped로 통일된 리스트에 저장
		dynamic_position_list.push_back(pose_msg);
	}


	// imu 속도 계산
	//  IMU1의 수평 가속(ax, ay), yaw(rad), dt[s]를 받아
	//  vx,vy,px,py를 적분 갱신한다. 
	void update_imu_dr_1(double ax_body, double ay_body, double yaw_rad, double dt)
	{
		if (dt <= 0.0) return;

		// 소거(노이즈 억제) — 필요 없으면 0.0로 낮춰도 됨
		const double acc_thresh = 0.005;
		if (std::fabs(ax_body) < acc_thresh) ax_body = 0.0;
		if (std::fabs(ay_body) < acc_thresh) ay_body = 0.0;

		// 바디→월드(수평) 회전 (yaw만 사용)
		const double cy = std::cos(yaw_rad), sy = std::sin(yaw_rad);
		const double ax_w =  cy*ax_body - sy*ay_body;
		const double ay_w =  sy*ax_body + cy*ay_body;

		// 적분
		dr1_.vx += ax_w * dt;
		dr1_.vy += ay_w * dt;

		// 간단 정지판정(ZUPT 흉내): 각속/수평가속 아주 작으면 속도 0으로
		const double gyro_n = std::sqrt(
			_pIMU_data_1.dAngular_velocity_x*_pIMU_data_1.dAngular_velocity_x +
			_pIMU_data_1.dAngular_velocity_y*_pIMU_data_1.dAngular_velocity_y +
			_pIMU_data_1.dAngular_velocity_z*_pIMU_data_1.dAngular_velocity_z);
		const double horiz_a = std::sqrt(ax_body*ax_body + ay_body*ay_body);
		if (gyro_n < 0.02 && horiz_a < 0.05) { dr1_.vx = 0.0; dr1_.vy = 0.0; }

		dr1_.px += dr1_.vx * dt;
		dr1_.py += dr1_.vy * dt;
	}

	// ─────────────────────────────────────────────
	// [무슨 함수?] IMU2용 동일 로직 (필요할 때만 호출)
	void update_imu_dr_2(double ax_body, double ay_body, double yaw_rad, double dt)
	{
		if (dt <= 0.0) return;

		const double acc_thresh = 0.15;
		if (std::fabs(ax_body) < acc_thresh) ax_body = 0.0;
		if (std::fabs(ay_body) < acc_thresh) ay_body = 0.0;

		const double cy = std::cos(yaw_rad), sy = std::sin(yaw_rad);
		const double ax_w =  cy*ax_body - sy*ay_body;
		const double ay_w =  sy*ax_body + cy*ay_body;

		dr2_.vx += ax_w * dt;
		dr2_.vy += ay_w * dt;

		const double gyro_n = std::sqrt(
			_pIMU_data_2.dAngular_velocity_x*_pIMU_data_2.dAngular_velocity_x +
			_pIMU_data_2.dAngular_velocity_y*_pIMU_data_2.dAngular_velocity_y +
			_pIMU_data_2.dAngular_velocity_z*_pIMU_data_2.dAngular_velocity_z);
		const double horiz_a = std::sqrt(ax_body*ax_body + ay_body*ay_body);
		if (gyro_n < 0.02 && horiz_a < 0.05) { dr2_.vx = 0.0; dr2_.vy = 0.0; }

		dr2_.px += dr2_.vx * dt;
		dr2_.py += dr2_.vy * dt;
	}

	// ─────────────────────────────────────────────
	// [무슨 함수?] DR 상태 리셋
	void reset_imu_dr_1(){ dr1_ = DRState2D{}; }
	void reset_imu_dr_2(){ dr2_ = DRState2D{}; }

	// ─────────────────────────────────────────────
	// [무슨 함수?] DR 상태 조회
	void get_imu_dr_1(double& px, double& py, double& vx, double& vy) const
	{ px = dr1_.px; py = dr1_.py; vx = dr1_.vx; vy = dr1_.vy; }

	void get_imu_dr_2(double& px, double& py, double& vx, double& vy) const
	{ px = dr2_.px; py = dr2_.py; vx = dr2_.vx; vy = dr2_.vy; }



	//imu 기준으로 찾은 거라 수정필수
	void hill_detector(double roll_deg_1, double pitch_deg_1, double yaw_deg_1, double roll_deg_2, double pitch_deg_2, double yaw_deg_2)

	{
		//이거 맞냐? 조건 수정하긴 해야 할 듯?
		if (abs(abs(roll_deg_1) - abs(roll_deg_2)) > 30)

		{

		dynamic_position_update();

		motor3_speed = basic_speed;

		motor4_speed = 0;

		motor8_speed = basic_speed;

		}

	}

	void printHillAndRobotOnce()
	{
		if (!have_last_hill_) {
			RCLCPP_WARN(get_logger(), "[HILL] 아직 수신 없음");
			return;
		}
		if (!have_last_loc_) {
			RCLCPP_WARN(get_logger(), "[ROBOT] 위치 수신 없음 (location/topic 확인)");
			return;
		}

		const auto &h = last_hill_;
		const auto &r = last_loc_;

		const double dx = h.pose.position.x - r.pose.position.x;
		const double dy = h.pose.position.y - r.pose.position.y;
		const double dz = h.pose.position.z - r.pose.position.z;
		const double dxy = std::hypot(dx, dy);

		RCLCPP_INFO(get_logger(),
			"[HILL] x=%.3f y=%.3f z=%.3f  frame=%s  stamp=%.3f\n"
			"[ROBOT] x=%.3f y=%.3f z=%.3f  frame=%s  stamp=%.3f\n"
			"[Δ] dx=%.3f dy=%.3f dz=%.3f  dist_xy=%.3f m",
			h.pose.position.x, h.pose.position.y, h.pose.position.z,
			h.header.frame_id.c_str(), rclcpp::Time(h.header.stamp).seconds(),
			r.pose.position.x, r.pose.position.y, r.pose.position.z,
			r.header.frame_id.c_str(), rclcpp::Time(r.header.stamp).seconds(),
			dx, dy, dz, dxy);
	}



	bool hill_dig()
	{

		 // ---- 내부 상태 정의 ----
		enum DigPhase {
			START,
			FORWARD_1,
			TURN_1,
			FORWARD_2,
			TURN_2,
			FORWARD_3,
			DONE
		};

		// static 변수를 사용하여 함수가 다시 호출되어도 이전 상태를 기억하도록 함
		static DigPhase current_phase = START;
		static geometry_msgs::msg::PoseStamped start_pose;
		static double start_yaw = 0.0;

		// 현재 로봇의 위치와 방향
		const auto& current_loc = last_loc_; 
		double current_yaw = tf2::getYaw(current_loc.pose.orientation);

		// 상태에 따라 다른 동작 수행
		switch (current_phase) {
			case START:
				RCLCPP_INFO(get_logger(), "[DIG] 언덕 파기 시작! 1단계: 전진 (5cm)");
				start_pose = current_loc; // 현재 위치를 시작점으로 기록
				current_phase = FORWARD_1;
				break;

			case FORWARD_1:
				// 목표: 5cm 전진
				move_forward_mode2(0.1, 0.0, _pIMU_data_1.dEuler_angle_Pitch, _pIMU_data_2.dEuler_angle_Pitch); // 저속(0.1m/s) 직진
				{
					double dist_moved = std::hypot(current_loc.pose.position.x - start_pose.pose.position.x, 
												current_loc.pose.position.y - start_pose.pose.position.y);
					if (dist_moved >= 0.05) {
						RCLCPP_INFO(get_logger(), "[DIG] 1단계 완료. 2단계: 좌회전 (10도)");
						stop_motors();
						start_yaw = current_yaw; // 현재 방향을 회전 시작점으로 기록
						current_phase = TURN_1;
					}
				}
				break;

			case TURN_1:
				// 목표: +10도 좌회전
				turn_left_mode2();
				{
					double angle_turned = normalize_angle(current_yaw - start_yaw);
					if (angle_turned >= (10.0 * M_PI / 180.0)) {
						RCLCPP_INFO(get_logger(), "[DIG] 2단계 완료. 3단계: 전진 (5cm)");
						stop_motors();
						start_pose = current_loc; // 다시 현재 위치를 시작점으로 기록
						current_phase = FORWARD_2;
					}
				}
				break;

			case FORWARD_2:
				// 목표: 5cm 추가 전진
				move_forward_mode2(0.1, 0.0, _pIMU_data_1.dEuler_angle_Pitch, _pIMU_data_2.dEuler_angle_Pitch);
				{
					double dist_moved = std::hypot(current_loc.pose.position.x - start_pose.pose.position.x, 
												current_loc.pose.position.y - start_pose.pose.position.y);
					if (dist_moved >= 0.05) {
						RCLCPP_INFO(get_logger(), "[DIG] 3단계 완료. 4단계: 좌회전 (10도)");
						stop_motors();
						start_yaw = current_yaw;
						current_phase = TURN_2;
					}
				}
				break;

			case TURN_2:
				// 목표: +10도 추가 좌회전
				turn_left_mode2();
				{
					double angle_turned = normalize_angle(current_yaw - start_yaw);
					if (angle_turned >= (10.0 * M_PI / 180.0)) {
						RCLCPP_INFO(get_logger(), "[DIG] 4단계 완료. 5단계: 전진 (7cm)");
						stop_motors();
						start_pose = current_loc;
						current_phase = FORWARD_3;
					}
				}
				break;

			case FORWARD_3:
				// 목표: 7cm 추가 전진
				move_forward_mode2(0.1, 0.0, _pIMU_data_1.dEuler_angle_Pitch, _pIMU_data_2.dEuler_angle_Pitch);
				{
					double dist_moved = std::hypot(current_loc.pose.position.x - start_pose.pose.position.x, 
												current_loc.pose.position.y - start_pose.pose.position.y);
					if (dist_moved >= 0.07) {
						RCLCPP_INFO(get_logger(), "[DIG] 5단계 완료. 언덕 파기 동작 종료.");
						stop_motors();
						current_phase = DONE;
					}
				}
				break;

			case DONE:
				current_phase = START; // 다음 호출을 위해 상태 초기화
				return true; // 동작 완료 신호
		}

		return false; // 아직 동작 진행 중


	}

	
	void hill_destroy() 
	{
		
		// ================== 1. 목표 선정 및 관리 ==================
		if (!is_target_locked_) {
			geometry_msgs::msg::PoseStamped next_hill;
			if (findNextHill(next_hill)) {
				locked_hill_target_ = next_hill;
				is_target_locked_ = true;
				RCLCPP_INFO(get_logger(), ">> 다음 목표 고정! (%.2f, %.2f)", 
							locked_hill_target_.pose.position.x, locked_hill_target_.pose.position.y);
			} else {
				RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "모든 언덕 파괴 완료. 대기 중...");
				stop_motors();
				return;
			}
		}

		// ================== 2. 상태 계산 ==================

		// ---- 제어 파라미터 및 상태 변수 (하나의 선언만 남김) ----
		const double YAW_OK          = 12.0 * M_PI / 180.0;
		const int    YAW_OK_STREAK_N = 6;
		const double REACH_OK        = 0.07;
		const int    REACH_OK_STREAK_N = 3;
		
		// <<-- 수정: DIGGING 상태를 포함한 enum 선언 하나만 남깁니다. -->>
		enum Phase { TURNING, FORWARD, DIGGING }; 
		static Phase phase = TURNING;
		static int yaw_ok_streak = 0;
		static int reach_ok_streak = 0;

		// 고정된 목표를 기준으로 현재 로봇과의 거리, 각도 등을 계산
		tryProcessWithTarget(locked_hill_target_);

		// ---- 상태 전환 조건 계산 ----
		// 도착 판정: 도착 반경에 연속으로 들어왔는지 확인
		if (dist_xy_m <= REACH_OK) ++reach_ok_streak; else reach_ok_streak = 0;
		// 정렬 판정: 정렬 각도에 연속으로 들어왔는지 확인
		if (std::fabs(yaw_error_rad) <= YAW_OK) ++yaw_ok_streak; else yaw_ok_streak = 0;

		// ================== 3. 상태에 따른 행동 결정 (하나의 `if-else if` 블록으로 통합) ==================

		if (phase == TURNING) {
			// 목표: 정렬 완료하기
			if (yaw_ok_streak < YAW_OK_STREAK_N) {
				// 아직 정렬 안됨 -> 계속 회전
				if (yaw_error_rad > 0.0) turn_left_mode2();
				else turn_right_mode2();
			} else {
				// 정렬 완료 -> FORWARD 상태로 전환
				stop_motors();
				RCLCPP_INFO(get_logger(), "정렬 완료 -> 전진 시작");
				phase = FORWARD;
			}
		}
		else if (phase == FORWARD) {
			// 목표: 목표 지점에 도착하기
			// 도착했는지 먼저 확인
			if (reach_ok_streak >= REACH_OK_STREAK_N) {
				RCLCPP_INFO(get_logger(), ">> 목표 도달! 언덕 파기 상태로 전환합니다.");
				stop_motors();
				phase = DIGGING; // 파기 상태로 전환
				return; // 즉시 다음 루프로 넘어가 DIGGING 시작
			}

			// 전진 중 정렬이 깨졌는지 확인
			if (yaw_ok_streak < YAW_OK_STREAK_N / 2) {
				stop_motors();
				RCLCPP_INFO(get_logger(), "재정렬 중...");
				phase = TURNING; // 재정렬을 위해 TURNING 상태로 복귀
				return;
			}
			
			// 정렬도 잘 되어있고 아직 도착 전 -> 계속 전진
			// --- MPC 기반 전진 로직 ---
			double current_linear_velocity = std::hypot(dr1_.vx, dr1_.vy);
			double robot_yaw_rad = tf2::getYaw(last_loc_.pose.orientation);
			double current_angular_velocity = _pIMU_data_1.dAngular_velocity_z;
			
			std::vector<double> current_state = { 
				last_loc_.pose.position.x, last_loc_.pose.position.y, robot_yaw_rad, 
				current_linear_velocity, current_angular_velocity 
			};
			
			nav_msgs::msg::Path target_path;
			target_path.poses.push_back(locked_hill_target_); 

			auto [optimal_v, optimal_w] = mpc_controller_->computeControls(current_state, target_path);

			move_forward_mode2(optimal_v, optimal_w, _pIMU_data_1.dEuler_angle_Pitch, _pIMU_data_2.dEuler_angle_Pitch);
		}
		else if (phase == DIGGING) {
			// 목표: hill_dig() 동작 완료하기
			if (hill_dig()) { // hill_dig()가 true를 반환하면 동작이 모두 끝났다는 의미
				// 임무 완료 처리
				RCLCPP_INFO(get_logger(), ">> 언덕 파괴 완료. 다음 목표를 탐색합니다.");
				markHillDestroyed(locked_hill_target_); 
				is_target_locked_ = false;               
				
				// 모든 상태를 초기화하여 다음 미션을 준비
				phase = TURNING; 
				yaw_ok_streak = 0; 
				reach_ok_streak = 0;
				
				rclcpp::sleep_for(std::chrono::seconds(2));
			}
			// hill_dig()가 false를 반환하면, 아직 동작 중이므로 아무것도 하지 않고 다음 루프를 기다림
		}
	}

	//카메라에서 좌표계를 가져와야 함 그러려면 카메라 좌표계를 뜯어봐야 하고 그걸 imu랑 합치기도 해야 하고 절대 좌표는 vslam하면서 카메라가 잡을 꺼니까 우린 이제 절대좌표가 필요 없어짐
	double initial_yaw_deg = 0.0;
	bool yaw_initialized = false;
	double current_yaw_deg = 0.0;

	// 처음 시작 시 yaw 값 저장 - > 아래 두 개로 대체됨
	void update_yaw_from_imu(double yaw_1_rad) 
	{
		current_yaw_deg = yaw_1_rad * 180.0 / M_PI;

		if (!yaw_initialized) {
			initial_yaw_deg = current_yaw_deg;
			yaw_initialized = true;
			RCLCPP_INFO(this->get_logger(), "초기 yaw 설정됨: %.2f deg", initial_yaw_deg);
		}
		RCLCPP_INFO(this->get_logger(), "current_yaw_deg = %.2f deg", current_yaw_deg);
	}


	// yaw 최초 초기화용
	void init_initial_yaw(double yaw_1_rad) {
		if (!yaw_initialized) {
			initial_yaw_deg = yaw_1_rad * 180.0 / M_PI;
			yaw_initialized = true;
			RCLCPP_INFO(this->get_logger(), "✅ 초기 yaw 설정됨: %.2f deg", initial_yaw_deg);
		}
	}

	// 현재 yaw 업데이트용 (매번 갱신만)
	void update_current_yaw(double yaw_1_rad) {
		current_yaw_deg = yaw_1_rad * 180.0 / M_PI;
		RCLCPP_INFO(this->get_logger(), "현재 yaw 값 갱신: %.2f deg", current_yaw_deg);
	}

	//카메라 생기면 다시
	void move_toward_target(double target_x, double target_y, double current_x, double current_y)
	{
		double dx = target_x - current_x;
		double dy = target_y - current_y;

		// 목표 지점까지의 절대 방향
		double target_yaw_rad = std::atan2(dy, dx);
		double target_yaw_deg = target_yaw_rad * 180.0 / M_PI;

		// 로봇이 현재 바라보는 방향 (초기 yaw 기준 상대값)
		double relative_yaw_deg = current_yaw_deg - initial_yaw_deg;

		// 목표 방향과의 차이 계산
		double yaw_error = target_yaw_deg - relative_yaw_deg;

		// -180~180 정규화
		while (yaw_error > 180) yaw_error -= 360;
		while (yaw_error < -180) yaw_error += 360;

		// 방향 판단 + 몇 초 회전 했을 때 얼마 회전하는 지 확인 해야 할 듯(표준화..?)
		if (std::abs(yaw_error) > 15.0) {
			if (yaw_error > 0)
			RCLCPP_INFO(this->get_logger(), " 왼쪽으로 회전 (%.1f도)", yaw_error);
			else
			RCLCPP_INFO(this->get_logger(), " 오른쪽으로 회전 (%.1f도)", yaw_error);
		} else {
			RCLCPP_INFO(this->get_logger(), " 전진 방향 정렬됨 → 전진");
		}
	}


	//모드 전환 모터 제어 없음
	void check_wall_collision(int16_t motor6_pos, int16_t motor7_pos,
		double x, double y,
		double& out_link1_x, double& out_link1_y,
		double& out_link2_x, double& out_link2_y)
	{
		const int16_t motor_center = 2048;
		const double angle_per_unit = 180.0 / 2048.0;
		const double link_length = 40.0;
		const double camera_to_link1 = 50.0;
		const double camera_to_link2 = 75.0;
		
		double theta6_deg = 0.0;
		double theta7_deg = 0.0;
		
		if (motor6_pos < motor_center && motor7_pos > motor_center) {
		theta6_deg = (motor_center - motor6_pos) * angle_per_unit;
		theta7_deg = (motor_center - motor7_pos) * angle_per_unit;
		} else if (motor6_pos > motor_center && motor7_pos < motor_center) {
		theta6_deg = (motor6_pos - motor_center) * angle_per_unit;
		theta7_deg = (motor7_pos - motor_center) * angle_per_unit;
		}
		
		double theta6_rad = theta6_deg * M_PI / 180.0;
		double theta7_rad = theta7_deg * M_PI / 180.0;
		
		double delta_L1 = link_length * std::cos(theta6_rad);
		double delta_L2 = link_length * std::cos(theta7_rad);
		double delta_W1 = link_length * std::sin(theta6_rad);
		double delta_W2 = link_length * std::sin(theta7_rad);
		
		out_link1_x = x + camera_to_link1 + delta_L1;
		out_link1_y = y + delta_W1;
		out_link2_x = x - camera_to_link2 - delta_L2;
		out_link2_y = y + delta_W2;
		
		// 멤버 변수 업데이트 (this-> 생략 가능)
		link1_safe = (out_link1_x >= 0 && out_link1_x <= 145) &&
		(out_link1_y >= 0 && out_link1_y <= 120);
		
		link2_safe = (out_link2_x >= 0 && out_link2_x <= 145) &&
		(out_link2_y >= 0 && out_link2_y <= 120);
		
		// 로깅도 함수 안에서 처리
		if (!link1_safe || !link2_safe) {
		RCLCPP_WARN(this->get_logger(), "벽 간섭 발생: 회피 필요");
		} else {
		RCLCPP_INFO(this->get_logger(), " 안전: 벽 간섭 없음");
		}
	}

	void initialize_positions() 	//좌표값 왜 이럼, 꼭짓점 가는 건가 이걸로
	{
		geometry_msgs::msg::PoseStamped p;

		p.pose.position.x = 0.0; p.pose.position.y = 0.0; dynamic_position_list.push_back(p);
		p.pose.position.x = 0.30; p.pose.position.y = 0.90; dynamic_position_list.push_back(p);
		p.pose.position.x = 0.90; p.pose.position.y = 0.90; dynamic_position_list.push_back(p);
		p.pose.position.x = 0.90; p.pose.position.y = 0.30; dynamic_position_list.push_back(p);

		current_index = 0;  // 시작 위치 설정
	}

	void move_to_next_position(const geometry_msgs::msg::PoseStamped current_position)
	{
		// 목표 좌표 리스트 끝에 도달한 경우 처리
		if (current_index >= dynamic_position_list.size()) {
			motor3_speed = 0;
			motor4_speed = 0;
			motor8_speed = 0;
			RCLCPP_INFO(this->get_logger(), " 모든 목표 좌표에 도달함");
			return;
		}

		// 현재 목표 좌표 설정
		geometry_msgs::msg::PoseStamped target_position = dynamic_position_list[current_index];

		// 목표 좌표 도달 여부 확인
		if (has_arrived(current_position, target_position)) {
			RCLCPP_INFO(this->get_logger(), " 목표 좌표 (%.2f, %.2f)에 도달. 다음 좌표로 이동", 
						target_position.pose.position.x, target_position.pose.position.y);
			current_index++;  // 다음 목표로 인덱스 증가
		} else {
			// 목표 좌표로 이동 중
			RCLCPP_INFO(this->get_logger(), " 이동 중: 현재 목표 → (%.2f, %.2f)",
						target_position.pose.position.x, target_position.pose.position.y);

			// 여기서 목표 좌표를 향한 모터 제어 코드 추가 가능
			// motor3_speed = /* 원하는 이동속도 */;
			// motor4_speed = /* 원하는 이동속도 */;
			// motor8_speed = /* 원하는 이동속도 */;
		}
	}



	int mode_state = 1;
	int mode_change = 1;

	int16_t basic_speed = 300;        // 기본 속도
	int16_t basic_speed_inverse = 1324; // 역방향

	int16_t motor1_position = 2048;
	int16_t motor2_position;
	int16_t motor5_position = 2048;
	int16_t motor6_position;
	int16_t motor7_position;
	int16_t motor9_position;

	int16_t motor3_speed;
	int16_t motor4_speed;
	int16_t motor8_speed;


	// 여기서 지정하면 값이 바뀌었을 때 아래 함수에서 변경이 안되는 타이밍 있을 수도 -> 애초에 안 바뀔 예정이라 일단 진행
	double target_pitch;
    double target_yaw;
    double target_roll;        

    // IMU 값 저장 변수
    double saved_pitch = 0.00, saved_yaw = 0.00, saved_roll = 0.00;

    double prev_error_pitch_1 = 0.0, prev_error_yaw_1 = 0.0, prev_error_roll_1 = 0.0;
    double integral_pitch_1 = 0.0, integral_yaw_1 = 0.0, integral_roll_1 = 0.0;

	double prev_error_pitch_2 = 0.0, prev_error_yaw_2 = 0.0, prev_error_roll_2 = 0.0;
    double integral_pitch_2 = 0.0, integral_yaw_2 = 0.0, integral_roll_2 = 0.0;

    // double Kp_pitch = 150.0, Ki_pitch = 10.0, Kd_pitch = 5.0;
    // double Kp_yaw = 100.0, Ki_yaw = 10.0, Kd_yaw = 5.0;
    // double Kp_roll = 150.0, Ki_roll = 3, Kd_roll = 10.0;

    // // Motor2와 Motor6의 범위를 맞추기 위해 Motor6 PID 계수 보정
    // double Kp_yaw_106 = Kp_yaw * (4095.0 / 1023.0) * 1.5;
    // double Ki_yaw_106 = Ki_yaw * (4095.0 / 1023.0) * 0.1;
    // double Kd_yaw_106 = Kd_yaw * (4095.0 / 1023.0) * 5;

	// PID 계수 (가중치를 yaw1, yaw2 별도로 관리 가능)
	double Kp_pitch_1 = 150.0, Ki_pitch_1 = 10.0, Kd_pitch_1 = 5.0;
	double Kp_roll_1 = 150.0, Ki_roll_1 = 3, Kd_roll_1 = 10.0;
	double Kp_yaw_1 = 100.0, Ki_yaw_1 = 10.0, Kd_yaw_1 = 5.0;
	double Kp_yaw_106_1 = Kp_yaw_1 * (4095.0 / 1023.0) * 1.5;
	double Ki_yaw_106_1 = Ki_yaw_1 * (4095.0 / 1023.0) * 0.1;
	double Kd_yaw_106_1 = Kd_yaw_1 * (4095.0 / 1023.0) * 5;

	double Kp_pitch_2 = 150.0, Ki_pitch_2 = 10.0, Kd_pitch_2 = 5.0;
	double Kp_roll_2 = 150.0, Ki_roll_2 = 3, Kd_roll_2 = 10.0;
	double Kp_yaw_2 = 100.0, Ki_yaw_2 = 10.0, Kd_yaw_2 = 5.0;
	double Kp_yaw_106_2 = Kp_yaw_2 * (4095.0 / 1023.0) * 1.5;
	double Ki_yaw_106_2 = Ki_yaw_2 * (4095.0 / 1023.0) * 0.1;
	double Kd_yaw_106_2 = Kd_yaw_2 * (4095.0 / 1023.0) * 5;


	int16_t initial_motor1_position = 2048 + static_cast<int16_t>((target_pitch / M_PI) * 2048);
    int16_t initial_motor2_position = 512 - static_cast<int16_t>((target_yaw / (5 * M_PI / 3)) * 511); // 300° 최대치 고려
    int16_t initial_motor5_position = 2048 + static_cast<int16_t>((target_pitch / M_PI) * 2048);
    int16_t initial_motor6_position = 2048 - static_cast<int16_t>((target_yaw / M_PI) * 2048);
    int16_t initial_motor7_position = 2048 + static_cast<int16_t>((target_yaw / M_PI) * 2048);
    //int16_t initial_motor8_position = 2048 - static_cast<int16_t>((target_pitch / M_PI) * 2048);

	
	void reset_pid_state()
    {
        integral_pitch_1 = 0.0;
        integral_yaw_1   = 0.0;
        integral_roll_1  = 0.0;

        prev_error_pitch_1 = 0.0;
        prev_error_yaw_1   = 0.0;
        prev_error_roll_1  = 0.0;

		integral_pitch_2 = 0.0;
        integral_yaw_2   = 0.0;
        integral_roll_2  = 0.0;

        prev_error_pitch_2 = 0.0;
        prev_error_yaw_2   = 0.0;
        prev_error_roll_2  = 0.0;
    }

	bool is_within_bounds(double x, double y, double max_x = 0.9, double max_y = 0.7) {
    return (x <= max_x && y <= max_y);
	}

	int forward_state = 1;
	// 전진 동작 (roll 중심 PID 사용)
	void move_Right_mode1(double roll_1, double pitch_1, double yaw_1,
                   double roll_2, double pitch_2, double yaw_2) {
		double control_roll_1 = compute_pid(target_roll, roll_1, integral_roll_1, prev_error_roll_1,
											Kp_roll_1, Ki_roll_1, Kd_roll_1, 1.0);
		double control_roll_2 = compute_pid(target_roll, roll_2, integral_roll_2, prev_error_roll_2,
											Kp_roll_2, Ki_roll_2, Kd_roll_2, 1.0);

		if(forward_state == 1){

            motor3_speed = basic_speed_inverse + 200 - static_cast<int16_t>(control_roll_1 * 0.1);
			motor4_speed = 0;
			motor8_speed = basic_speed_inverse + 100 - static_cast<int16_t>(control_roll_1 * 0.1);

            //forward_state = 0;

        }
        else
        {
            motor3_speed = 0;
			motor4_speed = basic_speed_inverse - static_cast<int16_t>(control_roll_1 * 0.1);
			motor8_speed = basic_speed_inverse + 100 - static_cast<int16_t>(control_roll_1 * 0.1);

            //forward_state = 1;
        }

		//원본 참고용으로 남김
		// motor3_speed = basic_speed_inverse + 200 - static_cast<int16_t>(control_roll_1 * 0.1);
		// motor4_speed = basic_speed_inverse - static_cast<int16_t>(control_roll_1 * 0.1);
		// motor8_speed = basic_speed_inverse + 100 - static_cast<int16_t>(control_roll_1 * 0.1);

		set_motor_speeds(motor3_speed, motor4_speed, motor8_speed);
	}

	void move_Left_mode1(double roll_1, double pitch_1, double yaw_1,
                   double roll_2, double pitch_2, double yaw_2) {
		double control_roll_1 = compute_pid(target_roll, roll_1, integral_roll_1, prev_error_roll_1,
											Kp_roll_1, Ki_roll_1, Kd_roll_1, 1.0);
		double control_roll_2 = compute_pid(target_roll, roll_2, integral_roll_2, prev_error_roll_2,
											Kp_roll_2, Ki_roll_2, Kd_roll_2, 1.0);

		if(forward_state == 1){

            motor3_speed = basic_speed + 200 - static_cast<int16_t>(control_roll_1 * 0.1);
			motor4_speed = 0;
			motor8_speed = basic_speed + 100 - static_cast<int16_t>(control_roll_1 * 0.1);

            //forward_state = 0;

        }
        else
        {
            motor3_speed = 0;
			motor4_speed = basic_speed - static_cast<int16_t>(control_roll_1 * 0.1);
			motor8_speed = basic_speed + 100 - static_cast<int16_t>(control_roll_1 * 0.1);

            //forward_state = 1;
        }

		//원본 참고용으로 남김
		// motor3_speed = basic_speed_inverse + 200 - static_cast<int16_t>(control_roll_1 * 0.1);
		// motor4_speed = basic_speed_inverse - static_cast<int16_t>(control_roll_1 * 0.1);
		// motor8_speed = basic_speed_inverse + 100 - static_cast<int16_t>(control_roll_1 * 0.1);

		set_motor_speeds(motor3_speed, motor4_speed, motor8_speed);
	}

	void execute_mode1(double saved_x, double saved_y) {

		if (is_mode1_running) return; 
		is_mode1_running = true;

		init_initial_yaw(imu_data.yaw_1);  // 초기 yaw 설정
		
		counter_ = 0;
		forward_state = 1;

		move_Right_mode1(imu_data.roll_1, imu_data.pitch_1, imu_data.yaw_1,
						imu_data.roll_2, imu_data.pitch_2, imu_data.yaw_2);
		
			auto t1 = this->create_wall_timer(
				std::chrono::milliseconds(100),
				[this]() {
					if (shutting_down_) return; // 종료 중이면 타이머 아무 동작도 안함

					if (counter_ < 20) {
						counter_++;
						if (shutting_down_) return;
						RCLCPP_INFO(this->get_logger(), " yaw 값: %.2f", imu_data.yaw_1 * 180.0 / M_PI);
					} else {
						//timer_->cancel();
						counter_ = 0;

						forward_state = 0;
						move_Right_mode1(imu_data.roll_1, imu_data.pitch_1, imu_data.yaw_1,
										imu_data.roll_2, imu_data.pitch_2, imu_data.yaw_2);

						// ✅ t1은 더 이상 필요 없으므로 cancel
						// for (auto& t : timers_) if (t) t->cancel();
						// timers_.clear();

						if (shutting_down_) return;

						auto t2 = this->create_wall_timer(
							std::chrono::milliseconds(100),
							[this]() {
								if (shutting_down_) return; // 종료 중이면 타이머 아무 동작도 안함
								move_Right_mode1(imu_data.roll_1, imu_data.pitch_1, imu_data.yaw_1,
												imu_data.roll_2, imu_data.pitch_2, imu_data.yaw_2);

								update_current_yaw(imu_data.yaw_1);

								if (shutting_down_) return;

								double yaw_error = current_yaw_deg - initial_yaw_deg;
								while (yaw_error > 180) yaw_error -= 360;
								while (yaw_error < -180) yaw_error += 360;

								if (shutting_down_) return;

								if (abs(yaw_error) < 5.0) {
									for (auto& t : timers_) if (t) t->cancel();
									timers_.clear();
									//stop_motors();
									is_mode1_running = false;
									if (shutting_down_) return;
									RCLCPP_INFO(this->get_logger(), "✅ 초기 yaw로 복귀 완료 → 정지");
								}
							}
						);
						timers_.push_back(t2);
					}
				}
			);
			timers_.push_back(t1);
	}

	// --- 추가: 튜닝이 필요한 새로운 상수 정의 ---

	// basic_speed(1000)가 대략 어느 정도의 전진 속도(m/s)에 해당하는지 정의합니다.
	// 이 값은 실제 로봇을 주행시켜보고 측정해야 하는 중요한 튜닝 값입니다.
	#define NEUTRAL_FORWARD_VELOCITY 0.02  // 예: basic_speed가 0.2 m/s에 해당한다고 가정

	// 속도(m/s)를 모터 제어값으로 변환하기 위한 변환 계수
	#define VELOCITY_TO_MOTOR_SPEED 1000.0




	// 모터 동작 기본 함수 (전진, 좌회전, 우회전, 정지)
    void move_forward_mode2(double target_linear_velocity, double target_angular_velocity,
                        double pitch_1, double pitch_2) {

		double target_yaw_rate = target_angular_velocity;
    	double current_yaw_rate = _pIMU_data_1.dAngular_velocity_z;

			// PID 계산
		double pitch_control_1 = compute_pid(target_pitch, pitch_1, integral_pitch_1, prev_error_pitch_1,
											Kp_pitch_1, Ki_pitch_1, Kd_pitch_1, 0.6);
		double pitch_control_2 = compute_pid(target_pitch, pitch_2, integral_pitch_2, prev_error_pitch_2,
											Kp_pitch_2, Ki_pitch_2, Kd_pitch_2, 0.625);
		double yaw_control_1 = compute_pid(target_yaw_rate, current_yaw_rate, integral_yaw_1, prev_error_yaw_1,
										Kp_yaw_106_1, Ki_yaw_106_1, Kd_yaw_106_1, 4095.0 / 1023.0);
		double yaw_control_2 = compute_pid(target_yaw_rate, current_yaw_rate, integral_yaw_2, prev_error_yaw_2,
										Kp_yaw_106_2, Ki_yaw_106_2, Kd_yaw_106_2, 4095.0 / 1023.0);

		// 위치 모터 계산
		motor1_position = initial_motor1_position + static_cast<int16_t>(pitch_control_1);
		motor5_position = initial_motor5_position + static_cast<int16_t>(pitch_control_2);
		motor6_position = initial_motor6_position - static_cast<int16_t>(yaw_control_1);
		motor7_position = initial_motor7_position + static_cast<int16_t>(yaw_control_2);


		// 3. 선속도(Linear Velocity) 보정값 계산
		// MPC의 목표 속도와 로봇의 평균 속도(NEUTRAL_FORWARD_VELOCITY)의 '차이'를 계산
		double velocity_error = target_linear_velocity - NEUTRAL_FORWARD_VELOCITY;
		
		// 이 속도 차이를 모터에 인가할 '추가적인' 속도 보정값으로 변환
		int16_t linear_velocity_adjustment = static_cast<int16_t>(velocity_error * VELOCITY_TO_MOTOR_SPEED);

		// 속도 모터 계산
		motor3_speed = basic_speed + linear_velocity_adjustment + 200 - static_cast<int16_t>(yaw_control_1 * 0.1);
		motor4_speed = basic_speed_inverse - linear_velocity_adjustment + 70 + static_cast<int16_t>(yaw_control_2 * 0.1);
		motor8_speed = basic_speed_inverse - linear_velocity_adjustment + 200;

		// 모터 제어(관절 제어 수정)
		//set_motor_positions(motor1_position, motor5_position, motor6_position, motor7_position);
		set_motor_speeds(motor3_speed, motor4_speed, motor8_speed);
    }

    void turn_left_mode2() {
        set_motor_speeds(basic_speed_inverse + 20, basic_speed_inverse, basic_speed_inverse + 20);
    }

    void turn_right_mode2() {
		set_motor_speeds(basic_speed + 50, basic_speed, basic_speed_inverse + 20);
    }

    void stop_motors() {
        set_motor_speeds(0, 0, 0);
    }

	
	// PID 제어 계산만 따로 빼놓은 함수(연산 공통 함수)
    double compute_pid(double target, double current, double& integral, double& prev_error,
                    double Kp, double Ki, double Kd, double scale_factor = 1.0) {
        double error = target - current;
        integral += error;
        double derivative = error - prev_error;
        prev_error = error;

        return (Kp * error + Ki * integral + Kd * derivative) * scale_factor;
    }

    // 모터 속도를 직접 설정하는 함수 (기존 코드 중복 줄임)
    void set_motor_speeds(int16_t speed3, int16_t speed4, int16_t speed8) {
        uint8_t dxl_error = 0;

		RCLCPP_INFO(this->get_logger(), "Motor3 Speed: %d, Motor4 Speed: %d , Motor8 Speed : %d", speed3, speed4 , speed8);

        packetHandler->write2ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_VELOCITY, speed3, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_GOAL_VELOCITY, speed4, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL8_ID, ADDR_GOAL_VELOCITY, speed8, &dxl_error);
    }
	//모터 각도 직접 설정하는 함수 (기존 코드 중복 줄이기)
	void set_motor_positions(int16_t pos1, int16_t pos5, int16_t pos6, int16_t pos7)
	{
		uint8_t dxl_error = 0;

		packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, pos1, &dxl_error);
		packetHandler->write2ByteTxRx(portHandler, DXL5_ID, ADDR_GOAL_POSITION, pos5, &dxl_error);
		packetHandler->write2ByteTxRx(portHandler, DXL6_ID, ADDR_GOAL_POSITION, pos6, &dxl_error);
		packetHandler->write2ByteTxRx(portHandler, DXL7_ID, ADDR_GOAL_POSITION, pos7, &dxl_error);
	}


    // void pid_control_mode2(double pitch_1, double pitch_2, double yaw_1, double yaw_2) {
    
    //     // pitch 제어 추가됨
    //     double pitch_control_1 = compute_pid(target_pitch, pitch_1, integral_pitch_1, prev_error_pitch_1,
    //                                         Kp_pitch_1, Ki_pitch_1, Kd_pitch_1, 0.6);

    //     double pitch_control_2 = compute_pid(target_pitch, pitch_2, integral_pitch_2, prev_error_pitch_2,
    //                                         Kp_pitch_2, Ki_pitch_2, Kd_pitch_2, 0.625);

    //     // yaw 제어도 yaw1, yaw2 각각 분리됨
    //     double yaw_control_1 = compute_pid(target_yaw, yaw_1, integral_yaw_1, prev_error_yaw_1,
    //                                     Kp_yaw_106_1, Ki_yaw_106_1, Kd_yaw_106_1, (4095.0 / 1023.0));

    //     double yaw_control_2 = compute_pid(target_yaw, yaw_2, integral_yaw_2, prev_error_yaw_2,
    //                                     Kp_yaw_106_2, Ki_yaw_106_2, Kd_yaw_106_2, (4095.0 / 1023.0));

    //     // 최종 전진 명령 수행 (pitch 제어값도 모터 위치에 적용 가능하도록 확장 가능)
    //     move_forward_mode2(yaw_control_1, yaw_control_2);
        
    //     // 모터 1, 5 (pitch 모터) PID 제어값으로 위치 제어
    //     uint8_t dxl_error = 0;
    //     motor1_position = initial_motor1_position + static_cast<int16_t>(pitch_control_1);
    //     motor5_position = initial_motor5_position + static_cast<int16_t>(pitch_control_2);

    //     packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, motor1_position, &dxl_error);
    //     packetHandler->write2ByteTxRx(portHandler, DXL5_ID, ADDR_GOAL_POSITION, motor5_position, &dxl_error);
    // }


    // MODE2 전체 흐름을 정리한 최상위 동작 함수
    void execute_mode2(double roll_1, double pitch_1, double yaw_1,
                   double roll_2, double pitch_2, double yaw_2,
                   double saved_x, double saved_y) {

		double roll_deg_1 = roll_1 * 180.0 / M_PI;
		double roll_deg_2 = roll_2 * 180.0 / M_PI;
		double yaw_deg_1 = yaw_1 * 180.0 / M_PI;
		double yaw_deg_2 = yaw_2 * 180.0 / M_PI;
		double pitch_deg_1 = pitch_1 * 180.0 / M_PI;
		double pitch_deg_2 = pitch_2 * 180.0 / M_PI;
		

		 if (!is_within_bounds(saved_x, saved_y)) {
        stop_motors();
        RCLCPP_WARN(this->get_logger(), " 위치 제한 초과 - 동작 정지 (x=%.2f, y=%.2f)", saved_x, saved_y);
        return;
    	}

        if (yaw_deg_1 < -25.0 || mode_state == 0 || saved_y > 0.68) {
            RCLCPP_INFO(this->get_logger(), "@@@@@@@go back!!!!!!!!!!!");
            turn_left_mode2();
            mode_state = 0;
            reset_pid_state();
        }
        else if (yaw_deg_1 > 0.0 || (saved_y > 0.50 && saved_y < 0.65)) {
            RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@@@@@right on target!!!!!!!!!!!!");
            mode_state = 1;
            move_forward_mode2(pitch_1, pitch_2, yaw_1, yaw_2);  // 전진
        }
        else {
            move_forward_mode2(pitch_1, pitch_2, yaw_1, yaw_2);  // 전진
        }

        // 특정 조건에서 정지 예시 (추가 가능)
        if (saved_x > 0.9) {
            stop_motors();
            RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@@@@@arrived!!!!!!!!!!!!");
        }
    }
	

	void MODE1(double roll_1, double pitch_1, double yaw_1, double roll_2, double pitch_2, double yaw_2)
{
	
	target_pitch = 0.0;

    double roll_deg_1 = roll_1 * 180.0 / M_PI;
	double pitch_deg_1 = pitch_1 * 180.0 / M_PI;
	double yaw_deg_1 = yaw_1 * 180.0 / M_PI;

	double roll_deg_2 = roll_2 * 180.0 / M_PI;
	double pitch_deg_2 = pitch_2 * 180.0 / M_PI;
	double yaw_deg_2 = yaw_2 * 180.0 / M_PI;

	RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!Using pose: (%.2f, %.2f, %.2f)", saved_x, saved_y, saved_z);
	

	RCLCPP_INFO(this->get_logger(), "IMU_1 Data Received - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll_deg_1, pitch_deg_1, yaw_deg_1);
	RCLCPP_INFO(this->get_logger(), "IMU_2 Data Received - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll_deg_2, pitch_deg_2, yaw_deg_2);
    
	//target position
	initial_motor1_position = 2048;
	initial_motor5_position = 2048;
	//initial_motor6_position = 3000;
    //initial_motor7_position = 1100;
    //motor9_position = 2500;

	double error_pitch_1 = target_pitch - pitch_1;
	double error_yaw_1 = target_yaw - yaw_1;
	double error_roll_1 = target_roll - roll_1;

	double error_pitch_2 = target_pitch - pitch_2;
	double error_yaw_2 = target_yaw - yaw_2;
	double error_roll_2 = target_roll - roll_2;

	// 2) 적분, 미분
	integral_pitch_1 += error_pitch_1;
	integral_yaw_1   += error_yaw_1;
	integral_roll_1  += error_roll_1;

	integral_pitch_2 += error_pitch_2;
	integral_yaw_2   += error_yaw_2;
	integral_roll_2  += error_roll_2;

	double derivative_pitch_1 = error_pitch_1 - prev_error_pitch_1;
	double derivative_yaw_1   = error_yaw_1   - prev_error_yaw_1;
	double derivative_roll_1  = error_roll_1  - prev_error_roll_1;

	double derivative_pitch_2 = error_pitch_2 - prev_error_pitch_2;
	double derivative_yaw_2   = error_yaw_2   - prev_error_yaw_2;
	double derivative_roll_2  = error_roll_2  - prev_error_roll_2;

	// Scale factor 추가
	double scale_factor = 5.0;  // PID 출력 신호를 10배로 확대

	
	prev_error_pitch_1 = error_pitch_1;
	prev_error_yaw_1 = error_yaw_1;
	prev_error_roll_1 = error_roll_1;

	prev_error_pitch_2 = error_pitch_2;
	prev_error_yaw_2 = error_yaw_2;
	prev_error_roll_2 = error_roll_2;

	// 3) PID 제어 식
	double control_signal_pitch_1 = (Kp_pitch_1*error_pitch_1 + Ki_pitch_1*integral_pitch_1 + Kd_pitch_1*derivative_pitch_1) * scale_factor /8 ;
	double control_signal_yaw_1   = (Kp_yaw_1  *error_yaw_1   + Ki_yaw_1  *integral_yaw_1   + Kd_yaw_1  *derivative_yaw_1);
	double control_signal_roll_1  = (Kp_roll_1 *error_roll_1  + Ki_roll_1 *integral_roll_1 + Kd_roll_1 *(derivative_roll_1)) ;
	double control_signal_yaw_106_1 = (Kp_yaw_106_1  *error_yaw_1   + Ki_yaw_106_1  *integral_yaw_1   + Kd_yaw_106_1  *derivative_yaw_1) * (4095.0 / 1023.0);

	double control_signal_pitch_2 = (Kp_pitch_2*error_pitch_2 + Ki_pitch_2*integral_pitch_2 + Kd_pitch_2*derivative_pitch_2) * scale_factor /8 ;
	double control_signal_yaw_2   = (Kp_yaw_2  *error_yaw_2   + Ki_yaw_2  *integral_yaw_2   + Kd_yaw_2  *derivative_yaw_2);
	double control_signal_roll_2  = (Kp_roll_2 *error_roll_2  + Ki_roll_2 *integral_roll_2 + Kd_roll_2 *(derivative_roll_2)) ;
	double control_signal_yaw_106_2 = (Kp_yaw_106_2  *error_yaw_2   + Ki_yaw_106_2  *integral_yaw_2   + Kd_yaw_106_2  *derivative_yaw_2) * (4095.0 / 1023.0);


	motor3_speed = basic_speed_inverse + 200 - static_cast<int16_t>(control_signal_roll_1 * 0.1);
    motor4_speed = basic_speed_inverse  - static_cast<int16_t>(control_signal_roll_1 * 0.1); // 역방향
	motor8_speed = basic_speed_inverse + 100 - static_cast<int16_t>(control_signal_roll_1 * 0.1);
	

    //motor5_position = initial_motor5_position + static_cast<int16_t>(control_signal_pitch);
	motor1_position = initial_motor1_position - static_cast<int16_t>(control_signal_pitch_1);
    motor5_position = initial_motor5_position + static_cast<int16_t>(control_signal_pitch_2);
    //motor6_position = initial_motor6_position - static_cast<int16_t>(control_signal_yaw_106_1);
    //motor7_position = initial_motor7_position + static_cast<int16_t>(control_signal_yaw_106_2);	

	
	//걍 좦표값 받아서 수정하는 걸로 ㄱ 근데 PID 제어값 리셋은 해줘야 함 언젠가
	//음 넘어짐 그리고 몸통 1이 토크 과부하 걸리고 몸통 3은 왜 위로 올라가냐
	 if (roll_deg_1 < - 30.0 || mode_state == 0 )
	 {
	 	RCLCPP_INFO(this->get_logger(), "!!!!!stand up!!!!!!!!!!!");
		
		//motor1_position = 2048;
		//motor5_position = 2048;
		
	 	motor3_speed = basic_speed;
	 	motor4_speed = basic_speed;
	 	motor8_speed = basic_speed;
		
	 	mode_state = 0;

	 	reset_pid_state();
		
	 }
	
	if( /*yaw_deg_1 < - 5.0 &&*/ yaw_deg_1 > -15.0){
	 	RCLCPP_INFO(this->get_logger(), "right on target!!!!!!!!!!!!");
	 	mode_state = 1;
	 }


	// if (saved_y < 0.30)
	// {
	// 	motor3_speed = 0;
	//  	motor4_speed = 0;
	//  	motor8_speed = 0;
	// }

	uint8_t dxl_error = 0;
	//packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, motor2_position, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, motor1_position, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, DXL5_ID, ADDR_GOAL_POSITION, motor5_position, &dxl_error);
	//packetHandler->write2ByteTxRx(portHandler, DXL6_ID, ADDR_GOAL_POSITION, motor6_position, &dxl_error);
	//packetHandler->write2ByteTxRx(portHandler, DXL7_ID, ADDR_GOAL_POSITION, motor7_position, &dxl_error);
	//packetHandler->write2ByteTxRx(portHandler, DXL9_ID, ADDR_GOAL_POSITION, motor9_position, &dxl_error);

	packetHandler->write2ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_VELOCITY, motor3_speed, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_GOAL_VELOCITY, motor4_speed, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, DXL8_ID, ADDR_GOAL_VELOCITY, motor8_speed, &dxl_error);
        
	RCLCPP_INFO(this->get_logger(), "Motor1 Position: %d, Motor5 Position: %d ",
										motor1_position, motor5_position);
	RCLCPP_INFO(this->get_logger(), "Motor6 Position: %d, Motor7 Position: %d ",
										motor6_position, motor7_position);

	RCLCPP_INFO(this->get_logger(), "Motor3 Speed: %d, Motor4 Speed: %d , Motor8 Speed : %d", motor3_speed, motor4_speed , motor8_speed);

	hill_detector(roll_deg_1, pitch_deg_1, yaw_deg_1, roll_deg_2, pitch_deg_2, yaw_deg_2);

	// 최초 한 번 초기화
	initialize_positions();

	// 주기적으로 호출 (예: timer 콜백 내부)
	geometry_msgs::msg::Point current_position;
	// current_position을 카메라나 UWB에서 받아와야 함
	current_position.x = saved_x;
	current_position.y = saved_y;

	//move_to_next_position(current_position);

	//나중에 모드 변환 코드 + 충돌 코드(연결) + 반대로 이동 코드 넣어야 함
	if (saved_y < 0.30) { // 특정 조건 만족 시 mode 변경 예시
        motor3_speed = 0;
        motor4_speed = 0;
        motor8_speed = 0;

        mode_change = 2; // MODE2로 전환
        reset_pid_state(); // PID 상태 초기화
        RCLCPP_INFO(this->get_logger(), "MODE1 완료 → MODE2로 전환");
    }


}

void MODE2(double roll_1, double pitch_1, double yaw_1, double roll_2, double pitch_2, double yaw_2)
{
	
	target_pitch = 0.0;

	RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!Using pose: (%.2f, %.2f, %.2f)", saved_x, saved_y, saved_z);

    double roll_deg_1 = roll_1 * 180.0 / M_PI;
	double pitch_deg_1 = pitch_1 * 180.0 / M_PI;
	double yaw_deg_1 = yaw_1 * 180.0 / M_PI;

	double roll_deg_2 = roll_2 * 180.0 / M_PI;
	double pitch_deg_2 = pitch_2 * 180.0 / M_PI;
	double yaw_deg_2 = yaw_2 * 180.0 / M_PI;

	RCLCPP_INFO(this->get_logger(), "IMU_1 Data Received - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll_deg_1, pitch_deg_1, yaw_deg_1);
	RCLCPP_INFO(this->get_logger(), "IMU_2 Data Received - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll_deg_2, pitch_deg_2, yaw_deg_2);
    
	//target position
	initial_motor1_position = 2048;
	initial_motor5_position = 2048;
	initial_motor6_position = 3000;
    initial_motor7_position = 1100;
    //motor9_position = 2500;

	double error_pitch_1 = target_pitch - pitch_1;
	double error_yaw_1 = target_yaw - yaw_1;
	double error_roll_1 = target_roll - roll_1;

	double error_pitch_2 = target_pitch - pitch_2;
	double error_yaw_2 = target_yaw - yaw_2;
	double error_roll_2 = target_roll - roll_2;

	// 2) 적분, 미분
	integral_pitch_1 += error_pitch_1;
	integral_yaw_1   += error_yaw_1;
	integral_roll_1  += error_roll_1;

	integral_pitch_2 += error_pitch_2;
	integral_yaw_2   += error_yaw_2;
	integral_roll_2  += error_roll_2;

	double derivative_pitch_1 = error_pitch_1 - prev_error_pitch_1;
	double derivative_yaw_1   = error_yaw_1   - prev_error_yaw_1;
	double derivative_roll_1  = error_roll_1  - prev_error_roll_1;

	double derivative_pitch_2 = error_pitch_2 - prev_error_pitch_2;
	double derivative_yaw_2   = error_yaw_2   - prev_error_yaw_2;
	double derivative_roll_2  = error_roll_2  - prev_error_roll_2;

	// Scale factor 추가
	double scale_factor = 5.0;  // PID 출력 신호를 10배로 확대
	
	prev_error_pitch_1 = error_pitch_1;
	prev_error_yaw_1 = error_yaw_1;
	prev_error_roll_1 = error_roll_1;

	prev_error_pitch_2 = error_pitch_2;
	prev_error_yaw_2 = error_yaw_2;
	prev_error_roll_2 = error_roll_2;

	// 3) PID 제어 식
	double control_signal_pitch_1 = (Kp_pitch_1*error_pitch_1 + Ki_pitch_1*integral_pitch_1 + Kd_pitch_1*derivative_pitch_1) * 0.6 ;
	double control_signal_yaw_1   = (Kp_yaw_1  *error_yaw_1   + Ki_yaw_1  *integral_yaw_1   + Kd_yaw_1  *derivative_yaw_1);
	double control_signal_roll_1  = (Kp_roll_1 *error_roll_1  + Ki_roll_1 *integral_roll_1 + Kd_roll_1 *(derivative_roll_1)) ;
	double control_signal_yaw_106_1 = (Kp_yaw_106_1  *error_yaw_1   + Ki_yaw_106_1  *integral_yaw_1   + Kd_yaw_106_1  *derivative_yaw_1) * (4095.0 / 1023.0);

	double control_signal_pitch_2 = (Kp_pitch_2*error_pitch_2 + Ki_pitch_2*integral_pitch_2 + Kd_pitch_2*derivative_pitch_2) * scale_factor /8 ;
	double control_signal_yaw_2   = (Kp_yaw_2  *error_yaw_2   + Ki_yaw_2  *integral_yaw_2   + Kd_yaw_2  *derivative_yaw_2);
	double control_signal_roll_2  = (Kp_roll_2 *error_roll_2  + Ki_roll_2 *integral_roll_2 + Kd_roll_2 *(derivative_roll_2)) ;
	double control_signal_yaw_106_2 = (Kp_yaw_106_2  *error_yaw_2   + Ki_yaw_106_2  *integral_yaw_2   + Kd_yaw_106_2  *derivative_yaw_2) * (4095.0 / 1023.0);


	motor3_speed = basic_speed_inverse  - static_cast<int16_t>(control_signal_yaw_106_1 * 0.1);
    motor4_speed = basic_speed + 120 + static_cast<int16_t>(control_signal_yaw_106_2 * 0.1); // 역방향
	//motor8_speed = basic_speed - static_cast<int16_t>(control_signal_yaw_2);
	// motor3_speed = basic_speed_inverse;
	// motor4_speed = basic_speed;
	 motor8_speed = (motor3_speed + (motor4_speed - 1024) )/2 - 50;

    //motor5_position = initial_motor5_position + static_cast<int16_t>(control_signal_pitch);
	motor1_position = initial_motor1_position + static_cast<int16_t>(control_signal_pitch_1);
    motor5_position = initial_motor5_position + static_cast<int16_t>(control_signal_pitch_2);
    //motor6_position = initial_motor6_position - static_cast<int16_t>(control_signal_yaw_106_1);
    //motor7_position = initial_motor7_position + static_cast<int16_t>(control_signal_yaw_106_2);	

	
	//걍 좦표값 받아서 수정하는 걸로 ㄱ 근데 PID 제어값 리셋은 해줘야 함 언젠가
	 if (yaw_deg_1 < - 25.0 || mode_state == 0  || saved_y > 0.68)
	 {
	 	RCLCPP_INFO(this->get_logger(), "@@@@@@@go back!!!!!!!!!!!");
		
		motor1_position = 2048;
		motor5_position = 2048;
		
	 	motor3_speed = basic_speed;
	 	motor4_speed = basic_speed_inverse;
	 	motor8_speed = 0;
		
	 	mode_state = 0;

	 	reset_pid_state();
		
	 }
	
	if( /*yaw_deg_1 < - 5.0 &&*/ yaw_deg_1 > 0.0 || (saved_y > 0.50 && saved_y < 0.65)){
	 	RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@@@@@right on target!!!!!!!!!!!!");
	 	mode_state = 1;
	 }

	 if (saved_x > 0.9)
	 {
		motor3_speed = 0;
	 	motor4_speed = 0;
	 	motor8_speed = 0;

		RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@@@@@arrived!!!!!!!!!!!!");

	 }

	uint8_t dxl_error = 0;
	//packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, motor2_position, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, motor1_position, &dxl_error);
	packetHandler->write2ByteTxRx(portHandler, DXL5_ID, ADDR_GOAL_POSITION, motor5_position, &dxl_error);
	//packetHandler->write2ByteTxRx(portHandler, DXL6_ID, ADDR_GOAL_POSITION, motor6_position, &dxl_error);
	//packetHandler->write2ByteTxRx(portHandler, DXL7_ID, ADDR_GOAL_POSITION, motor7_position, &dxl_error);
	//packetHandler->write2ByteTxRx(portHandler, DXL9_ID, ADDR_GOAL_POSITION, motor9_position, &dxl_error);

	packetHandler->write2ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_VELOCITY, motor3_speed, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, DXL4_ID, ADDR_GOAL_VELOCITY, motor4_speed, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, DXL8_ID, ADDR_GOAL_VELOCITY, motor8_speed, &dxl_error);
        
	RCLCPP_INFO(this->get_logger(), "Motor1 Position: %d, Motor5 Position: %d ",
										motor1_position, motor5_position);
	RCLCPP_INFO(this->get_logger(), "Motor6 Position: %d, Motor7 Position: %d ",
										motor6_position, motor7_position);

	RCLCPP_INFO(this->get_logger(), "Motor3 Speed: %d, Motor4 Speed: %d , Motor8 Speed : %d", motor3_speed, motor4_speed , motor8_speed);


	// 최초 한 번 초기화
	initialize_positions();

	// 주기적으로 호출 (예: timer 콜백 내부)
	geometry_msgs::msg::PoseStamped current_position;
	// current_position을 카메라나 UWB에서 받아와야 함
	current_position.pose.position.x = saved_x;
	current_position.pose.position.y = saved_y;

	move_to_next_position(current_position);
	 

	//hill_destroy();


}




private:

	rclcpp::Service<interfaces::srv::ImuReset>::SharedPtr euler_angle_reset_srv_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_posesub;

	bool Euler_angle_reset_callback(
		const std::shared_ptr<interfaces::srv::ImuReset::Request> request, 
		const std::shared_ptr<interfaces::srv::ImuReset::Response> response)
	{
		bool bResult = false;
		double dSend_Data[10];
		SendRecv("ra\n", dSend_Data, 10, serial_fd_1);
		SendRecv("ra\n", dSend_Data, 10, serial_fd_2);
		bResult = true;
		response->result = bResult;
		return true;
	}

	// 위치 저장용 멤버 변수

	
	 

	std::vector<geometry_msgs::msg::PoseStamped> hills_;  // 언덕 위치 저장
    std::vector<bool> hills_destroyed_;                   // 언덕 파괴 여부 기록


	//nav
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
	//rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

	//rclcpp::TimerBase::SharedPtr timer_;

    // 상태 저장
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool map_received_;
    bool pose_received_;

    // visited_map 저장 (coverage 기록)
    std::vector<int> visited_map_;

	//new 카메라

	std::string hill_topic_;
	std::string location_topic_;

	// 언덕 위치 구독, 로봇 위치 구독
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hill_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr loc_sub_;

	geometry_msgs::msg::PoseStamped last_hill_;
	geometry_msgs::msg::PoseStamped last_loc_;
	bool have_last_hill_ = false;
	bool have_last_loc_  = false;

	rclcpp::Time last_hill_update_time_;
	bool has_active_hill_target_ = false; // 언덕 목표가 유효한지 여부 플래그

	// --- 타겟 고정(Target Locking)을 위한 변수 ---
    bool is_target_locked_ = false; // 현재 목표물이 고정되었는지 여부
    geometry_msgs::msg::PoseStamped locked_hill_target_; // 고정된 언덕 목표 정보

	 // TF2 관련
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


	// MPC 컨트롤러
    std::unique_ptr<MPCController> mpc_controller_;

	
	void onHill(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
	{
		last_hill_ = *msg;
		have_last_hill_ = true;
		
		// <<-- 수정: 언덕 정보가 들어올 때마다 상태와 시간을 갱신 -->>
		has_active_hill_target_ = true;
		last_hill_update_time_ = this->now();

		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 2000,
			"Hill Updated: (%.3f, %.3f, %.3f) [frame=%s]",
			msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
			msg->header.frame_id.c_str());
		tryProcess();
	}

	void onLocation(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
	{
		last_loc_ = *msg;
		have_last_loc_ = true;

		RCLCPP_INFO_THROTTLE(
			get_logger(), *get_clock(), 2000,
			"Robot: (%.3f, %.3f, %.3f) [frame=%s]",
			msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
			msg->header.frame_id.c_str());
		tryProcess();
	}

	//언덕과 로봇 각도 수정 및 알려줌
	void tryProcess()
	{
		if (!(have_last_hill_ && have_last_loc_)) return;

		// --- 수정: TF 조회 대신, 구독받은 last_loc_의 방향 정보를 직접 사용 ---
		// 이 방식이 훨씬 더 안정적이고 직접적입니다.
		double robot_yaw_rad = tf2::getYaw(last_loc_.pose.orientation);

		// --- 기존 코드 ---
		const auto &hill = last_hill_;
		const auto &loc  = last_loc_;

		if (hill.header.frame_id != loc.header.frame_id) {
			RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 3000,
			"Frame mismatch: hill in '%s', location in '%s' — 같은 좌표계인지 확인 필요",
			hill.header.frame_id.c_str(), loc.header.frame_id.c_str());
		}

		const double dx = hill.pose.position.x - loc.pose.position.x;
		const double dy = hill.pose.position.y - loc.pose.position.y;
		const double dz = hill.pose.position.z - loc.pose.position.z;

		dist_xy_m = std::hypot(dx, dy);
		dist_3d_m = std::sqrt(dx*dx + dy*dy + dz*dz);

		to_hill_yaw_rad = std::atan2(dy, dx);

		// 이제 robot_yaw_rad에 실제 로봇의 Yaw 값이 들어오므로 정확한 오차 계산이 가능합니다.
		yaw_error_rad = normalize_angle(to_hill_yaw_rad - robot_yaw_rad);

		RCLCPP_INFO(get_logger(),
			"\n[SandHill & Location]\n"
			"  Robot @ (%.3f, %.3f, %.3f), yaw(base_link)=%.1f°\n"
			"  Hill  @ (%.3f, %.3f, %.3f)\n"
			"  Δ = (dx=%.3f, dy=%.3f, dz=%.3f), dist_xy=%.3f m, dist_3d=%.3f m\n"
			"  to_hill_yaw=%.1f°, yaw_error(rotate by)=%.1f°\n",
			loc.pose.position.x,  loc.pose.position.y,  loc.pose.position.z,  robot_yaw_rad * 180.0 / M_PI,
			hill.pose.position.x, hill.pose.position.y, hill.pose.position.z,
			dx, dy, dz, dist_xy_m, dist_3d_m,
			to_hill_yaw_rad * 180.0 / M_PI, yaw_error_rad * 180.0 / M_PI);
		
	}

	// <<-- 추가: `tryProcess`를 목표 기반으로 수정한 새 함수 -->>
	void tryProcessWithTarget(const geometry_msgs::msg::PoseStamped& target_hill)
	{
		// 로봇 위치 수신 여부만 확인
		if (!have_last_loc_) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[ROBOT] 위치 수신 없음");
			return;
		}

		double robot_yaw_rad = tf2::getYaw(last_loc_.pose.orientation);
		
		const auto &hill = target_hill; // <<-- 인자로 받은 고정된 목표 사용
		const auto &loc  = last_loc_;

		const double dx = hill.pose.position.x - loc.pose.position.x;
		const double dy = hill.pose.position.y - loc.pose.position.y;
		const double dz = hill.pose.position.z - loc.pose.position.z;

		dist_xy_m = std::hypot(dx, dy);
		dist_3d_m = std::sqrt(dx*dx + dy*dy + dz*dz);

		to_hill_yaw_rad = std::atan2(dy, dx);

		// 이제 robot_yaw_rad에 실제 로봇의 Yaw 값이 들어오므로 정확한 오차 계산이 가능합니다.
		yaw_error_rad = normalize_angle(to_hill_yaw_rad - robot_yaw_rad);

		RCLCPP_INFO(get_logger(),
			"\n[SandHill & Location]\n"
			"  Robot @ (%.3f, %.3f, %.3f), yaw(base_link)=%.1f°\n"
			"  Hill  @ (%.3f, %.3f, %.3f)\n"
			"  Δ = (dx=%.3f, dy=%.3f, dz=%.3f), dist_xy=%.3f m, dist_3d=%.3f m\n"
			"  to_hill_yaw=%.1f°, yaw_error(rotate by)=%.1f°\n",
			loc.pose.position.x,  loc.pose.position.y,  loc.pose.position.z,  robot_yaw_rad * 180.0 / M_PI,
			hill.pose.position.x, hill.pose.position.y, hill.pose.position.z,
			dx, dy, dz, dist_xy_m, dist_3d_m,
			to_hill_yaw_rad * 180.0 / M_PI, yaw_error_rad * 180.0 / M_PI);

	}



    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        if (visited_map_.empty()) {
            visited_map_.resize(msg->data.size(), 0);
        }
        map_received_ = true;
    }
	//이거 곧 삭제 될 예정 public의 pose_callback 으로 대체될 예정
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        pose_received_ = true;
    }

	  // 언덕 좌표를 실시간으로 받아 리스트에 추가
    void hillCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        // 너무 비슷한 위치에 이미 언덕이 있는지 확인 (중복 방지)
		bool is_new_hill = true;
		for (const auto& existing_hill : hills_) {
			double dx = msg->pose.position.x - existing_hill.pose.position.x;
			double dy = msg->pose.position.y - existing_hill.pose.position.y;
			if (std::hypot(dx, dy) < 0.25) { // 25cm 이내에 있으면 같은 언덕으로 간주
				is_new_hill = false;
				break;
			}
		}

		if (is_new_hill) {
        hills_.push_back(*msg);
        hills_destroyed_.push_back(false);
        RCLCPP_INFO(this->get_logger(), ">> 새로운 언덕 발견 및 리스트에 추가! 총 %zu개", hills_.size());
    }

    // 이 변수들은 타임아웃 용도로 계속 갱신하는 것이 좋습니다.
    last_hill_update_time_ = this->now();

    }

	bool findNextHill(geometry_msgs::msg::PoseStamped &next_hill) {
        double min_dist = std::numeric_limits<double>::max();
		bool found = false;

		// 로봇의 위치가 한 번이라도 수신되었는지 확인
		if (!have_last_loc_) {
			return false; // 위치 정보가 없으면 언덕을 찾을 수 없음
		}

		for (size_t i = 0; i < hills_.size(); i++) {
			if (hills_destroyed_[i]) continue; // 파괴된 언덕은 건너뜀

			// <<-- 핵심 수정: current_pose_ 대신 last_loc_ 을 사용합니다. -->>
			double dx = hills_[i].pose.position.x - last_loc_.pose.position.x;
			double dy = hills_[i].pose.position.y - last_loc_.pose.position.y;
			double dist = std::hypot(dx, dy); // sqrt(dx*dx + dy*dy) 보다 hypot이 더 안전하고 빠름

			if (dist < min_dist) {
				min_dist = dist;
				next_hill = hills_[i];
				found = true;
			}
		}
		return found;
    }

	void markHillDestroyed(const geometry_msgs::msg::PoseStamped &hill) {
        for (size_t i = 0; i < hills_.size(); i++) {
            double dx = hills_[i].pose.position.x - hill.pose.position.x;
            double dy = hills_[i].pose.position.y - hill.pose.position.y;
            if (sqrt(dx*dx + dy*dy) < 0.2) { // 언덕과의 거리 기준 (0.2m 이내)
                hills_destroyed_[i] = true;
                RCLCPP_INFO(this->get_logger(), "💥 Hill destroyed at x=%.2f, y=%.2f",
                            hill.pose.position.x, hill.pose.position.y);
                break;
            }
        }
    }


    void plannerLoop() {
        if (!map_received_ || !pose_received_) return;

        // 1. frontier 후보 찾기
        //auto target = findNextFrontier();

		geometry_msgs::msg::PoseStamped target;
		if (!findNextHill(target)) {
			RCLCPP_INFO(this->get_logger(), "🎉 모든 언덕을 파괴했습니다!");
			return;
		}

        // 2. A*로 경로 생성
        auto path = runAStar(current_pose_, target);

        // 3. path publish
        if (!path.poses.empty()) {
            path_pub_->publish(path);
            // 방문 경로 기록
            markVisited(path);

			// ✅ 도착 확인 및 언덕 파괴
			if (has_arrived(current_pose_, target, 0.3)) {
				markHillDestroyed(target);
			}

        }
    }

	//미방문 지역 중 다음으로 갈 곳을 정함
    geometry_msgs::msg::PoseStamped findNextFrontier() {

        geometry_msgs::msg::PoseStamped goal;
        // TODO: 미방문 영역 중 가장 가까운 점 선택 + 인데 왜 1.0 더함? 임시? - 임시 input 값이 아직 없어서..
        goal.pose.position.x = current_pose_.pose.position.x + 1.0;
        goal.pose.position.y = current_pose_.pose.position.y;
        goal.header.frame_id = "map";
        goal.header.stamp = this->now();
        return goal;
    }

    nav_msgs::msg::Path runAStar(const geometry_msgs::msg::PoseStamped &start,
                             const geometry_msgs::msg::PoseStamped &goal) {
		nav_msgs::msg::Path path;
		path.header.frame_id = "map";
		path.header.stamp = this->now();

		int width = current_map_.info.width;
		int height = current_map_.info.height;
		double res = current_map_.info.resolution;
		double origin_x = current_map_.info.origin.position.x;
		double origin_y = current_map_.info.origin.position.y;

		auto worldToGrid = [&](double wx, double wy) {
			int gx = (int)((wx - origin_x) / res);
			int gy = (int)((wy - origin_y) / res);
			return std::make_pair(gx, gy);
		};
		auto gridToWorld = [&](int gx, int gy) {
			double wx = gx * res + origin_x + res / 2.0;
			double wy = gy * res + origin_y + res / 2.0;
			return std::make_pair(wx, wy);
		};
		auto isValid = [&](int x, int y) {
			if (x < 0 || y < 0 || x >= width || y >= height) return false;
			int idx = y * width + x;
			return current_map_.data[idx] >= 0 && current_map_.data[idx] < 50;
		};
		auto heuristic = [&](int x, int y, int gx, int gy) {
			return hypot(gx - x, gy - y);
		};

		auto [sx, sy] = worldToGrid(start.pose.position.x, start.pose.position.y);
		auto [gx, gy] = worldToGrid(goal.pose.position.x, goal.pose.position.y);

		struct Node {int x,y; double g,h; Node* parent; double f() const{return g+h;}};
		struct NodeCmp {bool operator()(Node* a, Node* b){return a->f()>b->f();}};

		std::priority_queue<Node*,std::vector<Node*>,NodeCmp> open;
		std::vector<bool> closed(width*height,false);
		std::unordered_map<int,Node*> cache;

		Node* startNode=new Node{sx,sy,0.0,heuristic(sx,sy,gx,gy),nullptr};
		open.push(startNode);
		cache[sy*width+sx]=startNode;

		std::vector<std::pair<int,int>> dirs={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
		Node* goalNode=nullptr;

		while(!open.empty()){
			Node* cur=open.top(); open.pop();
			int idx=cur->y*width+cur->x;
			if(closed[idx]) continue;
			closed[idx]=true;

			if(cur->x==gx && cur->y==gy){goalNode=cur;break;}

			for(auto d:dirs){
				int nx=cur->x+d.first, ny=cur->y+d.second;
				if(!isValid(nx,ny)) continue;
				int nidx=ny*width+nx;
				if(closed[nidx]) continue;

				double step=(abs(d.first)+abs(d.second)==2)?1.414:1.0;
				double new_g=cur->g+step;

				if(cache.find(nidx)==cache.end()){
					Node* n=new Node{nx,ny,new_g,heuristic(nx,ny,gx,gy),cur};
					cache[nidx]=n; open.push(n);
				}else{
					Node* n=cache[nidx];
					if(new_g<n->g){n->g=new_g; n->parent=cur;}
				}
			}
		}

		if(goalNode){
			Node* n=goalNode;
			while(n){
				auto [wx,wy]=gridToWorld(n->x,n->y);
				geometry_msgs::msg::PoseStamped pose;
				pose.header.frame_id="map";
				pose.pose.position.x=wx; pose.pose.position.y=wy; pose.pose.orientation.w=1.0;
				path.poses.push_back(pose);
				n=n->parent;
			}
			std::reverse(path.poses.begin(),path.poses.end());
		}

		for(auto &kv:cache) delete kv.second;
		return path;
	}


    void markVisited(const nav_msgs::msg::Path &path) {
        
		if (!map_received_) return;

        auto worldToGrid = [&](double wx, double wy) {
            int gx = static_cast<int>((wx - current_map_.info.origin.position.x) / current_map_.info.resolution);
            int gy = static_cast<int>((wy - current_map_.info.origin.position.y) / current_map_.info.resolution);
            return std::make_pair(gx, gy);
        };

        for (const auto& pose : path.poses) {
            auto [gx, gy] = worldToGrid(pose.pose.position.x, pose.pose.position.y);
            int idx = gy * current_map_.info.width + gx;
            if (idx >= 0 && idx < visited_map_.size()) {
                visited_map_[idx] = 1; // 방문 표시
            }
        }

        RCLCPP_INFO(this->get_logger(), " Path marked as visited");

    }
	
 

};





std::shared_ptr<IAHRS> node = nullptr;

// SIGINT 핸들러 직접 구현
void sigint_handler(int signal)
{
    if (node) {
        node->stop_motors();    // 직접 모터 중지 명령 호출
        rclcpp::shutdown();     // 그 후에 shutdown 호출
    }
}

// SIGINT (Ctrl+C) 안전한 종료 처리를 위한 핸들러 함수
void safe_shutdown_handler(int signal) {
    if (node) {
        for (auto& t : node->timers_) {
            if (t) t->cancel(); // 벡터의 각 타이머를 하나씩 cancel
        }
        node->stop_motors();    // 모터 강제 정지 및 토크 OFF
    }
    rclcpp::shutdown();
}



int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);

	//auto node = std::shared_ptr<IAHRS>();

	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	
	double roll_1, pitch_1, yaw_1;
	double roll_2, pitch_2, yaw_2;
	double x, y;
	
	auto node = std::make_shared<IAHRS>();
	//IAHRS iahrs;

	// SIGINT (Ctrl+C) 시그널 핸들러 등록
    //signal(SIGINT, sigint_handler);
	signal(SIGINT, safe_shutdown_handler);

    //rclcpp::WallRate loop_rate(100); 

	IAHRS& iahrs = *node;   // node가 가리키는 동일한 객체를 iahrs 라는 이름으로 쓴다


	// These values do not need to be converted
	iahrs.imu_data_msg.linear_acceleration_covariance[0] = 0.0064;
	iahrs.imu_data_msg.linear_acceleration_covariance[4] = 0.0063;
	iahrs.imu_data_msg.linear_acceleration_covariance[8] = 0.0064;
	iahrs.imu_data_msg.angular_velocity_covariance[0] = 0.032*(M_PI/180.0);
	iahrs.imu_data_msg.angular_velocity_covariance[4] = 0.028*(M_PI/180.0);
	iahrs.imu_data_msg.angular_velocity_covariance[8] = 0.006*(M_PI/180.0);
	iahrs.imu_data_msg.orientation_covariance[0] = 0.013*(M_PI/180.0);
	iahrs.imu_data_msg.orientation_covariance[4] = 0.011*(M_PI/180.0);
	iahrs.imu_data_msg.orientation_covariance[8] = 0.006*(M_PI/180.0);



	rclcpp::WallRate loop_rate(100);
	iahrs.open_serial_1(SERIAL_PORT1);
	iahrs.open_serial_2(SERIAL_PORT2);
	iahrs.SendRecv("za\n", dSend_Data, 10 , serial_fd_1);	// Euler Angle -> '0.0' Reset
	iahrs.SendRecv("za\n", dSend_Data, 10 , serial_fd_2);
	usleep(10000);
	printf("                       | Z axis \n");
	printf("                       | \n");
	printf("                       |   / X axis \n");
	printf("                   ____|__/____ \n");
	printf("      Y axis     / *   | /    /| \n");
	printf("      _________ /______|/    // \n");
	printf("               /___________ // \n");
	printf("              |____iahrs___|/ \n");


		

	while(rclcpp::ok())
    	{
		rclcpp::spin_some(node);
		// ✅ IMU 데이터를 ROS 메시지로 변환

        sensor_msgs::msg::Imu imu_data_msg_1, imu_data_msg_2;
		iahrs.imu_array_msg.imus.clear();

		//iahrs.publish_transforms_worlds();

		iahrs.imu_data_1(imu_data_msg_1);

		iahrs.imu_data_2(imu_data_msg_2);

		iahrs.imu_array_msg.header.stamp = node->now();
		iahrs.imu_array_msg.header.frame_id = "imu_combined";
		iahrs.imu_array_pub->publish(iahrs.imu_array_msg);

		rclcpp::spin_some(node);

		// IMU 데이터 갱신
		iahrs.getRPYFromImu(
			iahrs.imu_array_msg.imus[0], 
			iahrs.imu_data.roll_1, 
			iahrs.imu_data.pitch_1, 
			iahrs.imu_data.yaw_1
		);

		iahrs.getRPYFromImu(
			iahrs.imu_array_msg.imus[1], 
			iahrs.imu_data.roll_2, 
			iahrs.imu_data.pitch_2, 
			iahrs.imu_data.yaw_2
		);

		// 동작 실험.
		//iahrs.execute_mode1(iahrs.saved_x, iahrs.saved_y);

		// if (iahrs.mode_change == 1) {
		// 	iahrs.MODE1(roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2);
		// }
		// else if (iahrs.mode_change == 2) {
		// 	iahrs.MODE2(roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2);
		// }

		//iahrs.MODE1(roll_1,pitch_1,yaw_1,roll_2,pitch_2,yaw_2);

		//iahrs.MODE2(roll_1,pitch_1,yaw_1,roll_2,pitch_2,yaw_2); 

		iahrs.hill_destroy();

		//iahrs.move_forward_mode2();

		//iahrs.set_motor_speeds(iahrs.basic_speed + 200, iahrs.basic_speed_inverse + 70, iahrs.basic_speed_inverse + 200 );

		//iahrs.execute_mode1(roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2, iahrs.saved_x, iahrs.saved_y);

	

		loop_rate.sleep();

		}
		
		//  iahrs.imu_array_msg.header.stamp = node->now();
		//  iahrs.imu_array_msg.header.frame_id = "imu_combined";
		//  iahrs.imu_array_pub->publish(iahrs.imu_array_msg);
		
		// 루프 종료 직후 확실한 모터 정지 (중요)
    	node->stop_motors();
        
    

	close (serial_fd_1);
	close (serial_fd_2);
	rclcpp::shutdown();

    return 0;
}