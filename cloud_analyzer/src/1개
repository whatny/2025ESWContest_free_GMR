//언덕인식 + pose + 언덕하나만

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>  // 추가
#include <geometry_msgs/msg/vector3_stamped.hpp>  // 추가
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>  // 추가
#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>

class GroundHillDetector : public rclcpp::Node
{
public:
    struct SandHill {
        int id;
        double center_x, center_y, center_z;
        double width, length, height;
        double distance_from_robot;
        double point_count;
        double elevation;  // 바닥 기준 높이
        double camera_yaw;  // 추가: 카메라 기준 방향각
        double map_yaw;     // 추가: map 기준 방향각
    };

    GroundHillDetector() : Node("ground_hill_detector")
    {
        RCLCPP_INFO(this->get_logger(), "=== Ground 격자+병합 모래언덕 감지기 (RTABMap Map 좌표계) === ");
        
        setupGroundParameters();
        
        // 파라미터로 target frame 설정 가능 (기본값: map)
        this->declare_parameter("target_frame", "map");
        target_frame_ = this->get_parameter("target_frame").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame_.c_str());
        
        // TF2 설정
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // RTABMap Ground Grid 구독
        depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rtabmap/local_grid_ground", 10,
            std::bind(&GroundHillDetector::groundCallback, this, std::placeholders::_1));
        
        // 모래언덕 위치+방향 발행 (RTABMap map 좌표계로 변환)
        hill_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/sand_hills", 10);  // 변경
        
        RCLCPP_INFO(this->get_logger(), " Ground 격자+병합 준비 완료!");
        RCLCPP_INFO(this->get_logger(), " 설정:");
        RCLCPP_INFO(this->get_logger(), "   - 분석 범위: %.1fm 반경", max_range_);
        RCLCPP_INFO(this->get_logger(), "   - 높이 차이: %.0fcm 이상", min_elevation_*100);
        RCLCPP_INFO(this->get_logger(), "   - 격자 크기: %.0fcm", grid_size_*100);
        RCLCPP_INFO(this->get_logger(), "좌표 변환: camera_link → %s (RTABMap)", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), " /rtabmap/local_grid_ground에서 모래언덕 감지 시작!");
    }

private:
    void setupGroundParameters()
    {
        max_range_ = 0.9;        
        min_range_ = 0.2;           
        min_elevation_ = 0.08;    
        max_elevation_ = 0.20;      
        min_cluster_size_ = 3;      
        grid_size_ = 0.08;         
        
        RCLCPP_INFO(this->get_logger(), " 올바른 좌표계: X(앞뒤), Y(좌우), Z(위아래)");
    }

    void groundCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        static int frame_count = 0;
        frame_count++;
        
        try {
            // PCL 변환
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *cloud);
            
            if (cloud->points.size() < 10) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    " Ground 포인트 수 부족: %zu개", cloud->points.size());
                return;
            }
            
            // 주기적 상태 출력
            if (frame_count % 50 == 1) {
                RCLCPP_INFO(this->get_logger(), "Ground 격자+병합 감지 중... 프레임 #%d", frame_count);
                printGroundStats(cloud);
            }
            
            // 1단계: 거리 필터링
            auto filtered_cloud = filterGroundPoints(cloud);
            
            // 2단계: 격자+병합 기반 높이 분석
            auto hills = detectGroundHills(filtered_cloud);
            
            // 3단계: 결과 발행 (camera_link와 map 좌표계 모두)
            publishHills(hills, msg->header);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), " Ground 처리 오류: %s", e.what());
        }
    }

    // 추가: 카메라 방향각을 map 방향각으로 변환
    double convertCameraYawToMapYaw(double camera_yaw)
    {
        try {
            // 1. 카메라에서 단위 벡터 생성
            geometry_msgs::msg::Vector3Stamped camera_vector;
            camera_vector.header.frame_id = "camera_link";
            camera_vector.header.stamp = this->now();
            camera_vector.vector.x = cos(camera_yaw);
            camera_vector.vector.y = sin(camera_yaw);
            camera_vector.vector.z = 0.0;
            
            // 2. map 좌표계로 변환
            geometry_msgs::msg::Vector3Stamped map_vector;
            tf_buffer_->transform(camera_vector, map_vector, target_frame_, tf2::durationFromSec(0.1));
            
            // 3. map에서 방향각 계산
            double map_yaw = atan2(map_vector.vector.y, map_vector.vector.x);
            
            return map_yaw;
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "방향각 변환 실패: %s", ex.what());
            return camera_yaw;  // 실패시 원본 반환
        }
    }

    // 좌표 변환 함수 (RTABMap map 좌표계용)
    geometry_msgs::msg::PointStamped transformToMap(const geometry_msgs::msg::PointStamped& camera_point)
    {
        geometry_msgs::msg::PointStamped map_point;
        
        try {
            // camera_link에서 RTABMap map으로 변환 (Foxy 방식)
            tf_buffer_->transform(camera_point, map_point, target_frame_, tf2::durationFromSec(0.1));
            
            RCLCPP_INFO(this->get_logger(), 
                        " RTABMap 좌표 변환 성공: camera_link(%.3f,%.3f,%.3f) → %s(%.3f,%.3f,%.3f)",
                        camera_point.point.x, camera_point.point.y, camera_point.point.z,
                        target_frame_.c_str(),
                        map_point.point.x, map_point.point.y, map_point.point.z);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(),
                        " RTABMap TF 변환 실패 (camera_link → %s): %s", 
                        target_frame_.c_str(), ex.what());
            
            // 변환 실패시 원본 좌표 반환 (frame_id만 target_frame으로 변경)
            map_point = camera_point;
            map_point.header.frame_id = target_frame_;
        }
        
        return map_point;
    }

    void printGroundStats(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
        
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& point = cloud->points[i];
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
                min_z = std::min(min_z, point.z);
                max_z = std::max(max_z, point.z);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "\n === Ground Point Cloud 통계 (camera_link) ===");
        RCLCPP_INFO(this->get_logger(), "포인트 수: %zu개", cloud->points.size());
        RCLCPP_INFO(this->get_logger(), "X 범위: %.2f ~ %.2fm (%.2fm)", min_x, max_x, max_x-min_x);
        RCLCPP_INFO(this->get_logger(), "Y 범위: %.2f ~ %.2fm (%.2fm)", min_y, max_y, max_y-min_y);
        RCLCPP_INFO(this->get_logger(), "Z 범위: %.2f ~ %.2fm (%.2fm)", min_z, max_z, max_z-min_z);
        RCLCPP_INFO(this->get_logger(), "================================================\n");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGroundPoints(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& point = cloud->points[i];
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
            
            // X-Y 평면에서 거리 계산 (Z는 높이)
            float distance = std::sqrt(point.x*point.x + point.y*point.y);
            
            if (distance >= min_range_ && distance <= max_range_) {
                filtered->points.push_back(point);
            }
        }
        
        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = false;
        
        return filtered;
    }

    std::vector<SandHill> detectGroundHills(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        std::vector<SandHill> hills;
        
        if (cloud->points.size() < 5) return hills;
        
        // 1단계: 2D 격자로 분할하고 각 격자의 평균 높이 계산
        std::map<std::pair<int, int>, std::vector<float>> grid_heights;
        
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& point = cloud->points[i];
            
            // X-Y를 격자 좌표로 변환 (Z는 높이)
            int grid_x = static_cast<int>(point.x / grid_size_);
            int grid_y = static_cast<int>(point.y / grid_size_);
            
            grid_heights[std::make_pair(grid_x, grid_y)].push_back(point.z);
        }
        
        // 2단계: 각 격자의 평균 높이 계산
        std::map<std::pair<int, int>, float> grid_avg_height;
        float overall_min_height = std::numeric_limits<float>::max();
        
        for (std::map<std::pair<int, int>, std::vector<float>>::iterator it = grid_heights.begin(); 
             it != grid_heights.end(); ++it) {
            std::pair<int, int> coord = it->first;
            std::vector<float>& heights = it->second;
            
            if (heights.size() >= 2) {  // 최소 2개 포인트
                float sum = 0;
                for (size_t i = 0; i < heights.size(); ++i) {
                    sum += heights[i];
                }
                float avg_height = sum / heights.size();
                grid_avg_height[coord] = avg_height;
                overall_min_height = std::min(overall_min_height, avg_height);
            }
        }
        
        // 3단계: 기준 높이보다 높은 격자들 찾기 (모래언덕)
        std::vector<std::pair<int, int>> elevated_grids;
        for (auto& grid_entry : grid_avg_height) {
            auto& coord = grid_entry.first;
            float avg_height = grid_entry.second;
            float elevation = avg_height - overall_min_height;
            
            if (elevation >= min_elevation_ && elevation <= max_elevation_) {
                elevated_grids.push_back(coord);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Ground 분석: %zu개 격자 → %zu개 높은 격자", 
                   grid_avg_height.size(), elevated_grids.size());
        
        if (elevated_grids.empty()) return hills;
        
        // 4단계: Union-Find로 인접한 격자들 병합
        std::vector<int> parent(elevated_grids.size());
        for (size_t i = 0; i < elevated_grids.size(); ++i) {
            parent[i] = static_cast<int>(i);
        }
        
        // Find 함수
        auto find_root = [&](int x) -> int {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];  // 경로 압축
                x = parent[x];
            }
            return x;
        };
        
        // Union 함수
        auto unite_grids = [&](int x, int y) {
            int px = find_root(x);
            int py = find_root(y);
            if (px != py) {
                parent[px] = py;
            }
        };
        
        // 인접한 격자들 연결 (X, Y, Z 모두 고려)
        for (size_t i = 0; i < elevated_grids.size(); ++i) {
            std::pair<int, int> coord1 = elevated_grids[i];
            int x1 = coord1.first;
            int y1 = coord1.second;
            float height1 = grid_avg_height.at(coord1);
            
            for (size_t j = i + 1; j < elevated_grids.size(); ++j) {
                std::pair<int, int> coord2 = elevated_grids[j];
                int x2 = coord2.first;
                int y2 = coord2.second;
                float height2 = grid_avg_height.at(coord2);
                
                // X, Y 방향 인접 확인
                int dx = abs(x1 - x2);
                int dy = abs(y1 - y2);
                
                // Z 방향 높이 차이 확인
                float dz = abs(height2 - height1);
                
                // X, Y, Z 모두 조건: 인접하고 높이 차이도 작아야 연결
                if (dx <= 1.2 && dy <= 1.2 && dz <= 0.03) {  // 3cm 이하 높이 차이
                    unite_grids(static_cast<int>(i), static_cast<int>(j));
                }
            }
        }
        
        // 5단계: 연결된 격자 그룹별로 클러스터링
        std::map<int, std::vector<size_t>> grid_clusters;
        for (size_t i = 0; i < elevated_grids.size(); ++i) {
            int root = find_root(static_cast<int>(i));
            grid_clusters[root].push_back(i);
        }
        
        // 6단계: 각 격자 클러스터를 모래언덕으로 분석
        int hill_id = 1;
        
        for (std::map<int, std::vector<size_t>>::iterator it = grid_clusters.begin(); 
             it != grid_clusters.end(); ++it) {
            std::vector<size_t>& cluster_indices = it->second;
            
            if (cluster_indices.size() >= static_cast<size_t>(min_cluster_size_)) {
                SandHill hill = analyzeGroundHill(cluster_indices, elevated_grids, grid_avg_height, overall_min_height, hill_id, cloud);
                hills.push_back(hill);
                
                RCLCPP_INFO(this->get_logger(),
                    "Ground 언덕 #%d: 위치(%.2f,%.2f,%.2f) 높이%.0fcm 격자%.0f개 방향%.1f°",
                    hill.id, hill.center_x, hill.center_y, hill.center_z, 
                    hill.elevation*100, hill.point_count, hill.camera_yaw*180.0/M_PI);
                
                hill_id++;
            }
        }
        
        return hills;
    }

    SandHill analyzeGroundHill(const std::vector<size_t>& cluster_indices,
                              const std::vector<std::pair<int, int>>& elevated_grids,
                              const std::map<std::pair<int, int>, float>& grid_avg_height,
                              float base_height, int id,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud)
    {
        SandHill hill;
        hill.id = id;
        hill.point_count = cluster_indices.size();
        
        // 기존 방식으로 클러스터 중심과 크기 계산
        double sum_x = 0, sum_y = 0, sum_z = 0;
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        float max_height = base_height;
        
        for (size_t i = 0; i < cluster_indices.size(); ++i) {
            size_t idx = cluster_indices[i];
            std::pair<int, int> coord = elevated_grids[idx];
            int grid_x = coord.first;
            int grid_y = coord.second;
            
            // 격자 중심 좌표로 변환
            float world_x = grid_x * grid_size_ + grid_size_/2;
            float world_y = grid_y * grid_size_ + grid_size_/2;
            float world_z = grid_avg_height.at(coord);
            
            sum_x += world_x;
            sum_y += world_y;
            sum_z += world_z;
            
            min_x = std::min(min_x, world_x);
            max_x = std::max(max_x, world_x);
            min_y = std::min(min_y, world_y);
            max_y = std::max(max_y, world_y);
            max_height = std::max(max_height, world_z);
        }
        
        // 기존 방식으로 언덕 속성 계산
        hill.width = max_x - min_x + grid_size_;
        hill.length = max_y - min_y + grid_size_;
        hill.height = max_height - base_height;
        hill.elevation = hill.height;
        
        // 클러스터 영역에서 실제 포인트클라우드의 최고점 찾기
        float highest_z = std::numeric_limits<float>::lowest();
        float highest_x = sum_x / cluster_indices.size();
        float highest_y = sum_y / cluster_indices.size();
        float highest_z_found = sum_z / cluster_indices.size();
        
        // 클러스터가 차지하는 격자 영역 정의
        for (size_t i = 0; i < original_cloud->points.size(); ++i) {
            const auto& point = original_cloud->points[i];
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
            
            // 이 포인트가 클러스터 영역에 속하는지 확인
            int point_grid_x = static_cast<int>(point.x / grid_size_);
            int point_grid_y = static_cast<int>(point.y / grid_size_);
            
            bool point_in_cluster = false;
            for (size_t j = 0; j < cluster_indices.size(); ++j) {
                size_t idx = cluster_indices[j];
                std::pair<int, int> cluster_coord = elevated_grids[idx];
                
                if (point_grid_x == cluster_coord.first && point_grid_y == cluster_coord.second) {
                    point_in_cluster = true;
                    break;
                }
            }
            
            // 클러스터 영역 내에서 가장 높은 실제 포인트 찾기
            if (point_in_cluster && point.z > highest_z) {
                highest_z = point.z;
                highest_x = point.x;
                highest_y = point.y;
                highest_z_found = point.z;
            }
        }
        
        // 언덕 위치를 실제 최고점으로 설정
        hill.center_x = highest_x;
        hill.center_y = highest_y;
        hill.center_z = highest_z_found;
        
        hill.distance_from_robot = std::sqrt(hill.center_x*hill.center_x + hill.center_y*hill.center_y);
        
        // 추가: 방향각 계산
        hill.camera_yaw = atan2(hill.center_y, hill.center_x);  // 카메라 기준 방향각
        hill.map_yaw = convertCameraYawToMapYaw(hill.camera_yaw);  // map 기준 방향각
        
        return hill;
    }

    void publishHills(const std::vector<SandHill>& hills, const std_msgs::msg::Header& header)
    {
        if (hills.empty()) {
            static int no_hill_count = 0;
            no_hill_count++;
            if (no_hill_count % 20 == 1) {
                RCLCPP_INFO(this->get_logger(), " Ground 격자+병합으로 탐색 중... (없음)");
            }
            return;
        }
        
        // 가장 가까운 언덕 선택 (기존 로직 그대로)
        SandHill closest_hill = hills[0];
        for (size_t i = 1; i < hills.size(); ++i) {
            if (hills[i].distance_from_robot < closest_hill.distance_from_robot) {
                closest_hill = hills[i];
            }
        }
        
        // camera_link 좌표로 위치 생성
        geometry_msgs::msg::PointStamped hill_point;
        hill_point.header = header;  // camera_link frame_id 유지
        hill_point.point.x = closest_hill.center_x;
        hill_point.point.y = closest_hill.center_y;
        hill_point.point.z = closest_hill.center_z;
        
        // map 좌표로 위치 변환
        geometry_msgs::msg::PointStamped hill_map_point = transformToMap(hill_point);
        
        // PoseStamped 메시지 생성 (map 좌표계)
        geometry_msgs::msg::PoseStamped hill_pose_msg;
        hill_pose_msg.header.frame_id = target_frame_;
        hill_pose_msg.header.stamp = this->now();
        
        // 위치 설정 (map 좌표)
        hill_pose_msg.pose.position.x = hill_map_point.point.x;
        hill_pose_msg.pose.position.y = hill_map_point.point.y;
        hill_pose_msg.pose.position.z = hill_map_point.point.z;
        
        // 방향 설정 (map 좌표계 yaw)
        tf2::Quaternion q;
        q.setRPY(0, 0, closest_hill.map_yaw);
        hill_pose_msg.pose.orientation.x = q.x();
        hill_pose_msg.pose.orientation.y = q.y();
        hill_pose_msg.pose.orientation.z = q.z();
        hill_pose_msg.pose.orientation.w = q.w();
        
        // PoseStamped로 발행
        hill_pub_->publish(hill_pose_msg);
        
        // 기존 출력 + 방향각 정보 추가
        RCLCPP_INFO(this->get_logger(), "\n===  Ground 격자+병합 모래언덕 감지 ===");
        RCLCPP_INFO(this->get_logger(), "총 %zu개 언덕 발견", hills.size());
        
        for (size_t i = 0; i < hills.size(); ++i) {
            const SandHill& hill = hills[i];
            RCLCPP_INFO(this->get_logger(),
                "언덕 #%d: 위치(%.2f,%.2f,%.2f) 거리%.2fm 높이%.0fcm 크기%.2f×%.2fm 방향%.1f°→%.1f°",
                hill.id, hill.center_x, hill.center_y, hill.center_z,
                hill.distance_from_robot, hill.elevation*100, hill.width, hill.length,
                hill.camera_yaw*180.0/M_PI, hill.map_yaw*180.0/M_PI);
        }
        
        RCLCPP_INFO(this->get_logger(), " 발행: #%d (거리 %.2fm)", 
                   closest_hill.id, closest_hill.distance_from_robot);
        RCLCPP_INFO(this->get_logger(), "토픽: /sand_hills → PoseStamped (위치+방향)");
        RCLCPP_INFO(this->get_logger(), " 위치: (%.3f, %.3f, %.3f)", 
                   hill_pose_msg.pose.position.x, hill_pose_msg.pose.position.y, hill_pose_msg.pose.position.z);
        RCLCPP_INFO(this->get_logger(), " 방향: %.1f° (map 기준)", 
                   closest_hill.map_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "=====================================\n");
    }

    // 멤버 변수들
    double max_range_, min_range_;
    double min_elevation_, max_elevation_;
    int min_cluster_size_;
    double grid_size_;
    std::string target_frame_;  // RTABMap 좌표계 이름
    
    // TF2 관련
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hill_pub_;  // 변경: PoseStamped
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GroundHillDetector>();
    
    RCLCPP_INFO(node->get_logger(), "Ground 격자+병합 모래언덕 감지 시작!");
    RCLCPP_INFO(node->get_logger(), " /rtabmap/local_grid_ground에서 바닥면 높이 변화로 모래언덕을 감지합니다!");
    RCLCPP_INFO(node->get_logger(), " 발행: /sand_hills (PoseStamped - 위치+방향, map 좌표계)");
    
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Ground 감지기 종료");
    rclcpp::shutdown();
    return 0;
}
