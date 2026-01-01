#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rm_interfaces/msg/target.hpp>
#include <cmath>
#include <optional>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class EnemyTrackingNode : public rclcpp::Node {
public:
    EnemyTrackingNode() : Node("enemy_tracking_node") {
        // 参数声明
        declare_parameter("target_topic", "armor_solver/target");
        declare_parameter("costmap_topic", "/global_costmap/costmap");
        declare_parameter("map_frame", "map");
        declare_parameter("attack_radius", 3.0);          // 攻击半径(与敌人保持的距离)
        declare_parameter("num_sectors", 36);             // 候选点数量
        declare_parameter("cost_threshold", 50);          // costmap代价阈值
        declare_parameter("goal_update_interval", 1.0);   // 目标更新最小间隔(秒)
        declare_parameter("enable_navigation", true);     // 是否启用导航(调试时可关闭)
        declare_parameter("tf_tolerance", 0.5);           // TF时间容忍度

        // 获取参数
        map_frame_ = get_parameter("map_frame").as_string();
        attack_radius_ = get_parameter("attack_radius").as_double();
        num_sectors_ = get_parameter("num_sectors").as_int();
        cost_threshold_ = get_parameter("cost_threshold").as_int();
        goal_update_interval_ = get_parameter("goal_update_interval").as_double();
        enable_navigation_ = get_parameter("enable_navigation").as_bool();
        tf_tolerance_ = get_parameter("tf_tolerance").as_double();

        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Action客户端
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 可视化发布器
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/enemy_tracking/markers", 10);

        // 订阅视觉追踪目标
        target_sub_ = create_subscription<rm_interfaces::msg::Target>(
            get_parameter("target_topic").as_string(), 10,
            std::bind(&EnemyTrackingNode::targetCallback, this, std::placeholders::_1));

        // 订阅costmap用于避障
        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            get_parameter("costmap_topic").as_string(), 10,
            std::bind(&EnemyTrackingNode::costmapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Enemy tracking node started");
    }

private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_ = msg;
    }

    void targetCallback(const rm_interfaces::msg::Target::SharedPtr msg) {
        // 检查是否正在追踪
        if (!msg->tracking) {
            // 目标丢失，使用最后已知位置
            if (last_enemy_position_.has_value()) {
                RCLCPP_DEBUG(get_logger(), "Target lost, using last known position");
            } else {
                return;
            }
        }

        // 更新频率限制
        auto now = get_clock()->now();
        if ((now - last_goal_time_).seconds() < goal_update_interval_) {
            return;
        }

        // 转换敌人位置到map坐标系
        geometry_msgs::msg::PointStamped enemy_point;
        enemy_point.header = msg->header;
        if (msg->tracking) {
            enemy_point.point = msg->position;
        } else {
            enemy_point.point = last_enemy_position_.value();
        }

        geometry_msgs::msg::PointStamped enemy_in_map;
        try {
            enemy_in_map = tf_buffer_->transform(enemy_point, map_frame_, tf2::durationFromSec(tf_tolerance_));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
            return;
        }

        // 保存最后已知位置
        if (msg->tracking) {
            last_enemy_position_ = msg->position;
        }

        // 生成候选攻击点(敌人周围的圆形分布)
        std::vector<geometry_msgs::msg::Point> candidates = generateCandidatePoints(enemy_in_map.point);

        // 使用costmap过滤不可行的点
        std::vector<geometry_msgs::msg::Point> feasible_points;
        if (costmap_) {
            feasible_points = filterFeasiblePoints(candidates, *costmap_);
        } else {
            feasible_points = candidates;  // 没有costmap时使用所有候选点
        }

        if (feasible_points.empty()) {
            RCLCPP_WARN(get_logger(), "No feasible attack points found");
            return;
        }

        // 获取机器人当前位置
        geometry_msgs::msg::Point robot_pos;
        if (!getRobotPosition(robot_pos)) {
            return;
        }

        // 选择最优攻击点(离机器人最近的可行点)
        geometry_msgs::msg::Point best_point = selectBestPoint(feasible_points, robot_pos);

        // 创建攻击姿态(朝向敌人)
        geometry_msgs::msg::PoseStamped attack_pose = createAttackPose(best_point, enemy_in_map);

        // 发布可视化
        publishMarkers(enemy_in_map.point, candidates, feasible_points, best_point);

        // 发送导航目标
        if (enable_navigation_) {
            sendNavigationGoal(attack_pose);
        }
        last_goal_time_ = now;
    }

    // 获取机器人当前位置
    bool getRobotPosition(geometry_msgs::msg::Point& robot_pos) {
        try {
            geometry_msgs::msg::TransformStamped robot_tf;
            robot_tf = tf_buffer_->lookupTransform(map_frame_, "base_link", tf2::TimePointZero);
            robot_pos.x = robot_tf.transform.translation.x;
            robot_pos.y = robot_tf.transform.translation.y;
            robot_pos.z = 0.0;
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "Cannot get robot position: %s", ex.what());
            return false;
        }
    }

    // 生成候选攻击点(敌人周围的圆形分布)
    std::vector<geometry_msgs::msg::Point> generateCandidatePoints(const geometry_msgs::msg::Point& enemy) {
        std::vector<geometry_msgs::msg::Point> candidates;
        candidates.reserve(num_sectors_);

        for (int i = 0; i < num_sectors_; ++i) {
            double angle = i * 2 * M_PI / num_sectors_;
            geometry_msgs::msg::Point p;
            p.x = enemy.x + attack_radius_ * cos(angle);
            p.y = enemy.y + attack_radius_ * sin(angle);
            p.z = 0.0;
            candidates.push_back(p);
        }
        return candidates;
    }

    // 使用costmap过滤不可行的点
    std::vector<geometry_msgs::msg::Point> filterFeasiblePoints(
        const std::vector<geometry_msgs::msg::Point>& candidates,
        const nav_msgs::msg::OccupancyGrid& costmap) {

        std::vector<geometry_msgs::msg::Point> feasible;
        const nav_msgs::msg::MapMetaData& info = costmap.info;

        for (size_t i = 0; i < candidates.size(); ++i) {
            const geometry_msgs::msg::Point& p = candidates[i];

            // 转换到栅格坐标
            int cell_x = static_cast<int>((p.x - info.origin.position.x) / info.resolution);
            int cell_y = static_cast<int>((p.y - info.origin.position.y) / info.resolution);

            // 边界检查
            if (cell_x < 0 || cell_x >= static_cast<int>(info.width) ||
                cell_y < 0 || cell_y >= static_cast<int>(info.height)) {
                continue;
            }

            // 检查代价值
            int index = cell_y * info.width + cell_x;
            int8_t cost = costmap.data[index];
            if (cost >= 0 && cost <= cost_threshold_) {
                feasible.push_back(p);
            }
        }
        return feasible;
    }

    // 选择最优攻击点(离机器人最近)
    geometry_msgs::msg::Point selectBestPoint(
        const std::vector<geometry_msgs::msg::Point>& points,
        const geometry_msgs::msg::Point& robot_pos) {

        geometry_msgs::msg::Point best = points[0];
        double min_dist = std::hypot(best.x - robot_pos.x, best.y - robot_pos.y);

        for (size_t i = 1; i < points.size(); ++i) {
            double dist = std::hypot(points[i].x - robot_pos.x, points[i].y - robot_pos.y);
            if (dist < min_dist) {
                min_dist = dist;
                best = points[i];
            }
        }
        return best;
    }

    // 创建攻击姿态(朝向敌人)
    geometry_msgs::msg::PoseStamped createAttackPose(
        const geometry_msgs::msg::Point& attack_point,
        const geometry_msgs::msg::PointStamped& enemy) {

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_frame_;
        pose.header.stamp = get_clock()->now();
        pose.pose.position = attack_point;

        // 计算朝向敌人的角度
        double dx = enemy.point.x - attack_point.x;
        double dy = enemy.point.y - attack_point.y;
        double yaw = atan2(dy, dx);

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);

        return pose;
    }

    // 发布可视化markers
    void publishMarkers(const geometry_msgs::msg::Point& enemy,
                        const std::vector<geometry_msgs::msg::Point>& candidates,
                        const std::vector<geometry_msgs::msg::Point>& feasible,
                        const geometry_msgs::msg::Point& best) {

        visualization_msgs::msg::MarkerArray markers;
        auto stamp = get_clock()->now();

        // 敌人位置(蓝色球)
        visualization_msgs::msg::Marker enemy_marker;
        enemy_marker.header.frame_id = map_frame_;
        enemy_marker.header.stamp = stamp;
        enemy_marker.ns = "enemy";
        enemy_marker.id = 0;
        enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
        enemy_marker.action = visualization_msgs::msg::Marker::ADD;
        enemy_marker.pose.position = enemy;
        enemy_marker.scale.x = 0.4;
        enemy_marker.scale.y = 0.4;
        enemy_marker.scale.z = 0.4;
        enemy_marker.color.r = 0.0;
        enemy_marker.color.g = 0.0;
        enemy_marker.color.b = 1.0;
        enemy_marker.color.a = 1.0;
        enemy_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        markers.markers.push_back(enemy_marker);

        // 最优攻击点(绿色球)
        visualization_msgs::msg::Marker best_marker;
        best_marker.header.frame_id = map_frame_;
        best_marker.header.stamp = stamp;
        best_marker.ns = "best";
        best_marker.id = 0;
        best_marker.type = visualization_msgs::msg::Marker::SPHERE;
        best_marker.action = visualization_msgs::msg::Marker::ADD;
        best_marker.pose.position = best;
        best_marker.scale.x = 0.4;
        best_marker.scale.y = 0.4;
        best_marker.scale.z = 0.4;
        best_marker.color.r = 0.0;
        best_marker.color.g = 1.0;
        best_marker.color.b = 0.0;
        best_marker.color.a = 1.0;
        best_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        markers.markers.push_back(best_marker);

        // 候选点(红色，可行点更亮)
        for (size_t i = 0; i < candidates.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_;
            m.header.stamp = stamp;
            m.ns = "candidates";
            m.id = i;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position = candidates[i];
            m.scale.x = 0.15;
            m.scale.y = 0.15;
            m.scale.z = 0.15;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.lifetime = rclcpp::Duration::from_seconds(2.0);

            // 检查是否是可行点
            bool is_feasible = false;
            for (size_t j = 0; j < feasible.size(); ++j) {
                if (feasible[j].x == candidates[i].x && feasible[j].y == candidates[i].y) {
                    is_feasible = true;
                    break;
                }
            }
            m.color.a = is_feasible ? 0.8 : 0.3;

            markers.markers.push_back(m);
        }

        // 攻击半径圆圈
        visualization_msgs::msg::Marker circle;
        circle.header.frame_id = map_frame_;
        circle.header.stamp = stamp;
        circle.ns = "range";
        circle.id = 0;
        circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
        circle.action = visualization_msgs::msg::Marker::ADD;
        circle.scale.x = 0.05;
        circle.color.r = 0.0;
        circle.color.g = 0.0;
        circle.color.b = 1.0;
        circle.color.a = 0.5;
        circle.lifetime = rclcpp::Duration::from_seconds(2.0);

        for (int i = 0; i <= 36; ++i) {
            double angle = i * 2 * M_PI / 36;
            geometry_msgs::msg::Point p;
            p.x = enemy.x + attack_radius_ * cos(angle);
            p.y = enemy.y + attack_radius_ * sin(angle);
            p.z = 0.0;
            circle.points.push_back(p);
        }
        markers.markers.push_back(circle);

        marker_pub_->publish(markers);
    }

    void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal_pose) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "Nav2 action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&EnemyTrackingNode::navigationResultCallback, this, std::placeholders::_1);

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(get_logger(), "Sent nav goal: (%.2f, %.2f)",
                    goal_pose.pose.position.x, goal_pose.pose.position.y);
    }

    void navigationResultCallback(const GoalHandleNav::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "Navigation succeeded");
        }
    }

    // 参数
    std::string map_frame_;
    double attack_radius_;
    int num_sectors_;
    int cost_threshold_;
    double goal_update_interval_;
    bool enable_navigation_;
    double tf_tolerance_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Action客户端
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    // 订阅者
    rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    // 发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // 状态
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    std::optional<geometry_msgs::msg::Point> last_enemy_position_;
    rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnemyTrackingNode>());
    rclcpp::shutdown();
    return 0;
}
