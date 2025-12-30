#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class EnemyTrackingNode : public rclcpp::Node {
public:
    EnemyTrackingNode() : Node("enemy_tracking_node") {
        // 参数声明
        declare_parameter("gimbal_frame", "gimbal");
        declare_parameter("map_frame", "map");
        declare_parameter("enemy_distance_topic", "/enemy_distance");
        declare_parameter("min_tracking_distance", 0.5);   // 太近不追
        declare_parameter("max_tracking_distance", 10.0);  // 太远不追
        declare_parameter("goal_update_interval", 1.0);    // 目标更新最小间隔(秒)
        declare_parameter("goal_offset", 0.3);             // 目标点距离敌人的偏移(不要撞上去)
        declare_parameter("enable_navigation", true);      // 是否启用导航(调试时可关闭)

        // 获取参数
        gimbal_frame_ = get_parameter("gimbal_frame").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        min_distance_ = get_parameter("min_tracking_distance").as_double();
        max_distance_ = get_parameter("max_tracking_distance").as_double();
        goal_update_interval_ = get_parameter("goal_update_interval").as_double();
        goal_offset_ = get_parameter("goal_offset").as_double();
        enable_navigation_ = get_parameter("enable_navigation").as_bool();

        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Action客户端
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 可视化发布器
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/enemy_goal_marker", 10);

        // 订阅敌人距离
        enemy_sub_ = create_subscription<std_msgs::msg::Float32>(
            get_parameter("enemy_distance_topic").as_string(), 10,
            std::bind(&EnemyTrackingNode::enemyDistanceCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Enemy tracking node started");
    }

private:
    void enemyDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        double distance = msg->data;

        // 距离检查
        if (distance <= 0 || distance < min_distance_ || distance > max_distance_) {
            RCLCPP_DEBUG(get_logger(), "Distance %.2f out of range [%.2f, %.2f]",
                         distance, min_distance_, max_distance_);
            return;
        }

        // 更新频率限制
        auto now = get_clock()->now();
        if ((now - last_goal_time_).seconds() < goal_update_interval_) {
            return;
        }

        // 构造gimbal坐标系下的敌人位置
        geometry_msgs::msg::PoseStamped enemy_pose_gimbal;
        enemy_pose_gimbal.header.frame_id = gimbal_frame_;
        enemy_pose_gimbal.header.stamp = now;
        enemy_pose_gimbal.pose.position.x = distance - goal_offset_;  // 留一点距离
        enemy_pose_gimbal.pose.position.y = 0.0;
        enemy_pose_gimbal.pose.position.z = 0.0;
        enemy_pose_gimbal.pose.orientation.w = 1.0;

        // 转换到map坐标系
        geometry_msgs::msg::PoseStamped enemy_pose_map;
        try {
            enemy_pose_map = tf_buffer_->transform(enemy_pose_gimbal, map_frame_);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
            return;
        }

        // 发布可视化
        publishMarker(enemy_pose_map);

        // 发送导航目标
        if (enable_navigation_) {
            sendNavigationGoal(enemy_pose_map);
        }
        last_goal_time_ = now;
    }

    void publishMarker(const geometry_msgs::msg::PoseStamped& pose) {
        visualization_msgs::msg::Marker marker;
        marker.header = pose.header;
        marker.ns = "enemy_goal";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose.pose;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        marker_pub_->publish(marker);
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
            [this](const GoalHandleNav::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Navigation succeeded");
                }
            };

        nav_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(get_logger(), "Sent nav goal: (%.2f, %.2f)",
                    goal_pose.pose.position.x, goal_pose.pose.position.y);
    }

    // 参数
    std::string gimbal_frame_;
    std::string map_frame_;
    double min_distance_;
    double max_distance_;
    double goal_update_interval_;
    double goal_offset_;
    bool enable_navigation_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Action客户端
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    // 订阅者
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr enemy_sub_;

    // 发布者
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // 状态
    rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnemyTrackingNode>());
    rclcpp::shutdown();
    return 0;
}
