#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <thread>

class SetPoseAndStaticTFNode : public rclcpp::Node
{
public:
    SetPoseAndStaticTFNode()
        : Node("set_pose_and_static_tf_node")
    {
        // Create publisher for setting pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/set_pose", 10);

        // Create publisher for static transforms
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Wait for 2 seconds
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Publish the static transform from odom to chassis
        publishChassisToGPSLinkTransform();

        // Publish the static transform from chassis to gps_link
        publishChassisToGPSLinkTransform();

        // Publish the static transform from odom to imu_link
        publishOdomToIMULinkTransform();
        
        publishOdomToIMULinkTransform2();

        // Set pose once
        setInitialPose();
    }

private:
    // Publishes a static transform from chassis to gps_link
    void publishChassisToGPSLinkTransform()
    {	
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "gps_link";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, 0.0);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();
        static_tf_broadcaster_->sendTransform(transformStamped);
    }

    // Publishes a static transform from odom to imu_link
    void publishOdomToIMULinkTransform()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "imu_link";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, 0.0);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();
        static_tf_broadcaster_->sendTransform(transformStamped);
    }
    
    void publishOdomToIMULinkTransform2()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "odom";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, 0.0);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();
        static_tf_broadcaster_->sendTransform(transformStamped);
    }
    // Sets the initial pose
    void setInitialPose()
    {
        auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        
        msg->pose.pose.position.x = 1.0;
        msg->pose.pose.position.y = -1.0;
        msg->pose.pose.position.z = 0.0;
        msg->pose.pose.orientation.x = 0.0;
        msg->pose.pose.orientation.y = 0.0;
        msg->pose.pose.orientation.z = 0.0;
        msg->pose.pose.orientation.w = 1.0;
        pose_publisher_->publish(*msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetPoseAndStaticTFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

