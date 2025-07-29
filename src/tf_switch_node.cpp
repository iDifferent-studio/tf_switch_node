#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/parameter_client.hpp>

class TfSwitchNode : public rclcpp::Node
{
public:
    TfSwitchNode() : Node("tf_switch_node")
    {
        // Declare parameters with default values
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("odom_frame_id", "odom");
        this->declare_parameter("base_link_frame_id", "base_link");
        this->declare_parameter("amcl_node_name", "amcl");
        this->declare_parameter("publish_tf_param_name", "tf_broadcast");
        this->declare_parameter("covariance_threshold", 1.5);

        // Get parameters
        map_frame_id_ = this->get_parameter("map_frame_id").as_string();
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        base_link_frame_id_ = this->get_parameter("base_link_frame_id").as_string();
        amcl_node_name_ = this->get_parameter("amcl_node_name").as_string();
        publish_tf_param_name_ = this->get_parameter("publish_tf_param_name").as_string();
        covariance_threshold_ = this->get_parameter("covariance_threshold").as_double();

        RCLCPP_DEBUG(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Initialized with parameters:");
        RCLCPP_INFO(this->get_logger(), "- map_frame_id: %s", map_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "- odom_frame_id: %s", odom_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "- base_link_frame_id: %s", base_link_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "- amcl_node_name: %s", amcl_node_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "- publish_tf_param_name: %s", publish_tf_param_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "- covariance_threshold: %.3f", covariance_threshold_);
        RCLCPP_DEBUG(this->get_logger(), "----------------------------------------");

        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        amcl_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, amcl_node_name_); 
        
        // Create a timer to publish transforms at a regular interval
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TfSwitchNode::timerCallback, this)
        );

        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix", 10,
            std::bind(&TfSwitchNode::gnssCallback, this, std::placeholders::_1)
        );

        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10,
            std::bind(&TfSwitchNode::amclPoseCallback, this, std::placeholders::_1)
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/global", 10,
            std::bind(&TfSwitchNode::odomCallback, this, std::placeholders::_1)
        );

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 1);
    }

private:
    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;
    std::string amcl_node_name_;
    std::string publish_tf_param_name_; 
    double covariance_threshold_;

    bool send_ekf_transforms_ = false;
    bool send_amcl_transforms_ = true;

    int8_t gnss_status = 0;
    bool amcl_covariance_valid = false;

    enum class LocalizationState {
        DEAD_RECKON,  
        RTK_BAD,      
        RTK_GOOD      
    };
    LocalizationState current_state_ = LocalizationState::DEAD_RECKON;

    nav_msgs::msg::Odometry::SharedPtr latest_odom_msg_;

    const int WAIT_CYCLES = 10;
    int initial_pose_counter_ = 0;
    
    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        gnss_status = msg->status.status;
        // RCLCPP_INFO(this->get_logger(), "Received GNSS (fix) message, status: %d", gnss_status);
    }

    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        double cov_x = msg->pose.covariance[0]; 
        double cov_y = msg->pose.covariance[7]; 

        if (cov_x < covariance_threshold_ && cov_y < covariance_threshold_) 
            amcl_covariance_valid = true;
        else
            amcl_covariance_valid = false;

        // RCLCPP_INFO(this->get_logger(), "Received amcl_pose message, cov_x: %f, cov_y: %f valid: %s", cov_x, cov_y, amcl_covariance_valid ? "true" : "false");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_msg_ = msg;
        //reference to the ekf source code.
        geometry_msgs::msg::TransformStamped world_base_link_trans_msg_;

        world_base_link_trans_msg_.header.stamp = static_cast<rclcpp::Time>(msg->header.stamp);
        world_base_link_trans_msg_.header.frame_id = msg->header.frame_id;
        world_base_link_trans_msg_.child_frame_id = msg->child_frame_id;

        world_base_link_trans_msg_.transform.translation.x = msg->pose.pose.position.x;
        world_base_link_trans_msg_.transform.translation.y = msg->pose.pose.position.y;
        world_base_link_trans_msg_.transform.translation.z = msg->pose.pose.position.z;
        world_base_link_trans_msg_.transform.rotation = msg->pose.pose.orientation;

        try 
        {
            tf2::Transform world_base_link_trans;
            tf2::fromMsg(
                world_base_link_trans_msg_.transform,
                world_base_link_trans);

            tf2::Transform base_link_odom_trans;
            tf2::fromMsg(
                tf_buffer_->lookupTransform(
                    base_link_frame_id_, 
                    odom_frame_id_,
                    tf2::TimePointZero
                ).transform,
                base_link_odom_trans
            );

            /*
            * First, see these two references:
            * http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms#lookupTransform
            * http://wiki.ros.org/geometry/CoordinateFrameConventions#Transform_Direction
            * We have a transform from map_frame_id_->base_link_frame_id_, but
            * it would actually transform a given pose from
            * base_link_frame_id_->map_frame_id_. We then used lookupTransform,
            * whose first two arguments are target frame and source frame, to
            * get a transform from base_link_frame_id_->odom_frame_id_.
            * However, this transform would actually transform data from
            * odom_frame_id_->base_link_frame_id_. Now imagine that we have a
            * position in the map_frame_id_ frame. First, we multiply it by the
            * inverse of the map_frame_id_->baseLinkFrameId, which will
            * transform that data from map_frame_id_ to base_link_frame_id_.
            * Now we want to go from base_link_frame_id_->odom_frame_id_, but
            * the transform we have takes data from
            * odom_frame_id_->base_link_frame_id_, so we need its inverse as
            * well. We have now transformed our data from map_frame_id_ to
            * odom_frame_id_. However, if we want other users to be able to do
            * the same, we need to broadcast the inverse of that entire
            * transform.
            */
            tf2::Transform map_odom_trans;
            map_odom_trans.mult(world_base_link_trans, base_link_odom_trans);

            geometry_msgs::msg::TransformStamped map_odom_trans_msg;
            map_odom_trans_msg.transform = tf2::toMsg(map_odom_trans);
            map_odom_trans_msg.header.stamp = static_cast<rclcpp::Time>(msg->header.stamp);
            map_odom_trans_msg.header.frame_id = map_frame_id_;
            map_odom_trans_msg.child_frame_id = odom_frame_id_;

            // Publish the transform
            if(send_ekf_transforms_)
            {
                broadcaster_->sendTransform(map_odom_trans_msg);
                RCLCPP_INFO(this->get_logger(), "Published transform from %s to %s",
                    map_frame_id_.c_str(), odom_frame_id_.c_str());
            }
        } 
        catch (const tf2::TransformException& ex) 
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
        catch (const std::runtime_error& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Runtime error: %s", ex.what());
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", ex.what());
        }
        catch (...) 
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown error occurred while getting transform from %s to %s",
                odom_frame_id_.c_str(), base_link_frame_id_.c_str());
        }

        // RCLCPP_INFO(this->get_logger(), "Received odometry/global message, x: %f, y: %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    void set_amcl_parameter(bool param_value)
    {
        // Check if parameter client is ready
        if (!amcl_param_client_->wait_for_service(std::chrono::seconds(1))) 
        {
            RCLCPP_ERROR(this->get_logger(), "AMCL parameter service not available after waiting");
            return;
        }

        auto parameters = std::vector<rclcpp::Parameter>{
            rclcpp::Parameter(publish_tf_param_name_, param_value)
        };
        
        RCLCPP_INFO(this->get_logger(), "Requesting to set AMCL parameter '%s' to %s", publish_tf_param_name_.c_str(), param_value ? "true" : "false");

        // Using the correct callback signature with shared_future
        amcl_param_client_->set_parameters(
            parameters,
            [this, param_value](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                try 
                {
                    auto results = future.get();
                    for (const auto & result : results) 
                    {
                        if (!result.successful) 
                        {
                            RCLCPP_INFO(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                        } 
                        else 
                        {
                            RCLCPP_INFO(this->get_logger(), "Successfully set parameter '%s' to %s", publish_tf_param_name_.c_str(), param_value ? "true" : "false");
                        }
                    }
                } 
                catch (const std::exception & e) 
                {
                    RCLCPP_INFO(this->get_logger(), "Error getting parameter result: %s", e.what());
                }
            }
        );
    }

    void setAmclInitialPose(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = this->now();
        initial_pose.header.frame_id = "map";
        initial_pose.pose.pose = msg->pose.pose;
        initial_pose.pose.covariance = msg->pose.covariance;
        
        initial_pose_pub_->publish(initial_pose);
    }    

    void timerCallback()
    {   
        LocalizationState previous_state = current_state_;

        switch (current_state_)
        {
            case LocalizationState::DEAD_RECKON:
                // Handle dead reckoning state
                if(gnss_status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
                {
                    current_state_ = LocalizationState::RTK_GOOD;
                    send_ekf_transforms_ = true;
                    set_amcl_parameter(false); 
                }
                else if(amcl_covariance_valid && gnss_status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
                {
                    current_state_ = LocalizationState::RTK_BAD;
                }
            break;

            case LocalizationState::RTK_BAD:
                // Handle RTK bad state
                if(gnss_status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
                {
                    current_state_ = LocalizationState::RTK_GOOD;
                    send_ekf_transforms_ = true;
                    set_amcl_parameter(false);
                }
                else if(!amcl_covariance_valid && gnss_status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
                {
                    current_state_ = LocalizationState::DEAD_RECKON;
                }
            break;

            case LocalizationState::RTK_GOOD:
                // Handle RTK good state
                if(amcl_covariance_valid && gnss_status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
                {
                    current_state_ = LocalizationState::RTK_BAD;
                    send_ekf_transforms_ = false;
                    set_amcl_parameter(true);
                }
                else if(!amcl_covariance_valid && gnss_status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
                {
                    current_state_ = LocalizationState::DEAD_RECKON;
                    send_ekf_transforms_ = false;
                    set_amcl_parameter(true);
                }
                else if(!amcl_covariance_valid)
                {
                    if (initial_pose_counter_ == 0) 
                    {  
                        if (latest_odom_msg_) 
                        {
                            setAmclInitialPose(latest_odom_msg_);
                            initial_pose_counter_ = WAIT_CYCLES;  
                        }
                    } 
                    else 
                    {
                        initial_pose_counter_--;
                    }
                }
            break;
        }

        if (previous_state != current_state_)
        {
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            RCLCPP_INFO(this->get_logger(), "current state %s", stateToString(current_state_).c_str());
            RCLCPP_INFO(this->get_logger(), "GNSS status: %d, AMCL valid: %s", gnss_status, amcl_covariance_valid ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        }
    }

    std::string stateToString(LocalizationState state)
    {
        switch (state)
        {
            case LocalizationState::DEAD_RECKON: return "DEAD_RECKON";
            case LocalizationState::RTK_BAD: return "RTK_BAD";
            case LocalizationState::RTK_GOOD: return "RTK_GOOD";
            default: return "UNKNOWN";
        }
    }
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<rclcpp::AsyncParametersClient> amcl_param_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfSwitchNode>());
    rclcpp::shutdown();
    return 0;
}