#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <fstream>

struct FileWriter
{
    explicit FileWriter(ros::NodeHandle& nh)
      : targets_sub_{nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, &FileWriter::targets_sub_callback, this)}
      , poses_feedback_pub_{nh.advertise<geometry_msgs::PoseArray>("feedback", 10)}
      , file_cmd_{}
      , poses_{}
      , poses_feedback_timer_{nh.createTimer(ros::Rate{50.0}, &FileWriter::poses_feedback_callback, this)}
    {
        open_file(nh);
        poses_.header.frame_id="map";
    }

    ~FileWriter()
    {
        if (file_cmd_.is_open())
            file_cmd_.close();
    }  

    void targets_sub_callback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        tf::Quaternion q{
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w,
        };
        tf::Matrix3x3 m{q};
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        

        file_cmd_ << "MOVE " << std::fixed << std::setprecision(4) << msg->pose.position.x << " " << msg->pose.position.y << " "<< yaw << "\n";
        poses_.poses.push_back(msg->pose);
        poses_.header=msg->header;
    }

    void poses_feedback_callback(const ros::TimerEvent& event)
    {
        poses_feedback_pub_.publish(poses_);
    }

    void open_file(ros::NodeHandle& nh)
    {
        std::string filename_abs {ros::package::getPath("uvone_robot_navigation")+"/paths/default.txt"};
        if(!nh.getParam("path_filename", filename_abs))
            ROS_WARN("No filename path. Default is: %s", filename_abs.c_str());
            
        file_cmd_.open(filename_abs);
        if(!file_cmd_)
            ROS_ERROR("Cannot open file: %s", filename_abs.c_str());
    }

private:
    ros::Subscriber targets_sub_;
    ros::Publisher poses_feedback_pub_;
    std::ofstream file_cmd_;
    geometry_msgs::PoseArray poses_;
    ros::Timer poses_feedback_timer_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_writer");
    ros::NodeHandle nh{"~"};

    FileWriter a{nh};
    ROS_INFO("Node ready to listen poses to create command file.");
    ROS_INFO("Press Ctrl+C to write poses file and exit.");

    ros::spin();

    return 0;
}