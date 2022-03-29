#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <uvone_robot_msgs/LightCmd.h>

#include <fstream>

struct CommandSender
{
    using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

    MoveBaseClient ac;
    std::string cmd_filename;
    std::ifstream cmds;
    std::size_t cmd_line {};
    ros::Publisher pub_light;

    explicit CommandSender(ros::NodeHandle& nh)
      : ac{nh, "move_base", true}
      , cmd_filename{nh.param<std::string>("cmd_filename", "")}
      , cmds{cmd_filename}
      , pub_light{nh.advertise<uvone_robot_msgs::LightCmd>("cmd_light", 400)}
    {
        if (cmd_filename.empty())
        {
            ROS_ERROR("cmd_filename param is not set. Setting default.txt");
            throw std::runtime_error("cmd_filename param is not set. Setting default.txt");
        }

        if (!cmds.is_open())
        {
            ROS_ERROR("failed to open %s.", cmd_filename.c_str());
            throw std::runtime_error("failed to open" + cmd_filename + ".");
        }

        ROS_INFO("Reading commands from %s.", cmd_filename.c_str());
        while(!ac.waitForServer(ros::Duration(5.0)) && ros::ok()){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    }

    bool execute()
    {
        std::string instruction;
        cmds >> instruction;
        ++cmd_line;

        if (!ros::ok() || !ac.waitForServer())
            return false;
        
        if(instruction == "MOVE")
            return execute_move();
        else if(instruction == "WAIT")
            return execute_wait();
        else if(instruction == "LIGHT")
            return execute_light();
        else
            return false;
    }

    bool execute_move()
    {
        float x, y, theta;
        if(! (cmds >> x >> y >> theta) )
        {
            ROS_ERROR("Error reading move instruction. Line %lu", cmd_line);
            return false;
        }
        ROS_INFO("Sending x:%6.3f y:%6.3f theta:%6.3f", x, y, theta);

        tf2::Quaternion myQ;

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        myQ.setRPY(0, 0, theta);

        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        tf2::convert(myQ, goal.target_pose.pose.orientation);

        ac.sendGoal(goal);

        ac.waitForResult();

        ros::Duration(0.1).sleep();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Base arrived successfully");
            return true;
        }
    
        ROS_WARN("The base failed for some reason");
        return false;
    }

    bool execute_wait()
    {
        double dur;
        if(! (cmds >> dur) )
        {
            ROS_ERROR("Error reading wait instruction. Line %lu", cmd_line);
            return false;
        }
        ROS_INFO("Waiting t:%6.3f seconds", dur);
        ros::Duration(dur).sleep();
        return true;
    }

    bool execute_light()
    {
        std::string type;
        if(! (cmds >> type) )
        {
            ROS_ERROR("Error reading light instruction. Line %lu", cmd_line);
            return false;
        }
        static uvone_robot_msgs::LightCmd msg {};
        ROS_INFO("Light instruction. Command", type);
        if (type == "OFF") {
            ROS_INFO("Turning off inverter");
            msg.inverter = false;
        }
        else if (type == "ON")
        {
            ROS_INFO("Turning on inverter");
            msg.inverter = true;
        }
        else if (type == "NONE")
        {
            ROS_INFO("Selecting none lamp");
            msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_NONE_LAMP;
        }
        else if (type == "LEFT")
        {
            ROS_INFO("Selecting none lamp");
            msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_LEFT_LAMP;
        }
        else if (type == "RIGHT")
        {
            ROS_INFO("Selecting none lamp");
            msg.lamp_selected = uvone_robot_msgs::LightCmd::SELECT_RIGHT_LAMP;
        }
        else
        {
            ROS_ERROR("Wrong light instruction. Line %lu", cmd_line);
            return false;
        }
        pub_light.publish(msg);
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_sender");
    ros::NodeHandle nh("~");

    CommandSender cmd{nh};

    while( cmd.execute() );

    ROS_INFO("Finished commands");
    ros::shutdown();

    return 0;
}