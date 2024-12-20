#include <ros/ros.h>
#include <Eigen/Dense>
#include <highlevel_msgs/PoseCommandAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

class ActionClient
{
    public:
        ActionClient() :
            pose_action_client("/gen3/action_planner/pose", true),
            gripper_action_client("/gen3/finger_group_action_controller/gripper_cmd", true)

        {   
            // Wait for the action server to start
            ROS_INFO("Waiting for pose action server to start...");
            pose_action_client.waitForServer();
            ROS_INFO("Pose action server started!");

            ROS_INFO("Waiting for gripper action server to start...");
            gripper_action_client.waitForServer();
            ROS_INFO("Gripper action server started!");
        }

        void sendPoseCommands()
        {
            int num_targets;
            if (!ros::param::get("/action_list/number_of_targets", num_targets))
            {
                ROS_ERROR("Failed to get number of targets from parameter server.");
                return;
            }

            for (int i = 0; i < num_targets; ++i)
            {
                std::string param_name = "/action_list/action_" + std::to_string(i);

                std::string action_type;
                if (!ros::param::get(param_name + "/action_type", action_type))
                {
                    ROS_ERROR("Failed to get action type for action %d.", i);
                    continue;
                }

                if (action_type == "pose")
                {
                    handlePoseAction(param_name);
                }
                else if (action_type == "gripper")
                {
                    handleGripperAction(param_name);
                }
                else
                {
                    ROS_ERROR("Unknown action type: %s", action_type.c_str());
                }
            }
            
            ROS_INFO_STREAM("Done, shutdown") ;
	        ros::shutdown();
        }

    private:
		// Node handle
		ros::NodeHandle node_handle;

        // Private server
        actionlib::SimpleActionClient<highlevel_msgs::PoseCommandAction> pose_action_client;
        actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client;

        void handlePoseAction(const std::string& param_name)
        {
            std::vector<double> target_translation;
            std::vector<double> target_orientation;
            double target_duration;

            if (!ros::param::get(param_name + "/translation", target_translation) ||
                !ros::param::get(param_name + "/orientation", target_orientation) ||
                !ros::param::get(param_name + "/duration", target_duration))
            {
                ROS_ERROR("Failed to get parameters for pose action %s.", param_name.c_str());
                return;
            }

            highlevel_msgs::PoseCommandGoal command;

            // Create action messages
            command.x = target_translation[0];
            command.y = target_translation[1];
            command.z = target_translation[2];

            command.roll = target_orientation[0];
            command.pitch = target_orientation[1];
            command.yaw = target_orientation[2];

            command.T = target_duration;

            ROS_INFO("Sending pose goal to action server: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, duration: %f]",
                    command.x, command.y, command.z, command.roll, command.pitch, command.yaw, command.T);

            pose_action_client.sendGoal(command);

            // Wait for the result
            ROS_INFO_STREAM("Sent pose action goal. Waiting for the results...") ;
            pose_action_client.waitForResult(ros::Duration(30.0));

            if (pose_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Successfully completed pose action.");
            }
            else
            {
                ROS_ERROR("Failed to complete pose action.");
            }
        }

        void handleGripperAction(const std::string& param_name)
        {
            double position, effort, duration;

            if (!ros::param::get(param_name + "/position", position) ||
                !ros::param::get(param_name + "/effort", effort) ||
                !ros::param::get(param_name + "/duration", duration))
            {
                ROS_ERROR("Failed to get parameters for gripper action %s.", param_name.c_str());
                return;
            }

            control_msgs::GripperCommandGoal command;

            // Create command action
            command.command.position = position;
            command.command.max_effort = effort;

            ROS_INFO("Sending gripper goal to action server: [position: %f, effort: %f]", position, effort);

            gripper_action_client.sendGoal(command);
            
            // Wait for the result
            ROS_INFO_STREAM("Sent handle action goal. Waiting for the results...") ;
            gripper_action_client.waitForResult(ros::Duration(duration));

            if (gripper_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Successfully completed gripper action.");
            }
            else
            {
                ROS_ERROR("Failed to complete gripper action.");
            }
        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "action_client");

    ActionClient client;
    client.sendPoseCommands();

    return 0;
}