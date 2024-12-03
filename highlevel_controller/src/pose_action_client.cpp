#include <ros/ros.h>
#include <Eigen/Dense>
#include <highlevel_msgs/PoseCommandAction.h>
#include <actionlib/client/simple_action_client.h>

class PoseActionClient
{
    public:
        PoseActionClient() :
            action_client("/gen3/action_planner/pose", true)
        {   
            // Wait for the action server to start
            ROS_INFO("Waiting for action server to start...");
            action_client.waitForServer();
            ROS_INFO("Action server started!");
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

                std::vector<double> target_translation;
                std::vector<double> target_orientation;
                double target_duration;

                if (!ros::param::get(param_name + "/translation", target_translation) ||
                    !ros::param::get(param_name + "/orientation", target_orientation) ||
                    !ros::param::get(param_name + "/duration", target_duration))
                {
                    ROS_ERROR("Failed to get parameters for action %d from parameter server.", i);
                    continue;
                }

                // Create action messages
                highlevel_msgs::PoseCommandGoal command;

                command.x = target_translation[0];
                command.y = target_translation[1];
                command.z = target_translation[2];

                command.roll = target_orientation[0];
                command.pitch = target_orientation[1];
                command.yaw = target_orientation[2];

                command.T = target_duration;

                ROS_INFO("Sending goal to action server: [x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f, duration: %f]",
                    command.x, 
                    command.y, 
                    command.z, 
                    command.roll, 
                    command.pitch, 
                    command.yaw, 
                    command.T
                    );

                action_client.sendGoal(command);

                // Wait for the result
                ROS_INFO_STREAM("Sent action goal. Waiting for the results...") ;
                action_client.waitForResult(ros::Duration(30.0));
                
                if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Successfully reached target %d", i);
                }
                else
                {
                    ROS_ERROR("Failed to reach target %d", i);
                }
            }
            
            ROS_INFO_STREAM("Done, shotdown") ;
	        ros::shutdown();
        }

    private:

		// Node handle
		ros::NodeHandle node_handle;

        // Private server
        actionlib::SimpleActionClient<highlevel_msgs::PoseCommandAction> action_client;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_action_client");

    PoseActionClient client;
    client.sendPoseCommands();

    return 0;
}