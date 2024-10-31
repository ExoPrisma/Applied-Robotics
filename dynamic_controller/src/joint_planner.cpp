#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>

// Controller Class
class ControllerNode
{
	public:
		if (!getURDF()) return;

		getPublishRate();
		getStiffness();
		getDamping();
		getSubscriber();
		getPublisher();

		// Subscribers
		feedback_subscriber = node_handle.subscribe(joint_states_topic, 3, &ControllerNode::feedbackCallBack, this);
    position_subscriber = node_handle.subscribe(reference_pose_topic, 3, &ControllerNode::referencePoseCallBack, this);
    velocity_subscriber = node_handle.subscribe(reference_velocity_topic, 3, &ControllerNode::referenceVelocityCallBack, this);

		set_joint_server = node_handle.advertiseService("/joint_controller/set_joint_pd", &PlannerNode::setJointPDCallBack, this);		
		// Publishers
		torque_publisher = node_handle.advertise<geometry_msgs::Pose>(feedback_pose_topic, 3);

		// Run method
		int run() 
		{
			ros::Rate loop_rate(publish_rate);

      while(ros::ok())
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
		}
		
		int setJointPDCallBack(highlevel_msgs::setKD::Request &req)
		{
			if(req.k < 0)
      {
        ROS_WARN("Invalid target stiffness: k must be >= 0");
        res.success = false;
        return false;
      }
			if(req.d < 0)
      {
        ROS_WARN("Invalid target damping: d must be >= 0");
        res.success = false;
        return false;
      }

			k = req.k;
			d = req.d;

			res.success = true;
			return true;
		}

	private:
		// Node handle
		ros::NodeHandle node_handle;

		// Private subscribers/publishers
    ros::Subscriber feedback_subscriber;
    ros::Subscriber position_subscriber;
		ros::Subscriber velocity_subscriber;
		ros::ServiceServer set_joint_server;
    ros::Publisher torque_publisher;

		// Private parameters
		std::string urdf_filename;

		double publish_rate;
		double k;
		double d;

		std::string joint_states_topic, reference_position_topic, reference_velocity_topic;
		std::string joint_torque_topic;

		// Get parameter helper functions
		bool getURDF()
		{
      if (!node_handle.getParam("/gen3/urdf_file_name", urdf_filename))
      {
        ROS_WARN("URDF filename not set.");
        return false;
      }
			return true;
		}

		int getPublishRate() 
		{
			if (!node_handle.getParam("/publish_rate", publish_rate))
      {
        ROS_WARN("Publish rate not set. Defaulting to 500 Hz.");
        publish_rate = 500.0;
				return 1;
      }
			return 0;	
		}

		int getStifness()
		{
			if (!node_handle.getParam("/stiffness", k))
      {
        ROS_WARN("Stiffness is not set.");
				return 1;
      }
			return 0;
		}

		int getDamping()
		{
			if (!node_handle.getParam("/damping", d))
			{
				ROS_WARN("Damping is not set.");
				return 1;
			}
			return 0;
		}

		int getSubscriber()
		{
			if (!node_handle.getParam("/joint_states_topic", joint_states_topic))
      {
        ROS_WARN("Joint states topic not set. Using default.");
        joint_states_topic = "/gen3/joint_states";
				return 1;
      }
      if (!node_handle.getParam("/reference_position_topic", reference_position_topic))
      {
        ROS_WARN("Reference pose topic not set. Using default.");
        reference_position_topic = "/gen3/reference/position";
				return 1;
      }
      if (!node_handle.getParam("/reference_velocity_topic", reference_velocity_topic))
      {
        ROS_WARN("Reference velocity topic not set. Using default.");
        reference_pose_topic = "/gen3/reference/velocity";
        return 1;
      }

			return 0;
		}

		int getPublisher() 
		{
			if (!node_handle.getParam("/joint_group_controller_command", joint_torque_topic))
      {
        ROS_WARN("Feedback pose topic not set. Using default.");
        joint_torque_topic = "/gen3/joint_group_effort_controller/command";
				return 1;
      }
			return 0;
		}

};

// Main method
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_controller");

  ControllerNode controller;
  controller.run();

  return 0;
}
