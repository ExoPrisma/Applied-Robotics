#include <pinocchio/fwd.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <highlevel_msgs/SetKD.h>
#include <std_msgs/Float64MultiArray.h>

// Controller Class
class ControllerNode
{
	public:
	
		// Pinocchio
    pinocchio::Model model;
    pinocchio::Data data;

		ControllerNode()
		{
			if (!getURDF()) return;

			getPublishRate();
			getStiffness();
			getDamping();
			getSubscriber();
			getPublisher();

			// Pinocchio
      model = pinocchio::Model();
      pinocchio::urdf::buildModel(urdf_filename, model);
      data = pinocchio::Data(model);

			end_effector_id = model.getJointId("bracelet_link") - 1;
			dim_joints = model.nq;

			// Subscribers
			feedback_subscriber = node_handle.subscribe(joint_states_topic, 3, &ControllerNode::feedbackCallback, this);
    	position_subscriber = node_handle.subscribe(reference_position_topic, 3, &ControllerNode::referencePositionCallback, this);
    	velocity_subscriber = node_handle.subscribe(reference_velocity_topic, 3, &ControllerNode::referenceVelocityCallback, this);

			// Servers
			set_joint_server = node_handle.advertiseService("/joint_controller/set_joint_pd", &ControllerNode::setJointPDCallback, this);		

			// Publishers
			torque_publisher = node_handle.advertise<std_msgs::Float64MultiArray>(joint_torque_topic, 3);
		}

		// Run method
		void run() 
		{
			ros::Rate loop_rate(publish_rate);

      while(ros::ok())
      {
				if (received_feedback && received_reference_position && received_reference_velocity)
				{
					publishTorque();	
				}
        ros::spinOnce();
        loop_rate.sleep();
      }
		}
		
		bool setJointPDCallback(highlevel_msgs::SetKD::Request &req,
													 highlevel_msgs::SetKD::Response &res)
		{
			if(req.k < 0)
      {
        ROS_WARN("Invalid target stiffness: k must be >= 0");
        return false;
      }
			if(req.d < 0)
      {
        ROS_WARN("Invalid target damping: d must be >= 0");
        return false;
      }

			k = req.k;
			d = req.d;

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

		// End effector joint ID 
    int end_effector_id;
		int dim_joints;

		// Private variables
		bool received_feedback = false;
		bool received_reference_position = false;
		bool received_reference_velocity = false;

		Eigen::MatrixXd M = Eigen::MatrixXd::Identity(dim_joints,dim_joints) ;
		Eigen::VectorXd h = Eigen::VectorXd::Zero(dim_joints) ;

		Eigen::VectorXd q_fbk;
		Eigen::VectorXd q_dot_fbk;
		Eigen::VectorXd q_ref;
		Eigen::VectorXd q_dot_ref;

		// Feedback callback
		void feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg)
		{
			q_fbk = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size());
		
			q_dot_fbk = Eigen::Map<const Eigen::VectorXd>(msg->velocity.data(), msg->velocity.size());	
		
			pinocchio::computeAllTerms(model, data, q_fbk, q_dot_fbk);
			M  = data.M;
			h  = data.nle;
			
			received_feedback = true;
		}

		// Reference position callback
		void referencePositionCallback(const std_msgs::Float64MultiArray& msg)
		{
			q_ref = Eigen::Map<const Eigen::VectorXd>(msg.data.data(), msg.data.size());
			received_reference_position = true;
		}
		
		// Reference velocity callback
		void referenceVelocityCallback(const std_msgs::Float64MultiArray& msg)
		{
			q_dot_ref = Eigen::Map<const Eigen::VectorXd>(msg.data.data(), msg.data.size());
			received_reference_velocity = true;
		}
		
		// Publish 
		void publishTorque() 
		{	
			Eigen::VectorXd q_dot2_cmd = (d * (q_dot_ref - q_dot_fbk)) + (k * (q_ref - q_fbk));

			Eigen::VectorXd tau_cmd = (M * q_dot2_cmd) + h;

			std_msgs::Float64MultiArray torque_msg;
			torque_msg.data.resize(tau_cmd.size());
			for (int i = 0; i < tau_cmd.size(); ++i) {
				torque_msg.data[i] = tau_cmd[i];
			}

			torque_publisher.publish(torque_msg);
		}

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

		int getStiffness()
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
        reference_velocity_topic = "/gen3/reference/velocity";
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
