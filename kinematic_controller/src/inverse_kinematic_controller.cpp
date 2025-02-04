#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>


#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Controller Class
class ControllerNode
{
	public:

		// Pinocchio
		pinocchio::Model model;
    	pinocchio::Data data;

		ControllerNode()
		{
      		if(!getURDF()) return;

			getPublishRate();
			getKAtt();
			getMaxVelocity();
			getWithRedundancy();
			getWithOrientation();
			getSubscriber();
			getPublisher();

			// Pinocchio
			model = pinocchio::Model();
			pinocchio::urdf::buildModel(urdf_filename, model);
      		data = pinocchio::Data(model);

			end_effector_id = model.getJointId("bracelet_link") - 1;

			// Subscribers
			joint_subscriber = node_handle.subscribe(joint_states_topic, 3, &ControllerNode::jointRateCallBack, this);
      		pose_subscriber = node_handle.subscribe(reference_pose_topic, 3, &ControllerNode::referencePoseCallBack, this);
			home_config_subscriber = node_handle.subscribe(reference_velocity_topic, 3, &ControllerNode::referenceVelocityCallBack, this);

			// Publishers
			pose_publisher = node_handle.advertise<geometry_msgs::Pose>(feedback_pose_topic, 3);
			twist_publisher = node_handle.advertise<geometry_msgs::Twist>(feedback_twist_topic, 3);	
			velocity_publisher = node_handle.advertise<std_msgs::Float64MultiArray>(joint_group_controller_command_topic, 3);
		}		
		
		// Run method
		void run()
		{
			ros::Rate loop_rate(publish_rate);

			while(ros::ok())
			{
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

	private:
		
		// Node handle
		ros::NodeHandle node_handle;

		// Private subscribers/publishers
		ros::Subscriber joint_subscriber;
		ros::Subscriber pose_subscriber;
		ros::Subscriber home_config_subscriber;
		ros::Publisher pose_publisher;
		ros::Publisher twist_publisher;
		ros::Publisher velocity_publisher;

		// Private parameters
		std::string urdf_filename;

		double publish_rate;
    	double linear_k_att;
		double angular_k_att;
		double linear_max_velocity;
		double angular_max_velocity;

		bool with_redundancy;
		bool with_orientation;

		std::string joint_states_topic, reference_pose_topic, reference_velocity_topic;
		std::string feedback_pose_topic, feedback_twist_topic, joint_group_controller_command_topic;

		// End effector joint ID 
		int end_effector_id;
		
		// Private variables
		bool received_feedback_joint = false;
		bool received_reference_velocity = false;
	
		// Feedback pose & twist
		geometry_msgs::Pose fbk_pose_msg;
  		geometry_msgs::Twist fbk_twist_msg;

		geometry_msgs::Pose ref_pose_msg;

		Eigen::VectorXd joint_position;
		Eigen::VectorXd secondary_joint_velocities;
 
		// Joint Rate callback function
		void jointRateCallBack(const sensor_msgs::JointState::ConstPtr& msg)
		{
			received_feedback_joint = true;

			joint_position = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size() - 1);
					
			Eigen::VectorXd joint_velocity = Eigen::Map<const Eigen::VectorXd>(msg->velocity.data(), msg->velocity.size() - 1);

			computeKinematics(joint_velocity);
		}

		// Joint rate helper function
		void computeKinematics(const Eigen::VectorXd& joint_velocity)
		{
			// Compute all kinematic terms
			pinocchio::computeAllTerms(model, data, joint_position, joint_velocity);

			// Compute the end-effector pose
			pinocchio::SE3 end_effector_pose = data.oMi[end_effector_id];

			// Set pose
			fbk_pose_msg.position.x = end_effector_pose.translation().x();
			fbk_pose_msg.position.y = end_effector_pose.translation().y();
			fbk_pose_msg.position.z = end_effector_pose.translation().z();

			// Compute the end-effector pose orientation
			Eigen::Quaterniond quaternion;
			pinocchio::quaternion::assignQuaternion(quaternion, end_effector_pose.rotation());

			fbk_pose_msg.orientation.x = quaternion.x();
			fbk_pose_msg.orientation.y = quaternion.y();
			fbk_pose_msg.orientation.z = quaternion.z();
			fbk_pose_msg.orientation.w = quaternion.w();

			// ROS_INFO("Pose Position - x: %f, y: %f, z: %f",
			//  fbk_pose_msg.position.x,
			//  fbk_pose_msg.position.y,
			//  fbk_pose_msg.position.z);

			// double roll, pitch, yaw;
			// tf2::Quaternion quaternion_tf;
			// quaternion_tf = tf2::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
			// ROS_WARN("End-effector pose quaternion - x: %f, y: %f, z: %f, w:%f", quaternion_tf.x(), quaternion_tf.y(), quaternion_tf.z(), quaternion_tf.w());
			
			// tf2::Matrix3x3(quaternion_tf).getRPY(roll, pitch, yaw);
			// ROS_WARN("End-effector pose Euler Angles - roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
				
			pose_publisher.publish(fbk_pose_msg);

			// Compute the end-effector twist
			Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);

			pinocchio::getJointJacobian(model, data, end_effector_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);

			Eigen::VectorXd end_effector_twist = jacobian * joint_velocity;
				
			// Set twist
			fbk_twist_msg.linear.x = end_effector_twist[0];
			fbk_twist_msg.linear.y = end_effector_twist[1];
			fbk_twist_msg.linear.z = end_effector_twist[2];

			fbk_twist_msg.angular.x = end_effector_twist[3];	
			fbk_twist_msg.angular.y = end_effector_twist[4];
			fbk_twist_msg.angular.z = end_effector_twist[5];

			// ROS_INFO("Twist Position - x: %f, y: %f, z: %f",
			// 	fbk_twist_msg.linear.x,
			// 	fbk_twist_msg.linear.y,
			// 	fbk_twist_msg.linear.z);

			// ROS_INFO("Twist Orientation - x: %f, y: %f, z: %f",
			// 	fbk_twist_msg.angular.x,
			// 	fbk_twist_msg.angular.y,
			// 	fbk_twist_msg.angular.z);

			// ROS_INFO("-----------------------------------------------");

			twist_publisher.publish(fbk_twist_msg);
		}
		
		// Reference pose callback function
		void referencePoseCallBack(const geometry_msgs::Pose::ConstPtr& msg)
		{
			Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(3);
			Eigen::VectorXd x_fbk = Eigen::VectorXd::Zero(3);
			Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);

      		pinocchio::getJointJacobian(model, data, end_effector_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);

			ref_pose_msg = *msg;

			if (!with_orientation)
			{
				jacobian = jacobian.topRows(3);
			}
	
			x_ref << ref_pose_msg.position.x,
          		 	 ref_pose_msg.position.y,
          		 	 ref_pose_msg.position.z;
			
			x_fbk << fbk_pose_msg.position.x,
					 fbk_pose_msg.position.y,
					 fbk_pose_msg.position.z;

			//ROS_INFO("x_ref: [%f, %f, %f]", x_ref(0), x_ref(1), x_ref(2));
			//ROS_INFO("x_fbk: [%f, %f, %f]", x_fbk(0), x_fbk(1), x_fbk(2));

			if (received_feedback_joint)
			{	
				if (with_redundancy) 
				{
					if (received_reference_velocity)
					{
						computeJointVelocityWithRedundancy(x_ref, x_fbk, jacobian);
					}
				}
				else
				{
					computeJointVelocity(x_ref, x_fbk, jacobian);
				}
			}
    	}

		Eigen::VectorXd computeVRef(Eigen::VectorXd& v_ref)
		{
			v_ref.conservativeResize(6);	

			pinocchio::SE3 pose_fbk = data.oMi[end_effector_id];

			Eigen::Quaterniond quaternion;
			quaternion.x() = ref_pose_msg.orientation.x;
			quaternion.y() = ref_pose_msg.orientation.y;
			quaternion.z() = ref_pose_msg.orientation.z;
			quaternion.w() = ref_pose_msg.orientation.w;

    		Eigen::Matrix3d pose_ref_rotation = quaternion.toRotationMatrix();

			// calculating orientation error from rotation matrix
			Eigen::Matrix3d rot_err_ 	  = pose_fbk.rotation().transpose() * pose_ref_rotation ; // difference in rotation
			Eigen::Vector3d ang_vel_local = pinocchio::log3(rot_err_) ; 							// convert it to a vector in local frame
			Eigen::Vector3d ang_vel_world = pose_fbk.rotation() * ang_vel_local ; 					// convert it to the world frame
			//ROS_INFO_STREAM("orientation error from pinocchio rotation " << ang_vel_world.transpose() );
			
			ang_vel_world = angular_k_att * ang_vel_world;

			if (ang_vel_world.norm() > angular_max_velocity)
			{
				ang_vel_world = ang_vel_world * (angular_max_velocity / ang_vel_world.norm());
			}

			v_ref.tail(3) = ang_vel_world;

			return v_ref;
		}
		
		// Reference pose helper function
		void computeJointVelocity(const Eigen::VectorXd& x_ref, const Eigen::VectorXd& x_fbk, const Eigen::MatrixXd& jacobian)
		{
      		Eigen::VectorXd v_ref = linear_k_att * (x_ref - x_fbk);
			
			if (v_ref.norm() > linear_max_velocity)
			{
				v_ref = v_ref * (linear_max_velocity / v_ref.norm());
			}

			if (with_orientation)
			{
				computeVRef(v_ref);
			}

			Eigen::MatrixXd jacobian_pseudo_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
			Eigen::VectorXd joint_velocities = jacobian_pseudo_inv * v_ref;
			Eigen::VectorXd joint_positions = joint_position + (joint_velocities * (1.0 / publish_rate));	

			std_msgs::Float64MultiArray joint_velocities_msg;
			joint_velocities_msg.data.resize(joint_positions.size());
			for (int i = 0; i < joint_positions.size(); ++i) {
				joint_velocities_msg.data[i] = joint_positions[i];
			}

			velocity_publisher.publish(joint_velocities_msg);	
		}

		// Reference pose helper function
		void computeJointVelocityWithRedundancy(const Eigen::VectorXd& x_ref, const Eigen::VectorXd& x_fbk, const Eigen::MatrixXd& jacobian)
		{	
			Eigen::VectorXd v_ref = linear_k_att * (x_ref - x_fbk);
			
			if (v_ref.norm() > linear_max_velocity)
			{
				v_ref = v_ref * (linear_max_velocity / v_ref.norm());
			}

			if (with_orientation)
			{
				computeVRef(v_ref);
			}

      		Eigen::MatrixXd jacobian_pseudo_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

			Eigen::VectorXd primary_joint_velocities = jacobian_pseudo_inv * v_ref;

			Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols());
  			Eigen::MatrixXd null_space_projector = identity - jacobian_pseudo_inv * jacobian;

			Eigen::VectorXd joint_velocities = primary_joint_velocities + (null_space_projector * secondary_joint_velocities);

      		Eigen::VectorXd joint_positions = joint_position + (joint_velocities * (1.0 / publish_rate));
			
			std_msgs::Float64MultiArray joint_velocities_msg;
			joint_velocities_msg.data.resize(joint_positions.size());
			for (int i = 0; i < joint_positions.size(); ++i) {
				joint_velocities_msg.data[i] = joint_positions[i];
			}

			velocity_publisher.publish(joint_velocities_msg);
		}

		void referenceVelocityCallBack(const std_msgs::Float64MultiArray& msg)
		{
	    	secondary_joint_velocities = Eigen::VectorXd::Map(msg.data.data(), msg.data.size());
			received_reference_velocity = true;
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

		int getKAtt()
		{
			if (!node_handle.getParam("/gen3/linear/k_att", linear_k_att))
			{
				ROS_WARN("Linear k_att not set.");
				return 1;
			}
			if (!node_handle.getParam("/gen3/angular/k_att", angular_k_att))
			{
				ROS_WARN("Angular k_att not set.");
				return 1;
			}	
			return 0;
		}

		int getMaxVelocity()
		{
			if (!node_handle.getParam("/gen3/linear/max_velocity", linear_max_velocity))
			{
				ROS_WARN("Max linear velocity not set.");
				return 1;
			}
			if (!node_handle.getParam("/gen3/angular/max_velocity", angular_max_velocity))
			{
				ROS_WARN("Max angular velocity not set.");
				return 1;
			}
			return 0;
		}
		
		int getWithRedundancy()
		{
			if (!node_handle.getParam("/controller/with_redundancy", with_redundancy))
			{
				ROS_WARN("With redundancy set to false.");
				with_redundancy = false;
				return 1;
			}
			return 0;
		}

		int getWithOrientation()
		{
			if (!node_handle.getParam("/controller/with_orientation", with_orientation))
			{
				ROS_WARN("With orientation set to false.");
				with_orientation = false;
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
			if (!node_handle.getParam("/reference_pose_topic", reference_pose_topic))
			{
				ROS_WARN("Reference pose topic not set. Using default.");
				reference_pose_topic = "/gen3/reference/pose";
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
			if (!node_handle.getParam("/feedback_pose_topic", feedback_pose_topic))
			{
				ROS_WARN("Feedback pose topic not set. Using default.");
				feedback_pose_topic = "/gen3/feedback/pose";
				return 1;
			}
			if (!node_handle.getParam("/feedback_twist_topic", feedback_twist_topic))
			{
				ROS_WARN("Feedback twist topic not set. Using default.");
				feedback_twist_topic = "/gen3/feedback/twist";
				return 1;
			}
			if (!node_handle.getParam("/joint_group_controller_command_topic", joint_group_controller_command_topic))
			{
				ROS_WARN("Joint group position controller command topic not set. Using default.");
				joint_group_controller_command_topic = "/gen3/joint_group_position_controller/command";
				return 1;
			}

			return 0;
		}

};

// Main method
int main(int argc, char** argv)
{
  ros::init(argc, argv, "inverse_kinematic_controller");

  ControllerNode controller;
  controller.run();

  return 0;
}

