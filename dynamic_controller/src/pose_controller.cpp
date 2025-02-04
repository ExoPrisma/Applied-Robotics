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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <highlevel_msgs/SetKD.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <sstream>

// Control Class
class ControlNode
{
	public:
	
		// Pinocchio
    pinocchio::Model model;
    pinocchio::Data data;

		ControlNode()
		{
			if (!getURDF()) return;

			getPublishRate();
			getStiffness();
			getDamping();
			getWithRedundancy();
			getSubscriber();
			getPublisher();

			// Pinocchio
      model = pinocchio::Model();
      pinocchio::urdf::buildModel(urdf_filename, model);
      data = pinocchio::Data(model);

			model.gravity.linear() = Eigen::Vector3d(0, 0, -9.81);  
    	model.gravity.angular().setZero();     
			
			end_effector_id = model.getJointId("bracelet_link") - 1;
			dim_joints = model.nq;

			M = Eigen::MatrixXd::Identity(dim_joints, dim_joints);
			M_inverse = Eigen::MatrixXd::Identity(dim_joints, dim_joints);

			// Subscribers
			feedback_subscriber = node_handle.subscribe(joint_states_topic, 3, &ControlNode::feedbackCallback, this);
    	pose_subscriber = node_handle.subscribe(reference_pose_topic, 3, &ControlNode::referencePoseCallback, this);
    	twist_subscriber = node_handle.subscribe(reference_twist_topic, 3, &ControlNode::referenceTwistCallback, this);
    	position_subscriber = node_handle.subscribe(reference_position_topic, 3, &ControlNode::referencePositionCallback, this);
    	velocity_subscriber = node_handle.subscribe(reference_velocity_topic, 3, &ControlNode::referenceVelocityCallback, this);

			// Servers
			set_joint_server = node_handle.advertiseService("/pose_controller/set_joint_pd", &ControlNode::setJointPDCallback, this);		
			set_effector_server = node_handle.advertiseService("/pose_controller/set_linear_pd", &ControlNode::setEffectorPDCallback, this);		

			// Publishers
    	pose_publisher = node_handle.advertise<geometry_msgs::Pose>(feedback_pose_topic, 3);
    	twist_publisher = node_handle.advertise<geometry_msgs::Twist>(feedback_twist_topic, 3);
			torque_publisher = node_handle.advertise<std_msgs::Float64MultiArray>(joint_torque_topic, 3);	
		}

		// Run method
		void run() 
		{
			ros::Rate loop_rate(publish_rate);

      while(ros::ok())
      {
				if (received_feedback && 
						received_reference_pose && received_reference_twist &&
						received_reference_position && received_reference_velocity)
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

			k_joint = req.k;
			d_joint = req.d;

			return true;
		}

    bool setEffectorPDCallback(highlevel_msgs::SetKD::Request &req,
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

      k_effector = req.k;
      d_effector = req.d;

      return true;
    }


	private:
		// Node handle
		ros::NodeHandle node_handle;

		// Private subscribers/publishers
    ros::Subscriber feedback_subscriber;
    ros::Subscriber pose_subscriber;
		ros::Subscriber twist_subscriber;
    ros::Subscriber position_subscriber;
		ros::Subscriber velocity_subscriber;

		ros::ServiceServer set_joint_server;
		ros::ServiceServer set_effector_server;

    ros::Publisher pose_publisher;
		ros::Publisher twist_publisher;
    ros::Publisher torque_publisher;

		// Private parameters
		std::string urdf_filename;

		double publish_rate;
		double k_joint;
		double d_joint;
		double k_effector;
		double d_effector;

		bool with_redundancy;

		std::string joint_states_topic, reference_pose_topic, reference_twist_topic, reference_position_topic, reference_velocity_topic;
		std::string feedback_pose_topic, feedback_twist_topic, joint_torque_topic;

		// End effector joint ID 
    int end_effector_id;
		int dim_joints;

		// Private variables
		bool received_feedback = false;
		bool received_reference_pose = false;
		bool received_reference_twist = false;
		bool received_reference_position = false;
		bool received_reference_velocity = false;

		Eigen::MatrixXd M;
		Eigen::MatrixXd M_inverse;
		Eigen::VectorXd h;
		Eigen::MatrixXd J;
		Eigen::MatrixXd J_transpose;
		Eigen::MatrixXd J_pseudo;
		Eigen::MatrixXd J_transpose_pseudo;
		Eigen::MatrixXd J_dot;

		Eigen::VectorXd q_fbk = Eigen::VectorXd::Zero(7);
		Eigen::VectorXd q_dot_fbk = Eigen::VectorXd::Zero(7);
		Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(7);
		Eigen::VectorXd q_dot_ref = Eigen::VectorXd::Zero(7);

		Eigen::VectorXd x_fbk = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd x_dot_fbk = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(3);
		Eigen::VectorXd x_dot_ref = Eigen::VectorXd::Zero(3);

		// Feedback callback
		void feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg)
		{
			q_fbk = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size());
		
			q_dot_fbk = Eigen::Map<const Eigen::VectorXd>(msg->velocity.data(), msg->velocity.size());	
		
			pinocchio::computeAllTerms(model, data, q_fbk, q_dot_fbk);

			Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, dim_joints);
			Eigen::MatrixXd jacobian_dot = Eigen::MatrixXd::Zero(6, dim_joints);

			pinocchio::getJointJacobian(model, data, end_effector_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
      J = jacobian.topRows<3>();
			J_transpose = J.transpose();
			J_pseudo = J.completeOrthogonalDecomposition().pseudoInverse();
			J_transpose_pseudo = J_transpose.completeOrthogonalDecomposition().pseudoInverse();

			pinocchio::getJointJacobianTimeVariation(model, data, end_effector_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot);
			J_dot = jacobian_dot.topRows<3>();
	
			M  = data.M;
			M_inverse = M.inverse();
			h  = data.nle;

			received_feedback = true;
			//ROS_INFO("Joint: %d", dim_joints);
			//ROS_INFO("Jacobian size: %ld x %ld", jacobian.rows(), jacobian.cols());
    	//ROS_INFO("Jacobian dot size: %ld x %ld", jacobian_dot.rows(), jacobian_dot.cols());
    	//ROS_INFO("J size: %ld x %ld", J.rows(), J.cols());
    	//ROS_INFO("J transpose size: %ld x %ld", J_transpose.rows(), J_transpose.cols());
    	//ROS_INFO("J pseudo-inverse size: %ld x %ld", J_pseudo.rows(), J_pseudo.cols());
    	//ROS_INFO("J transpose pseudo-inverse size: %ld x %ld", J_transpose_pseudo.rows(), J_transpose_pseudo.cols());
    	//ROS_INFO("Jacobian dot size: %ld x %ld", J_dot.rows(), J_dot.cols());
    	//ROS_INFO("Mass matrix size: %ld x %ld", M.rows(), M.cols());
    	//ROS_INFO("Inverse mass matrix size: %ld x %ld", M_inverse.rows(), M_inverse.cols());
    	//ROS_INFO("NLE size: %ld", h.size());

			// Compute the end-effector pose and twist
			geometry_msgs::Pose pose_msg;
	    geometry_msgs::Twist twist_msg;
      pinocchio::SE3 end_effector_pose = data.oMi[end_effector_id];

			x_fbk(0) = end_effector_pose.translation().x(); 
			x_fbk(1) = end_effector_pose.translation().y();
			x_fbk(2) = end_effector_pose.translation().z();

      // Set pose
      pose_msg.position.x = end_effector_pose.translation().x();
      pose_msg.position.y = end_effector_pose.translation().y();
      pose_msg.position.z = end_effector_pose.translation().z();

      //ROS_INFO("Pose Position - x: %f, y: %f, z: %f",
      //  pose_msg.position.x,
      //  pose_msg.position.y,
      //  pose_msg.position.z);

      pose_publisher.publish(pose_msg);

      // Set twist
      Eigen::VectorXd end_effector_twist = J * q_dot_fbk;

			x_dot_fbk(0) = end_effector_twist[0]; 
			x_dot_fbk(1) = end_effector_twist[1];
			x_dot_fbk(2) = end_effector_twist[2];

      twist_msg.linear.x = end_effector_twist[0];
      twist_msg.linear.y = end_effector_twist[1];
      twist_msg.linear.z = end_effector_twist[2];

      //ROS_INFO("Twist Position - x: %f, y: %f, z: %f",
      //  twist_msg.linear.x,
      //  twist_msg.linear.y,
      //  twist_msg.linear.z);

      twist_publisher.publish(twist_msg);
		}

		// Reference pose callback
		void referencePoseCallback(const geometry_msgs::Pose msg)
		{
			x_ref(0) = msg.position.x;
    	x_ref(1) = msg.position.y;
    	x_ref(2) = msg.position.z;	

			received_reference_pose = true;
		}

		// Reference twist callback
		void referenceTwistCallback(const geometry_msgs::Twist msg)
		{
			x_dot_ref(0) = msg.linear.x;
			x_dot_ref(1) = msg.linear.y;
			x_dot_ref(2) = msg.linear.z;
	
			received_reference_twist = true;
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

		void publishTorque()
		{
			if (with_redundancy)
			{
				publishTorqueWithRedundancy();
			}
			else
			{
				publishTorqueWithoutRedundancy();
			}
		}

		void publishTorqueWithoutRedundancy()
		{
			Eigen::MatrixXd wedge = computeWedge();	
      Eigen::MatrixXd bias = computeBias(wedge);
      Eigen::VectorXd x_dot2_cmd = computeXDot2Command();

			Eigen::VectorXd tau_cmd = J_transpose * ((wedge * x_dot2_cmd) + bias);

			std_msgs::Float64MultiArray torque_msg;
      torque_msg.data.resize(tau_cmd.size());
      for (int i = 0; i < tau_cmd.size(); ++i) {
        torque_msg.data[i] = tau_cmd[i];
      }

      torque_publisher.publish(torque_msg);
		}		

		// Publish 
		void publishTorqueWithRedundancy() 
		{	
			Eigen::MatrixXd wedge = computeWedge();	
			Eigen::MatrixXd bias = computeBias(wedge);
			Eigen::VectorXd x_dot2_cmd = computeXDot2Command();
	
			//std::stringstream ss;
			//ss << "wedge:\n" << wedge << "\n";
			//ss << "bias:\n" << bias << "\n";
			//ss << "x_dot2_cmd:\n" << x_dot2_cmd << "\n";

			Eigen::VectorXd tau_cmd = J_transpose * ((wedge * x_dot2_cmd) + bias);
			//ss << "tau_cmd:\n" << tau_cmd << "\n";
	
			Eigen::MatrixXd P = computeP();
			//ss << "P:\n" << P << "\n";

			Eigen::VectorXd tau_joint = computeTauJoint();
			//ss << "tau_joint:\n" << tau_joint << "\n";

			Eigen::VectorXd tau_null = P * tau_joint;
			//ss << "tau_null:\n" << tau_null << "\n";

			Eigen::VectorXd tau_total = tau_cmd + tau_null;
			//ss << "tau_total:\n" << tau_total << "\n";

			//ROS_INFO("%s", ss.str().c_str());
	
			std_msgs::Float64MultiArray torque_msg;
			torque_msg.data.resize(tau_total.size());
			for (int i = 0; i < tau_total.size(); ++i) {
				torque_msg.data[i] = tau_total[i];
			}

			torque_publisher.publish(torque_msg);
		}

		Eigen::MatrixXd computeWedge()
		{	
			Eigen::MatrixXd wedge = J_transpose_pseudo * M * J_pseudo;
			return wedge;
		}

		Eigen::MatrixXd computeBias(const Eigen::MatrixXd& wedge)
		{		
			Eigen::MatrixXd bias = (J_transpose_pseudo * h) - (wedge * J_dot * q_dot_fbk);
			return bias;
		}

		Eigen::VectorXd computeXDot2Command()
		{
			Eigen::VectorXd x_dot2_cmd = (d_effector * (x_dot_ref - x_dot_fbk)) + (k_effector * (x_ref - x_fbk));
			return x_dot2_cmd;
		}

		Eigen::MatrixXd computeP()
		{
			Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_joints, dim_joints);
			Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_joints, dim_joints);
			//ROS_INFO("J size: %ld x %ld", J.rows(), J.cols());
			//ROS_INFO("M_inverse size: %ld x %ld", M_inverse.rows(), M_inverse.cols());
			//ROS_INFO("J_transpose size: %ld x %ld", J_transpose.rows(), J_transpose.cols());
			//ROS_INFO("I size: %ld x %ld", I.rows(), I.cols());

			Eigen::MatrixXd J_M_inv = J * M_inverse;
			//ROS_INFO("J * M_inverse size: %ld x %ld", J_M_inv.rows(), J_M_inv.cols());

			Eigen::MatrixXd J_M_inv_JT = J_M_inv * J_transpose;
			//ROS_INFO("J * M_inverse * J_transpose size: %ld x %ld", J_M_inv_JT.rows(), J_M_inv_JT.cols());

			if (J_M_inv_JT.rows() != J_M_inv_JT.cols()) {
    		//ROS_ERROR("Matrix J * M_inverse * J_transpose is not square and cannot be inverted.");
			} else {
    		Eigen::MatrixXd J_M_inv_JT_inv = J_M_inv_JT.inverse();
    		//ROS_INFO("Inverse of (J * M_inverse * J_transpose) size: %ld x %ld", J_M_inv_JT_inv.rows(), J_M_inv_JT_inv.cols());

    		Eigen::MatrixXd JT_J_M_inv_JT_inv = J_transpose * J_M_inv_JT_inv;
    		//ROS_INFO("J_transpose * (J * M_inverse * J_transpose).inverse() size: %ld x %ld", JT_J_M_inv_JT_inv.rows(), JT_J_M_inv_JT_inv.cols());

    		Eigen::MatrixXd JT_J_M_inv_JT_inv_J_M_inv = JT_J_M_inv_JT_inv * J_M_inv;
    		//ROS_INFO("J_transpose * (J * M_inverse * J_transpose).inverse() * J * M_inverse size: %ld x %ld", JT_J_M_inv_JT_inv_J_M_inv.rows(), JT_J_M_inv_JT_inv_J_M_inv.cols());

				//ROS_INFO("I matrix size: %ld x %ld", I.rows(), I.cols());
    		if (I.rows() == JT_J_M_inv_JT_inv_J_M_inv.rows() && I.cols() == JT_J_M_inv_JT_inv_J_M_inv.cols()) {
        	P = I - JT_J_M_inv_JT_inv_J_M_inv;
        	//ROS_INFO("Final P size: %ld x %ld", P.rows(), P.cols());
    		} else {
        	ROS_ERROR("Dimension mismatch in final subtraction: I and the computed matrix have different sizes.");
    		}
			}

			return P;
		}

		Eigen::VectorXd computeTauJoint()
		{
			Eigen::VectorXd q_dot2_cmd = (d_joint * (q_dot_ref - q_dot_fbk)) + (k_joint * (q_ref - q_fbk));

      Eigen::VectorXd tau_joint = (M * q_dot2_cmd) + h;

			return tau_joint;
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
			if (!node_handle.getParam("/stiffness_joint", k_joint))
      {
        ROS_WARN("Stiffness joint is not set.");
				return 1;
      }
			if (!node_handle.getParam("/stiffness_effector", k_effector))
      {
        ROS_WARN("Stiffness end effector is not set.");
        return 1;
      }
			return 0;
		}

		int getDamping()
		{
			if (!node_handle.getParam("/damping_joint", d_joint))
			{
				ROS_WARN("Damping joint is not set.");
				return 1;
			}
			if (!node_handle.getParam("/damping_effector", d_effector))
      {
        ROS_WARN("Damping effector is not set.");
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
      if (!node_handle.getParam("/reference_twist_topic", reference_twist_topic))
      {
        ROS_WARN("Reference twist topic not set. Using default.");
        reference_twist_topic = "/gen3/reference/twist";
        return 1;
      }
      if (!node_handle.getParam("/reference_position_topic", reference_position_topic))
      {
        ROS_WARN("Reference position topic not set. Using default.");
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
			if (!node_handle.getParam("/feedback_pose_topic", feedback_pose_topic))
      {
        ROS_WARN("Feedback pose topic not set. Using default.");
        feedback_pose_topic = "/gen3/feedback/pose";
        return 1;
      }
      if (!node_handle.getParam("/feedback_twist_topic", feedback_twist_topic))
      {
        ROS_WARN("Feeback twist topic not set. Using default.");
        feedback_twist_topic = "/gen3/feeback/twist";
        return 1;
      }

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
  ros::init(argc, argv, "pose_controller");

  ControlNode controller;
  controller.run();

  return 0;
}
