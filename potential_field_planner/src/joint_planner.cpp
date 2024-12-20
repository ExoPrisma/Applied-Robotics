#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <sstream>

class JointPlannerNode
{
	public:
		
		JointPlannerNode()
		{
			getPublishRate();
			getKAtt();
			getMaxVelocity();
			getDefaultJoint();
			getSubscriber();
			getPublisher();
			
			// Subscriber
			joint_subscriber = node_handle.subscribe(joint_states_topic, 10, &JointPlannerNode::jointStatesCallback, this);
			
			// Publisher
			position_publisher = node_handle.advertise<std_msgs::Float64MultiArray>(reference_position_topic, 10);
      velocity_publisher = node_handle.advertise<std_msgs::Float64MultiArray>(reference_velocity_topic, 10);
		}
		
		// Run method
    void run()
    {
      ros::Rate loop_rate(publish_rate);

      while(ros::ok())
      {
				if(received_feedback)
        {
					publishReferencePositionVelocity();
        }

        ros::spinOnce();
        loop_rate.sleep();
      }
    }

	private:

		// Node handle
		ros::NodeHandle node_handle;

		// Private subscriber/publisher
    ros::Subscriber joint_subscriber;
		ros::Publisher position_publisher;
    ros::Publisher velocity_publisher;

		// Private parameters
    double publish_rate;
    double k_att;
    double max_velocity;
		std::vector<double> default_configurations;

    std::string joint_states_topic, reference_position_topic, reference_velocity_topic;

		// Private variable
    bool received_feedback = false;

		// Reference configuration
		Eigen::VectorXd current_configurations;
		Eigen::VectorXd target_configurations;

		// Joint state callback
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
      current_configurations = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size() - 1);

			received_feedback = true;
    }

		void publishReferencePositionVelocity()
		{	
      Eigen::VectorXd q_ref = k_att * (target_configurations - current_configurations);

			Eigen::VectorXd joint_velocity = q_ref;
			if (q_ref.norm() > max_velocity)
      {
        joint_velocity = q_ref * (max_velocity / q_ref.norm());
      }

			Eigen::VectorXd joint_position = current_configurations + (q_ref * (1.0/publish_rate));

      std_msgs::Float64MultiArray position_msg;
			position_msg.data.resize(joint_position.size());
      for (size_t i = 0; i < joint_position.size(); ++i) {
        position_msg.data[i] = joint_position[i];
      }

			position_publisher.publish(position_msg);

      std_msgs::Float64MultiArray velocity_msg;
			velocity_msg.data.resize(joint_velocity.size());
      for (size_t i = 0; i < joint_velocity.size(); ++i) {
        velocity_msg.data[i] = joint_velocity[i];
      }

      velocity_publisher.publish(velocity_msg);
		}
	
		// Get parameter helper functions
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
      if (!node_handle.getParam("/gen3/linear/k_att", k_att))
      {
        ROS_WARN("K_att not set.");
        return 1;
      }
      return 0;
    }

    int getMaxVelocity()
    {
      if (!node_handle.getParam("/gen3/linear/max_velocity", max_velocity))
      {
        ROS_WARN("Max velocity not set.");
        return 1;
      }
      return 0;
    }

		int getDefaultJoint()
		{
			if (!node_handle.getParam("/gen3/joint/default", default_configurations))
			{
				ROS_WARN("Default joints not set.");
				return 1;
			}
      target_configurations = Eigen::Map<const Eigen::VectorXd>(default_configurations.data(), default_configurations.size());

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

      return 0;
    }

    int getPublisher()
    {
			if (!node_handle.getParam("/reference_position_topic", reference_position_topic))
      {
        ROS_WARN("Reference position topic not set. Using default.");
				reference_position_topic  = "/gen3/reference/position";
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

};

// Main method
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_planner");

  JointPlannerNode planner;
  planner.run();

  return 0;
}

