#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

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

			joint_subscriber = node_handle.subscribe(joint_states_topic, 10, &JointPlannerNode::jointStatesCallback, this);
      velocity_publisher = node_handle.advertise<std_msgs::Float64MultiArray>(reference_velocity_topic, 10);
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

		// Private subscriber/publisher
    ros::Subscriber joint_subscriber;
    ros::Publisher velocity_publisher;

		// Private parameters
    double publish_rate;
    double k_att;
    double max_velocity;

    std::vector<double> default_positions;

    std::string joint_states_topic, reference_velocity_topic;

		// Joint state callback
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {

      Eigen::VectorXd joint_position = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size());
      Eigen::VectorXd default_position = Eigen::Map<const Eigen::VectorXd>(default_positions.data(), default_positions.size());

      if (msg->position.size() != default_positions.size())
      {
        ROS_WARN("Position size does not match default positions size!");
        return;
      }
			
			computePotentialField(joint_position, default_position);
    }

		void computePotentialField(const Eigen::VectorXd& joint_position, const Eigen::VectorXd& default_position)
		{
      std_msgs::Float64MultiArray velocity_msg;
			
      Eigen::VectorXd v_ref = k_att * (default_position - joint_position);

      for (int i = 0; i < v_ref.size(); ++i) {
        if (std::abs(v_ref(i)) > max_velocity) {
            ROS_INFO("Joint %d velocity exceeded limit: %f", i, v_ref(i));
            v_ref(i) = std::copysign(max_velocity, v_ref(i));
        }
      }

      Eigen::VectorXd joint_velocity = joint_position + (v_ref * (1.0 / publish_rate));

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
			if (!node_handle.getParam("/gen3/joint/default", default_positions))
			{
				ROS_WARN("Default joints not set.");
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

      return 0;
    }

    int getPublisher()
    {
      if (!node_handle.getParam("/reference_velocity_topic", reference_velocity_topic))
      {
        ROS_WARN("Reference velocity topic not set. Using default.");
        joint_states_topic = "reference_velocity_topic";
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

