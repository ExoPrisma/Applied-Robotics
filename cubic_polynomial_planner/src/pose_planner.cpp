#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <highlevel_msgs/MoveTo.h>

class PlannerNode
{
	public:
		
		PlannerNode()
		{
			getPublishRate();
			getSubscriber();
			getPublisher();

			pose_subscriber = node_handle.subscribe(feedback_pose_topic, 3, &PlannerNode::poseCallBack, this);

      move_to_server = node_handle.advertiseService("/pose_planner/move_to", &PlannerNode::moveToCallBack, this);

      pose_publisher = node_handle.advertise<geometry_msgs::Pose>(reference_pose_topic, 3);
      twist_publisher = node_handle.advertise<geometry_msgs::Twist>(reference_twist_topic, 3);
		}

		void run()
    {

      ros::Rate loop_rate(publish_rate);

      while(ros::ok())
      {
        while(is_moving)
        {
          publishPoseTwist();
        }

        ros::spinOnce();
        loop_rate.sleep();
      }
    }
		
		bool moveToCallBack(highlevel_msgs::MoveTo::Request &req,
                        highlevel_msgs::MoveTo::Response &res)
    {

      if(req.z <= 0)
      {
        ROS_WARN("Invalid target position: z must be > 0");
        res.success = false;
        return false;
      }

      final_pose.position.x = req.x;
      final_pose.position.y = req.y;
      final_pose.position.z = req.z;
      final_pose.orientation = current_pose.orientation;
      target_time = req.T;

      start_time = ros::Time::now().toSec();

      is_moving = true;

      ROS_INFO("Received call back");
      res.success = true;
      return true;
    }


	private:
		
		// Node handle
		ros::NodeHandle node_handle;

		// Private subscribers/servers/publishers
    ros::Subscriber pose_subscriber;
    ros::ServiceServer move_to_server;
    ros::Publisher pose_publisher;
    ros::Publisher twist_publisher;

		// Private parameters
		double publish_rate;

    std::string feedback_pose_topic, reference_pose_topic, reference_twist_topic;

		// Private variables
		double start_time;
    double target_time;

    bool is_moving = false;

		// Reference pose
		geometry_msgs::Pose current_pose;
    geometry_msgs::Pose final_pose;
		
		// Pose callback function
		void poseCallBack(const geometry_msgs::Pose::ConstPtr& msg)
    {
      current_pose = *msg;
    }

		// 
		void publishPoseTwist()
    {
      double current_time = ros::Time::now().toSec();
      double t = current_time - start_time;

      if(t >= target_time)
      {
        is_moving = false;
        ROS_INFO("Has reached the target position");
        return;
      }

      Eigen::VectorXd start_pose(3);
      Eigen::VectorXd end_pose(3);

      start_pose << current_pose.position.x,
                   current_pose.position.y,
                   current_pose.position.z;
      end_pose << final_pose.position.x,
                 final_pose.position.y,
                 final_pose.position.z;

      double pose_polynomial = ((3 * t * t) / (target_time * target_time)) - ((2 * t * t * t) / (target_time * target_time * target_time));
      Eigen::VectorXd target_pose = start_pose + (pose_polynomial * (end_pose - start_pose));

      double twist_polynomial = ((6 * t) / (target_time * target_time)) - ((6 * t * t) / (target_time * target_time * target_time));
      Eigen::VectorXd target_twist = twist_polynomial * (end_pose - start_pose);


      geometry_msgs::Pose published_pose = current_pose;
      published_pose.position.x = target_pose[0];
      published_pose.position.y = target_pose[1];
      published_pose.position.z = target_pose[2];
      published_pose.orientation = final_pose.orientation;

      pose_publisher.publish(published_pose);
      ROS_INFO("Published pose: [%f, %f, %f] with orientation [%f, %f, %f, %f] at time %f",
        target_pose[0],
        target_pose[1],
        target_pose[2],
        final_pose.orientation.x,
        final_pose.orientation.y,
        final_pose.orientation.z,
        final_pose.orientation.w,
        t);

      geometry_msgs::Twist published_twist;
      published_twist.linear.x = target_twist[0];
      published_twist.linear.y = target_twist[1];
      published_twist.linear.z = target_twist[2];

      twist_publisher.publish(published_twist);
      ROS_INFO("Published twist: [%f, %f, %f] at time %f", 
        target_twist[0],
        target_twist[1],  
        target_twist[2],
        t);
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
		
		int getSubscriber()
		{
			 if (!node_handle.getParam("feedback_pose_topic", feedback_pose_topic))
      {
        ROS_WARN("Feedback pose topic not set. Using default.");
        feedback_pose_topic = "/gen3/feedback/pose";
				return 1;
      }
			return 0;
		}
		
		int getPublisher()
		{
			if (!node_handle.getParam("reference_pose_topic", reference_pose_topic))
      {
        ROS_WARN("Reference pose topic not set. Using default.");
        reference_pose_topic = "/gen3/reference/pose";
				return 1;
      }
      if (!node_handle.getParam("reference_twist_topic", reference_twist_topic))
      {
        ROS_WARN("Reference twist topic not set. Using default.");
        reference_twist_topic = "/gen3/reference/twist";
				return 1;
      }
			return 0;
		}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_planner");

  PlannerNode planner;
  planner.run();

  return 0;
}

