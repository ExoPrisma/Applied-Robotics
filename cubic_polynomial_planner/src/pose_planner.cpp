#include <ros/ros.h>
#include <cmath>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <highlevel_msgs/MoveTo.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PlannerNode
{
	public:
		
		PlannerNode()
		{
			getPublishRate();
			getDefault();
			getSubscriber();
			getPublisher();

			pose_subscriber = node_handle.subscribe(feedback_pose_topic, 3, &PlannerNode::poseCallback, this);

      move_to_server = node_handle.advertiseService("/pose_planner/move_to", &PlannerNode::moveToCallback, this);
			move_ori_server = node_handle.advertiseService("/pose_planner/move_ori", &PlannerNode::moveOriCallback, this);

      pose_publisher = node_handle.advertise<geometry_msgs::Pose>(reference_pose_topic, 3);
      twist_publisher = node_handle.advertise<geometry_msgs::Twist>(reference_twist_topic, 3);
		}

		void run()
    {

      ros::Rate loop_rate(publish_rate);

      while(ros::ok())
      {
				if(!is_moving)
				{
					pose_publisher.publish(final_pose);
				}
        while(is_moving)
        {
					//ROS_INFO("Published default pose: [%f, %f, %f] with orientation [%f, %f, %f, %f]",
        	//	current_pose.position.x,
        	//	current_pose.position.y,
        	//	current_pose.position.z,
        	//	current_pose.orientation.x,
        	//	current_pose.orientation.y,
        	//	current_pose.orientation.z,
       		//	current_pose.orientation.w);
          publishPoseTwist();
        }

        ros::spinOnce();
        loop_rate.sleep();
      }
    }
		
		bool moveToCallback(highlevel_msgs::MoveTo::Request &req,
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
      target_time = req.T;

      start_time = ros::Time::now().toSec();

			is_moving = true;

      ROS_INFO("Received move_to call back");
      res.success = true;
      return true;
    }
		
		bool moveOriCallback(highlevel_msgs::MoveTo::Request &req,
												 highlevel_msgs::MoveTo::Response &res)
		{
			tf2 ::Quaternion q;
			q.setRPY(req.x, req.y, req.z);

			final_pose.orientation =  tf2 ::toMsg(q);	
			target_time = req.T;

      start_time = ros::Time::now().toSec();

			is_moving = true;

			ROS_INFO("Received move_ori call back");
			res.success = true;
			return true;
		}


	private:
		
		// Node handle
		ros::NodeHandle node_handle;

		// Private subscribers/servers/publishers
    ros::Subscriber pose_subscriber;
    ros::ServiceServer move_to_server;
    ros::ServiceServer move_ori_server;
    ros::Publisher pose_publisher;
    ros::Publisher twist_publisher;

		// Private parameters
		double publish_rate;
		std::vector<double> default_translation;
		std::vector<double> default_orientation;

		std::string default_translation_topic, default_orientation_topic;
    std::string feedback_pose_topic;
		std::string reference_pose_topic, reference_twist_topic;

		// Private variables
		double start_time;
    double target_time;

    bool is_moving = false;

		// Reference pose
		geometry_msgs::Pose current_pose;
    geometry_msgs::Pose final_pose;	

		// Pose callback
		void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
      current_pose = *msg;
    }

		// Publish pose & twist
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
			tf2::Quaternion current_q;
			tf2::Quaternion final_q;

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

			tf2::fromMsg(current_pose.orientation, current_q);
			tf2::fromMsg(final_pose.orientation, final_q);
		
			tf2::Quaternion published_q;

			published_q = final_q.slerp(current_q, t);

      geometry_msgs::Pose published_pose = current_pose;
      published_pose.position.x = target_pose[0];
      published_pose.position.y = target_pose[1];
      published_pose.position.z = target_pose[2];
			published_pose.orientation = tf2::toMsg(published_q);

      pose_publisher.publish(published_pose);
      //ROS_INFO("Published pose: [%f, %f, %f] with orientation [%f, %f, %f, %f] at time %f",
      //  current_pose.position.x,
      //  final_pose.position.y,
      //  published_pose.position.z,
      //  published_pose.orientation.x,
      //  published_pose.orientation.y,
      //  published_pose.orientation.z,
      //  published_pose.orientation.w,
      //  t);

      geometry_msgs::Twist published_twist;
      published_twist.linear.x = target_twist[0];
      published_twist.linear.y = target_twist[1];
      published_twist.linear.z = target_twist[2];

      twist_publisher.publish(published_twist);
      //ROS_INFO("Published twist: [%f, %f, %f] at time %f", 
      //  target_twist[0],
      //  target_twist[1],  
      //  target_twist[2],
      //  t);
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

		int getDefault()
		{
			if (!node_handle.getParam("/default_translation_topic", default_translation_topic))
      {
        ROS_WARN("Default translation topic not set. Using default.");
        default_translation_topic = "/gen3/linear/default";
        return 1;
      }
			
			if (!node_handle.getParam("default_orientation_topic", default_orientation_topic))
			{
				ROS_WARN("Default orientation topic not set. Using default.");
				default_orientation_topic = "/gen3/angular/default";
				return 1;
			}

			if (!node_handle.getParam(default_translation_topic, default_translation))
			{
				ROS_WARN("Couldn't retrieve default translation. Using default.");
				default_translation = {0.0, 0.0, 0.0};
				return 1;
			}
			final_pose.position.x = default_translation[0];
			final_pose.position.y = default_translation[1];
			final_pose.position.z = default_translation[2];
			ROS_INFO("x: %f, y: %f, z: %f", final_pose.position.x, final_pose.position.y, final_pose.position.z);

			if (!node_handle.getParam(default_orientation_topic, default_orientation))
			{
				ROS_WARN("Couldn't retrieve default orientation. using default.");
				default_orientation = {0.0, 0.0, 0.0};
				return 1;
			}
			tf2::Quaternion q;
			q.setRPY(default_orientation[0], default_orientation[1], default_orientation[2]);

			final_pose.orientation = tf2 ::toMsg(q);
			ROS_INFO("x: %f, y: %f, z: %f, w: %f", final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w);

      return 0;
		}
		
		int getSubscriber()
		{
			if (!node_handle.getParam("/feedback_pose_topic", feedback_pose_topic))
      {
        ROS_WARN("Feedback pose topic not set. Using default.");
        feedback_pose_topic = "/gen3/feedback/pose";
				return 1;
      }
			return 0;
		}
		
		int getPublisher()
		{
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

