#include <ros/ros.h>
#include <cstdlib>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <highlevel_msgs/MoveTo.h>
#include <Eigen/Dense>

class PlannerNode
{
  public:

    PlannerNode()
    {
      pose_subscriber = node_handle.subscribe("/firefly/ground_truth/pose", 3, &PlannerNode::poseCallBack, this);

			move_to_server = node_handle.advertiseService("/pose_planner/move_to", &PlannerNode::moveToCallBack, this);

			pose_publisher = node_handle.advertise<geometry_msgs::Pose>("/firefly/command/pose", 3);
			twist_publisher = node_handle.advertise<geometry_msgs::Twist>("/firefly/command/twist", 3);
    }

    void run() 
    {
      ros::Rate loop_rate(500);

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

    ros::NodeHandle node_handle;
    ros::Subscriber pose_subscriber;
		ros::ServiceServer move_to_server;
		ros::Publisher pose_publisher;
		ros::Publisher twist_publisher;

    geometry_msgs::Pose current_pose;
		geometry_msgs::Pose final_pose;

		double start_time;
		double target_time;

		bool is_moving = false;

    void poseCallBack(const geometry_msgs::Pose::ConstPtr& msg)
    {
      current_pose = *msg;
    }

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
//			ROS_INFO("Published twist: [%f, %f, %f] at time %f", 
//				target_twist[0],
//				target_twist[1],	
//			  target_twist[2],
//				t);
		}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_planner");

  PlannerNode planner;
  planner.run();

  return 0;
}
