#include "ros/ros.h"
#include "trajopt_examples/trajopt_node.h"
#include <control_msgs/GripperCommandActionGoal.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "nist/cable_model.h"
#include <geometry_msgs/Transform.h>
#include <kortex_driver/SendTwistCommand.h>
#include <iostream>
#include <cmath>        // std::abs
#include <stdlib.h>     /* abs */
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nist/cable_plane_color.h"
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/BaseFeedback.h>



class nist_task
{
	public: 
		nist_task()
		{
			client = n.serviceClient<trajopt_examples::trajopt_node>("/my_gen3/trajopt_motion_planning");
			gripper_publisher = n.advertise<control_msgs::GripperCommandActionGoal>("/my_gen3/robotiq_2f_85_gripper_controller/gripper_cmd/goal", 1, true);
			client_cable = n.serviceClient<nist::cable_model>("/nist/cable_modeling");
			gen3_velocity_client =n.serviceClient<kortex_driver::SendTwistCommand>("/my_gen3/base/send_twist_command");
			cable_2d_client = n.serviceClient<nist::cable_plane_color>("/nist/cable_plane_color");
			external_force_sub = n.subscribe("/my_gen3/base_feedback", 1, &nist_task::force_callback, this);
		}


		void Gripper(double position)
		{
			// 0 -> open, 0.77 - > close
			control_msgs::GripperCommandActionGoal msg;
		    msg.goal.command.position = position;
		    gripper_publisher.publish(msg);
		}

		void Motion(geometry_msgs::Pose pose, double dt)
		{
			
			srv.request.pose = pose;
			srv.request.dt = dt;
			if (client.call(srv))
			{
				ROS_INFO("Successed to call trajopt service");
			}
			else
			{
				ROS_ERROR("Failed to call trajopt service");
				exit(1);
			}
		}


		void GoToInitPosition()
		{

			geometry_msgs::Pose pose;
			pose.position.x = 0.478;
			pose.position.y = 0.140;
			pose.position.z = 0.428;
			pose.orientation.x = 1.000;
			pose.orientation.y = 0.013;
			pose.orientation.z = 0.014;
			pose.orientation.w = 0.002;

			// move the arm
			Motion(pose,1.0);
		}


	private:
		ros::NodeHandle n;
		ros::ServiceClient client;
		ros::ServiceClient client_cable;
		ros::Publisher gripper_publisher;
		trajopt_examples::trajopt_node srv;
		nist::cable_model srv_cable;
		geometry_msgs::Transform transform_cable;
		ros::ServiceClient gen3_velocity_client;
		ros::ServiceClient cable_2d_client;
		nist::cable_plane_color srv_cable_plane;
		ros::Subscriber external_force_sub;

    	
};


int main(int argc, char **argv)
{

	ros::init (argc, argv, "nist_trajopt");
	nist_task nist_cool;

	
	nist_cool.Gripper(0.0);
	nist_cool.GoToInitPosition();
	sleep(2.0);
	nist_cool.Gripper(0.77);

	return 0;
}