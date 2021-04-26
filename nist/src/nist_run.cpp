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

		void force_callback(const kortex_driver::BaseCyclic_Feedback msg)
		{
			external_force_y = msg.base.tool_external_wrench_force_y;
			//std::cout << external_force_y << std::endl;
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

		double get_abs(double a)
		{
			if (a<0)
			{
				a = -a;
			}
			else
			{
				a = a;
			}
			return a;
		}

	    void gen3_velocity_control(float x, float y, float z, float roll, float pitch, float yaw)
	    {
	      roll = (float)(roll/3.14159)*180;
	      pitch = (float)(pitch/3.14159)*180;
	      yaw = (float)(yaw/3.14159)*180;
	      kortex_driver::SendTwistCommand srv;
	      srv.request.input.twist.linear_x = x;
	      srv.request.input.twist.linear_y = y;
	      srv.request.input.twist.linear_z = z;
	      srv.request.input.twist.angular_x = roll;
	      srv.request.input.twist.angular_y = pitch;
	      srv.request.input.twist.angular_z = yaw;
	      gen3_velocity_client.call(srv);
	    }

		void updateControlInput(std::string frame_name)
		{
		  try{
		      listener_.waitForTransform(frame_name, "cable_tip_insert",ros::Time(0), ros::Duration(3.0));
		      listener_.lookupTransform(frame_name, "cable_tip_insert",ros::Time(0), transform_tip);
		  }
		  catch (tf::TransformException ex) {
		      ROS_ERROR("%s",ex.what());
		  }

		  tf::Quaternion q_tip(transform_tip.getRotation().x(),transform_tip.getRotation().y(),transform_tip.getRotation().z(),transform_tip.getRotation().w());
		  tf::Matrix3x3 m_tip(q_tip);
		  m_tip.getEulerYPR(delta_yaw, delta_pitch, delta_roll);
		  std::cout << "Pose error before convertion" << delta_x << " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;


		  delta_x = transform_tip.getOrigin().x();
		  delta_y = transform_tip.getOrigin().y();   // to the pre-insert state
		  delta_z = transform_tip.getOrigin().z();
		}


		/* this function transfer the angle difference to the end_effector_link frame */
	    void getEulerYPR_ee(std::string frame_name)
	    {
	      try{
	          listener_.waitForTransform("end_effector", frame_name,ros::Time(0), ros::Duration(3.0));
	          listener_.lookupTransform("end_effector", frame_name,ros::Time(0), transform_pre_ee);
	      }
	      catch (tf::TransformException ex) {
	          ROS_ERROR("%s",ex.what());
	      }
	      try{
	          listener_.waitForTransform("end_effector", "cable_tip_insert",ros::Time(0), ros::Duration(3.0));
	          listener_.lookupTransform("end_effector", "cable_tip_insert",ros::Time(0), transform_tip_ee); 
	      }
	      catch (tf::TransformException ex) {
	          ROS_ERROR("%s",ex.what());
	      }
	          
	      tf::Quaternion q_pre(transform_pre_ee.getRotation().x(),transform_pre_ee.getRotation().y(),transform_pre_ee.getRotation().z(),transform_pre_ee.getRotation().w());
	      tf::Matrix3x3 m_pre(q_pre);
	      m_pre.getEulerYPR(yaw_pre_ee, pitch_pre_ee, roll_pre_ee);    
	      tf::Quaternion q_tip(transform_tip_ee.getRotation().x(),transform_tip_ee.getRotation().y(),transform_tip_ee.getRotation().z(),transform_tip_ee.getRotation().w());
	      tf::Matrix3x3 m_tip(q_tip);
	      m_tip.getEulerYPR(yaw_tip_ee, pitch_tip_ee, roll_tip_ee); 

	      // std::cout << "pre to ee" << roll_pre_ee << ", " << pitch_pre_ee << ", " << yaw_pre_ee << std::endl;
	      // std::cout << "tip to ee" << roll_tip_ee << ", " << pitch_tip_ee << ", " << yaw_tip_ee << std::endl;

	      roll_ee = (roll_pre_ee - roll_tip_ee);
	      pitch_ee = (pitch_pre_ee - pitch_tip_ee);
	      yaw_ee = (yaw_pre_ee - yaw_tip_ee);
	    }

	    /* this function transfer the position difference to the base_link frame */
	    void getXYZ_gen3_base(std::string frame_name)
	    {

	      try{
	          listener_.waitForTransform("base_link", frame_name,ros::Time(0), ros::Duration(3.0));
	          listener_.lookupTransform("base_link", frame_name,ros::Time(0), transform_pre_world);
	      }
	      catch (tf::TransformException ex) {
	          ROS_ERROR("%s",ex.what());
	      }
	      try{
	          listener_.waitForTransform("base_link", "cable_tip_insert",ros::Time(0), ros::Duration(3.0));
	          listener_.lookupTransform("base_link", "cable_tip_insert",ros::Time(0), transform_tip_world);
	      }
	      catch (tf::TransformException ex) {
	          ROS_ERROR("%s",ex.what());
	      }
	      x_world = (transform_pre_world.getOrigin().x() - transform_tip_world.getOrigin().x());
	      y_world = (transform_pre_world.getOrigin().y() - transform_tip_world.getOrigin().y());
	      z_world = (transform_pre_world.getOrigin().z() - transform_tip_world.getOrigin().z());
	    }

        float pid_calculation(float setpoint, float pv, std::string ss)
	    {
	      if (ss == "x"){
	        _Kp = 0.2;      // 2
	        _Kd = 0.02;    // 0.2
	        _Ki = 0; 
	      }
	      else if (ss=="y"){
	        _Kp = 0.2;      // 2
	        _Kd = 0.02;    // 0.2
	        _Ki = 0; 
	      }
	      else if (ss=="z"){
	        _Kp = 0.2;      // 2
	        _Kd = 0.02;    // 0.2
	        _Ki = 0; 
	      }
	      else if (ss=="roll"){       // kp = 1, kd = 0.01, ki = 0 (roll, pitch, yaw)works for pose alignment control in short distance
	        _Kp = 0.2;      // 2
	        _Kd = 0.02;    // 0.2
	        _Ki = 0;
	      }
	      else if (ss=="pitch"){
	        _Kp = 0.2;      // 2    // for experiment 2 and 3  use 0.01  (roll, pitch, yaw)
	        _Kd = 0.02;    // 0.2  // for experiment 2 and 3 use 0.01  (roll, pitch, yaw)
	        _Ki = 0;             // for experiment 2 and 3 use 0.000001 (roll, pitch, yaw)
	      }
	      else if (ss=="yaw"){
	        _Kp = 0.2;      // 2
	        _Kd = 0.02;    // 0.2
	        _Ki = 0;
	      }


	      // Calculate error
	      float error = pv - setpoint;

	      // Proportional term
	      float Pout = _Kp * error;

	      float Iout;

	      // Integral term
	      if (ss == "x"){
	        _integral_x += error * _dt;
	        Iout= _Ki * _integral_x;
	      }
	      else if (ss=="y"){
	        _integral_y += error * _dt;
	        Iout = _Ki * _integral_y;
	      }
	      else if (ss=="z"){
	        _integral_z += error * _dt;
	        Iout = _Ki * _integral_z;
	      }
	      else if (ss=="roll"){
	        _integral_roll += error * _dt;
	        Iout = _Ki * _integral_roll;
	      }
	      else if (ss=="pitch"){
	        _integral_pitch += error * _dt;
	        Iout = _Ki * _integral_pitch;
	      }
	      else if (ss=="yaw"){
	        _integral_yaw += error * _dt;
	        Iout = _Ki * _integral_yaw;
	      }


	      // Derivative term
	      float derivative;
	      if (ss=="x"){
	        derivative = (error - _pre_error_x) / _dt;
	      }
	      else if (ss=="y"){
	        derivative = (error - _pre_error_y) / _dt;
	      }
	      else if (ss=="z"){
	        derivative = (error - _pre_error_z) / _dt;
	      }
	      else if (ss=="roll"){
	        derivative = (error - _pre_error_roll) / _dt;
	      }
	      else if (ss=="pitch"){
	        derivative = (error - _pre_error_pitch) / _dt;
	      }
	      else if (ss=="yaw"){
	        derivative = (error - _pre_error_yaw) / _dt;
	      }


	      float Dout = _Kd * derivative;

	      // Calculate total output
	      float output = Pout + Iout + Dout;

	      // Save error to previous error
	      if (ss=="x"){
	        _pre_error_x = error;
	      }
	      else if (ss=="y"){
	        _pre_error_y = error;
	      }
	      else if (ss=="z"){
	        _pre_error_z = error;
	      }
	      else if (ss=="roll"){
	        _pre_error_roll = error;
	      }
	      else if (ss=="pitch"){
	        _pre_error_pitch = error;
	      }
	      else if (ss=="yaw"){
	        _pre_error_yaw = error;
	      }
	      

	      return output;
	    }

		void Insert_visual_servoing(std::string frame_name)
	    {
	      updateControlInput(frame_name);
	      
	      getEulerYPR_ee(frame_name);
	      getXYZ_gen3_base(frame_name);

	      while ((get_abs(delta_roll) > 0.2) || (get_abs(delta_pitch) > 0.1) || (get_abs(delta_yaw) > 0.1) || (get_abs(delta_x) > 0.015) || (get_abs(delta_y) > 0.015) || (get_abs(delta_z) > 0.015))
	      {
	        
	        std::cout << "I am in the loop" << std::endl;

	        float av_yaw = -sin(roll_ee)*pitch_ee + cos(pitch_ee)*cos(roll_ee)*yaw_ee;
	        float av_pitch = cos(roll_ee)*pitch_ee + cos(pitch_ee)*sin(roll_ee)*yaw_ee;
	        float av_roll = roll_ee - sin(pitch_ee)*yaw_ee;

	        // gen3 - jaco has different setup
	        float gen3_roll = av_roll;
	        float gen3_pitch = av_pitch;
	        float gen3_yaw  = av_yaw;

	        float control_step_x = (float) pid_calculation(0, (float)x_world,"x");
	        float control_step_y = (float) pid_calculation(0, (float)y_world,"y");
	        float control_step_z = (float) pid_calculation(0, (float)z_world,"z");
	        float control_step_roll = (float) pid_calculation(0, gen3_roll,"roll");
	        float control_step_pitch = (float) pid_calculation(0, gen3_pitch,"pitch");
	        float control_step_yaw = (float) pid_calculation(0, gen3_yaw,"yaw");
	        gen3_velocity_control(control_step_x,control_step_y,control_step_z,control_step_roll,control_step_pitch,control_step_yaw);

	        // velocity_pub((float)delta_y, (float)delta_x, (float)delta_z, jaco_roll, jaco_pitch,jaco_yaw);
	        getEulerYPR_ee(frame_name);
	        getXYZ_gen3_base(frame_name);
	        updateControlInput(frame_name);
	        std::cout << "Pose error data:" << delta_x << " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;

	      }
	      gen3_velocity_control(0.00, 0.00, 0.00, 0.00, 0.00, 0.00);
	    }

	    void Insert()
	    {
	    	tf::StampedTransform transform_insert;
			try{
		          listener_.waitForTransform("base_link", "end_effector",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "end_effector",ros::Time(0), transform_insert);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}
			geometry_msgs::Pose insert_pose;
			insert_pose.position.x = transform_insert.getOrigin().x();
			insert_pose.position.y = transform_insert.getOrigin().y()-0.005;
			insert_pose.position.z = transform_insert.getOrigin().z()-0.13;
			insert_pose.orientation.x = transform_insert.getRotation().x();
			insert_pose.orientation.y = transform_insert.getRotation().y();
			insert_pose.orientation.z = transform_insert.getRotation().z();
			insert_pose.orientation.w = transform_insert.getRotation().w();	
			Motion(insert_pose,0.5);    	
	    }

		void cable_model(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, int selected)
		{
			
			srv_cable.request.x_min = x_min;
			srv_cable.request.x_max = x_max;
			srv_cable.request.y_min = y_min;
			srv_cable.request.y_max = y_max;
			srv_cable.request.z_min = z_min;
			srv_cable.request.z_max = z_max;
			srv_cable.request.selected = selected;


			if (client_cable.call(srv_cable))
			{
				ROS_INFO("Successed to call cable model service");
				transform_cable = srv_cable.response.transform_cable;
			}
			else
			{
				ROS_ERROR("Failed to call cable model service");
				exit(1);
			}
		}

		void GoToInitPosition()
		{
			/* retract
		      orientation(0.029, 0.706, 0.707, 0.029) // w x y z
		      Eigen::Vector3d(0.125, 0.002, 0.331)
		    */
		    /* home 
		      orientation(0.499, 0.500, 0.500, 0.501); // w x y z
		      Eigen::Vector3d(0.456, 0.02, 0.434);
		    */
			// start postion of part 3
			// rostopic echo /my_gen3/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
			// position: 
			// x: 0.343422460792
			// y: 0.00134346334286
			// z: 0.343825302969
			// orientation: 
			// x: 0.999054092944
			// y: 0.0260039666881
			// z: 0.0348388842375
			// w: 0.000982462666346

			geometry_msgs::Pose pose;
			pose.position.x = 0.478;
			pose.position.y = 0.140;
			pose.position.z = 0.428;
			pose.orientation.x = 1.000;
			pose.orientation.y = 0.013;
			pose.orientation.z = 0.014;
			pose.orientation.w = 0.002;

			// move the arm
			Motion(pose,0.8);
		}

		void GoToInit_tube1_tube2()
		{	// old
			// geometry_msgs::Pose pose;
			// pose.position.x = 0.382;
			// pose.position.y = -0.047;
			// pose.position.z = 0.400;
			// pose.orientation.x = 0.698;
			// pose.orientation.y = 0.668;
			// pose.orientation.z = 0.187;
			// pose.orientation.w = 0.180;

			geometry_msgs::Pose pose;
			pose.position.x = 0.387;
			pose.position.y = 0.053;
			pose.position.z = 0.384;
			pose.orientation.x = 0.704;
			pose.orientation.y = 0.710;
			pose.orientation.z = 0.012;
			pose.orientation.w = 0.010;
			Motion(pose,0.8);
		}

		void GoToPre_tube1_tube2()
		{
			geometry_msgs::Pose pose;
			pose.position.x = 0.350;
			pose.position.y = 0.041;
			pose.position.z = 0.280;
			pose.orientation.x = 0.698;
			pose.orientation.y = 0.703;
			pose.orientation.z = 0.098;
			pose.orientation.w = 0.095;

			Motion(pose,0.5);
		}

		void GoToReady_tube1_tube2_1()
		{
			geometry_msgs::Pose pose;
			pose.position.x = 0.351;
			pose.position.y = 0.038;
			pose.position.z = 0.219;
			pose.orientation.x = 0.673;
			pose.orientation.y = 0.678;
			pose.orientation.z = 0.212;
			pose.orientation.w = 0.208;

			Motion(pose,0.5);
		}

		void GoToReady_tube1_tube2_2()
		{
			geometry_msgs::Pose pose;
			pose.position.x = 0.428;
			pose.position.y = 0.044;
			pose.position.z = 0.248;
			pose.orientation.x = 0.702;
			pose.orientation.y = 0.707;
			pose.orientation.z = 0.062;
			pose.orientation.w = 0.059;

			Motion(pose,0.5);			
		}

		// very close to limit (better use 1 and 2 above)
		// void GoToReady_tube1_tube2_final()
		// {
		// 	geometry_msgs::Pose pose;
		// 	pose.position.x = 0.434;
		// 	pose.position.y = 0.039;
		// 	pose.position.z = 0.250;
		// 	pose.orientation.x = 0.694;
		// 	pose.orientation.y = 0.717;
		// 	pose.orientation.z = 0.055;
		// 	pose.orientation.w = 0.037;

		// 	Motion(pose,0.5);			
		// }

		void GoToPushPosition()

		{
			geometry_msgs::Pose pose;
			pose.position.x = 0.517;
			pose.position.y = 0.149;
			pose.position.z = 0.406;
			pose.orientation.x = 0.991;
			pose.orientation.y = -0.036;
			pose.orientation.z = -0.013;
			pose.orientation.w = -0.131;

			// move the arm
			Motion(pose,0.8);
		}

		// void GoToPushPosition_2()

		// {
		// 	geometry_msgs::Pose pose;
		// 	pose.position.x = 0.513;
		// 	pose.position.y = 0.139;
		// 	pose.position.z = 0.406;
		// 	pose.orientation.x = 0.993;
		// 	pose.orientation.y = -0.039;
		// 	pose.orientation.z = 0.011;
		// 	pose.orientation.w = -0.113;

		// 	// move the arm
		// 	Motion(pose,0.8);
		// }

		void GoToViewPosition()

		{
			geometry_msgs::Pose pose;
			pose.position.x = 0.411;
			pose.position.y = 0.156;
			pose.position.z = 0.404;
			pose.orientation.x = 1.000;
			pose.orientation.y = 0.012;
			pose.orientation.z = 0.023;
			pose.orientation.w = -0.006;
			Motion(pose,0.8);
		}



		void Insert_model_once()
		{
			// pre insert
			tf::TransformListener listener_;
			tf::StampedTransform transform_cable_ee;
			tf::StampedTransform transform_base_tube;
			tf::Transform transform_base_ee_goal;
			tf::TransformBroadcaster broadcaster;
			tf::TransformBroadcaster br_wire;
			tf::StampedTransform transform;
			tf::Transform transform_wire;

	        listener_.waitForTransform("/base_link", "end_effector_default",ros::Time(0), ros::Duration(3.0));
        	listener_.lookupTransform("/base_link", "end_effector_default",ros::Time(0), transform);
        	double x_min = transform.getOrigin().x()-0.1;
        	double x_max = transform.getOrigin().x()+0.1;
        	double y_min = transform.getOrigin().y()-0.2;
        	double y_max = transform.getOrigin().y()-0.02;
        	double z_min = 0.12;
        	double z_max = transform.getOrigin().z()-0.23+0.02; // 0.23 - > finger offset
        	int selected = 0;

        	cable_model(x_min, x_max, y_min, y_max, z_min, z_max, selected);
        	transform_wire.setOrigin(tf::Vector3(transform_cable.translation.x, transform_cable.translation.y, transform_cable.translation.z));
      		transform_wire.setRotation(tf::Quaternion(transform_cable.rotation.x, transform_cable.rotation.y, transform_cable.rotation.z, transform_cable.rotation.w));


        	br_wire.sendTransform(tf::StampedTransform(transform_wire, ros::Time::now(), "end_effector_default", "cable"));


			try{
		          listener_.waitForTransform("cable", "end_effector",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("cable", "end_effector",ros::Time(0), transform_cable_ee);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			try{
		          listener_.waitForTransform("base_link", "thick_tube1",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "thick_tube1",ros::Time(0), transform_base_tube);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			transform_base_ee_goal = transform_base_tube * transform_cable_ee; // the orders matter !!!!

			broadcaster.sendTransform(tf::StampedTransform(transform_base_ee_goal,ros::Time::now(),"base_link", "end_effector_target"));

			geometry_msgs::Pose pre_pose;
			pre_pose.position.x = transform_base_ee_goal.getOrigin().x();
			pre_pose.position.y = transform_base_ee_goal.getOrigin().y();
			pre_pose.position.z = transform_base_ee_goal.getOrigin().z();
			pre_pose.orientation.x = transform_base_ee_goal.getRotation().x();
			pre_pose.orientation.y = transform_base_ee_goal.getRotation().y();
			pre_pose.orientation.z = transform_base_ee_goal.getRotation().z();
			pre_pose.orientation.w = transform_base_ee_goal.getRotation().w();

			Motion(pre_pose,0.5);

			// insert
			geometry_msgs::Pose pose = pre_pose;
			pose.position.y -= 0.03;
			pose.position.z -= 0.11;

			Motion(pose,0.5);

		}

		void plan_by_finger()
		{
			// get the finger frame based on the tube1 frame
			// update the transform (translation and rotation)			
			tf::StampedTransform transform_finger;
			tf::TransformBroadcaster br_finger;
			tf::TransformBroadcaster br_ee;
			tf::StampedTransform transform_base_finger;
			tf::StampedTransform transform_finger_ee;
			tf::Transform transform_base_ee_goal;

			tf::Quaternion rot_quaternion;
			double r=1.0, p=3.14159, y=1.5708;  // Rotate the previous pose by 180* about X
			rot_quaternion.setRPY(r, p, y);
			rot_quaternion.normalize();
			transform_finger.setOrigin(tf::Vector3(0.0, 0.0, 0.1));
			transform_finger.setRotation(rot_quaternion);
			br_finger.sendTransform(tf::StampedTransform(transform_finger,ros::Time::now(),"thick_tube1", "finger_tube1_tube2"));
			// apply 45 degree rotation

			// get the ee pose
			try{
		          listener_.waitForTransform("base_link", "finger_tube1_tube2",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "finger_tube1_tube2",ros::Time(0), transform_base_finger);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			try{
		          listener_.waitForTransform("finger", "end_effector",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("finger", "end_effector",ros::Time(0), transform_finger_ee);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			transform_base_ee_goal = transform_base_finger * transform_finger_ee; // the orders matter !!!!

			br_ee.sendTransform(tf::StampedTransform(transform_base_ee_goal,ros::Time::now(),"base_link", "end_effector_target"));

			geometry_msgs::Pose pose_ee;
			pose_ee.position.x = transform_base_ee_goal.getOrigin().x();
			pose_ee.position.y = transform_base_ee_goal.getOrigin().y();
			pose_ee.position.z = transform_base_ee_goal.getOrigin().z();
			pose_ee.orientation.x = transform_base_ee_goal.getRotation().x();
			pose_ee.orientation.y = transform_base_ee_goal.getRotation().y();
			pose_ee.orientation.z = transform_base_ee_goal.getRotation().z();
			pose_ee.orientation.w = transform_base_ee_goal.getRotation().w();

			// here we can get the ee pose for grasping the cable in between tube1 and tube2

		}

		void tube1_tube2_pre()
		{
			GoToInitPosition();
			Gripper(0.77);
			geometry_msgs::Pose pose_target;
			pose_target.position.x = 0.334;
			pose_target.position.y = 0.216;
			pose_target.position.z = 0.350;
			pose_target.orientation.x = 1.000;
			pose_target.orientation.y = 0.017;
			pose_target.orientation.z = 0.011;
			pose_target.orientation.w = 0.007;
			Motion(pose_target,0.5);
			ros::Duration(1.0).sleep();

			pose_target.position.x = 0.555;
			pose_target.position.y = 0.211;
			pose_target.position.z = 0.358;
			pose_target.orientation.x = 1.000;
			pose_target.orientation.y = 0.017;
			pose_target.orientation.z = 0.011;
			pose_target.orientation.w = 0.007;
			Motion(pose_target,0.8);
			ros::Duration(1.0).sleep();
		}


		void tube1_tube2_new()
		{
			
			GoToInit_tube1_tube2();
			ros::Duration(5.0).sleep();
			Gripper(0.0);
			ros::Duration(0.5).sleep();
			GoToPre_tube1_tube2();
			ros::Duration(2.0).sleep();
			GoToReady_tube1_tube2_1();
			Gripper(0.77);
			GoToReady_tube1_tube2_2();
			Gripper(0.0);
			GoToInit_tube1_tube2();
			ros::Duration(2.0).sleep();
		}
		void tube1_tube2()
		{	// declaration of transforms and broadcaster // should declare in the begining of the function
			tf::StampedTransform transform_finger;
			tf::TransformBroadcaster br_finger_final;
			tf::TransformBroadcaster br_ee;
			tf::StampedTransform transform_base_finger;
			tf::StampedTransform transform_finger_ee;
			tf::Transform transform_base_ee_goal;


			// // below is hard-coding, but we can use plan_by_finger function to get the ee pose for plan
			// GoToInitPosition();
			GoToInit_tube1_tube2();
			// GoToPre_tube1_tube2();
			 // GoToReady_tube1_tube2();
			// // close the gripper
			// Gripper(0.77);

			// motion plan in the constrained space
			// get the final pose of the finger
			// parameters: 2d pose of the circle center C(x: 0.42,z: 0.065); 2d pose of tube2 bottom A(x: 0.40, z:0.01); 2d pose of tube2 top B(x: 0.40, z:0.04)

			// get theta cos(theta) = (CA^2+CB^2-BA^2)/(2*CA*CB)
			double ca=sqrt(pow((0.42-0.40),2)+pow((0.065-0.01),2));
			double cb=sqrt(pow((0.42-0.40),2)+pow((0.065-0.04),2));
			double ba=sqrt(pow((0.40-0.40),2)+pow((0.01-0.04),2));
			double theta = acos((pow(ca,2)+pow(cb,2)-pow(ba,2))/(2*ca*cb));

			// get radius
			try{
		          listener_.waitForTransform("base_link", "finger",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "finger",ros::Time(0), transform_base_finger);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			double grasp_x = transform_base_finger.getOrigin().x();
			double grasp_z = transform_base_finger.getOrigin().z();

			std::cout << "grasp_x: " << grasp_x << std::endl;
			std::cout << "grasp_z: " << grasp_z << std::endl;

			
			double radius = sqrt(pow((grasp_x-0.42),2)+pow((grasp_z-0.065),2));

			//get 2d final pose of finger
			double final_x = 0.42+(grasp_x-0.42)*cos(theta)-(grasp_z-0.065)*sin(theta);
			double final_z = 0.065+(grasp_z-0.065)*cos(theta)+(grasp_x-0.42)*sin(theta);

			std::cout << "final_x: " << final_x << std::endl;
			std::cout << "final_z: " << final_z << std::endl;

			// transfer frame finger to ee to do motion plan			
			
			tf::Quaternion rot_quaternion;
			double r=theta, p=0.0, y=0.0;  // Rotate the previous pose by theta about X
			rot_quaternion.setRPY(r, p, y);
			rot_quaternion.normalize();
			double diff_z = final_x-grasp_x-0.01;
			double diff_x = 0.0;
			double diff_y = final_z-grasp_z-0.035;

			transform_finger.setOrigin(tf::Vector3(diff_x, 0.0, diff_z));
			transform_finger.setRotation(rot_quaternion);
			br_finger_final.sendTransform(tf::StampedTransform(transform_finger,ros::Time::now(),"finger", "finger_final"));

			// get the ee pose
			try{
		          listener_.waitForTransform("base_link", "finger_final",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "finger_final",ros::Time(0), transform_base_finger);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			try{
		          listener_.waitForTransform("finger", "end_effector",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("finger", "end_effector",ros::Time(0), transform_finger_ee);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			transform_base_ee_goal = transform_base_finger * transform_finger_ee; // the orders matter !!!!

			br_ee.sendTransform(tf::StampedTransform(transform_base_ee_goal,ros::Time::now(),"base_link", "end_effector_target"));

			geometry_msgs::Pose pose_ee;
			pose_ee.position.x = transform_base_ee_goal.getOrigin().x();
			pose_ee.position.y = transform_base_ee_goal.getOrigin().y();
			pose_ee.position.z = transform_base_ee_goal.getOrigin().z();
			pose_ee.orientation.x = transform_base_ee_goal.getRotation().x();
			pose_ee.orientation.y = transform_base_ee_goal.getRotation().y();
			pose_ee.orientation.z = transform_base_ee_goal.getRotation().z();
			pose_ee.orientation.w = transform_base_ee_goal.getRotation().w();

			// move the end_effector, the cable should go into the tube2
			// Motion(pose_ee,0.5);



		}

		void push_once(int grasp_id, int target_id)
		{

			GoToPushPosition();
			ros::Duration(3.0).sleep();
			Gripper(0.0);
			ros::Duration(1.0).sleep();

			// rosservice call /nist/cable_modeling "{x_min: 0.31, x_max: 0.51, y_min: -0.18, y_max: 0.02, z_min: 0.12, z_max: 0.24, selected: 3}"
			tf::StampedTransform transform;
			tf::Transform transform_wire;
			tf::Transform transform_wire_target;
			tf::TransformBroadcaster br_wire;
			tf::TransformBroadcaster br_wire_target;
			tf::Transform transform_base_ee_goal;
			tf::StampedTransform transform_base_cable;
			tf::StampedTransform transform_base_cable_target;
			tf::StampedTransform transform_finger_ee;
			tf::TransformListener listener_;

			listener_.waitForTransform("/base_link", "finger",ros::Time(0), ros::Duration(3.0));
        	listener_.lookupTransform("/base_link", "finger",ros::Time(0), transform);
        	double x_min = transform.getOrigin().x()-0.1;
        	double x_max = transform.getOrigin().x()+0.1;
        	double y_min = transform.getOrigin().y()-0.3;
        	double y_max = transform.getOrigin().y()-0.01;
        	double z_min = 0.122;
        	double z_max = transform.getOrigin().z()+0.01; 
        	// get grasp pose    	
        	int selected_grasp = grasp_id;
        	cable_model(x_min, x_max, y_min, y_max, z_min, z_max, selected_grasp);
        	transform_wire.setOrigin(tf::Vector3(transform_cable.translation.x, transform_cable.translation.y, transform_cable.translation.z));
      		transform_wire.setRotation(tf::Quaternion(transform_cable.rotation.x, transform_cable.rotation.y, transform_cable.rotation.z, transform_cable.rotation.w));
			sleep(1.0);
			// get target pose
			int selected_target = target_id;
        	cable_model(x_min, x_max, y_min, y_max, z_min, z_max, selected_target);
        	transform_wire_target.setOrigin(tf::Vector3(transform_cable.translation.x, transform_cable.translation.y, transform_cable.translation.z));
      		transform_wire_target.setRotation(tf::Quaternion(transform_cable.rotation.x, transform_cable.rotation.y, transform_cable.rotation.z, transform_cable.rotation.w));		

      		// get the ee pose for grasping
			br_wire.sendTransform(tf::StampedTransform(transform_wire, ros::Time::now(), "end_effector_default", "cable"));
			try{
		          listener_.waitForTransform("base_link", "cable",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "cable",ros::Time(0), transform_base_cable);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			try{
		          listener_.waitForTransform("finger", "end_effector",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("finger", "end_effector",ros::Time(0), transform_finger_ee);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}
			sleep(1.0);
			// get the ee target pose for pushing
			br_wire_target.sendTransform(tf::StampedTransform(transform_wire_target, ros::Time::now(), "end_effector_default", "cable_target"));
			
			try{
		          listener_.waitForTransform("base_link", "cable_target",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "cable_target",ros::Time(0), transform_base_cable_target);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			try{
		          listener_.waitForTransform("finger", "end_effector",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("finger", "end_effector",ros::Time(0), transform_finger_ee);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			transform_base_ee_goal = transform_base_cable * transform_finger_ee; // the orders matter !!!!

			br_wire.sendTransform(tf::StampedTransform(transform_base_ee_goal,ros::Time::now(),"base_link", "end_effector_target"));

			geometry_msgs::Pose pose_grasp;
			pose_grasp.position.x = transform_base_ee_goal.getOrigin().x();
			pose_grasp.position.y = transform_base_ee_goal.getOrigin().y();
			pose_grasp.position.z = transform_base_ee_goal.getOrigin().z();
			pose_grasp.orientation.x = transform_base_ee_goal.getRotation().x();
			pose_grasp.orientation.y = transform_base_ee_goal.getRotation().y();
			pose_grasp.orientation.z = transform_base_ee_goal.getRotation().z();
			pose_grasp.orientation.w = transform_base_ee_goal.getRotation().w();

			Motion(pose_grasp,0.5);
			// close the gripper
			Gripper(0.77);

			ros::Duration(1.0).sleep();
			

			transform_base_ee_goal = transform_base_cable_target * transform_finger_ee; // the orders matter !!!!

			br_wire_target.sendTransform(tf::StampedTransform(transform_base_ee_goal,ros::Time::now(),"base_link", "end_effector_target"));

			geometry_msgs::Pose pose_target;
			pose_target.position.x = transform_base_ee_goal.getOrigin().x();
			pose_target.position.y = transform_base_ee_goal.getOrigin().y();
			pose_target.position.z = transform_base_ee_goal.getOrigin().z();
			pose_target.orientation.x = transform_base_ee_goal.getRotation().x();
			pose_target.orientation.y = transform_base_ee_goal.getRotation().y();
			pose_target.orientation.z = transform_base_ee_goal.getRotation().z();
			pose_target.orientation.w = transform_base_ee_goal.getRotation().w();

			Motion(pose_target,0.5);
			Gripper(0.0);

		}

		void tube2_tube3()
		{
			push_once(9,3);
			GoToPushPosition();
			GoToViewPosition();
			srv_cable_plane.request.region = 1;
			if (cable_2d_client.call(srv_cable_plane))
			{
				ROS_INFO("Successed to call cable plane model service");
			}
			else
			{
				ROS_ERROR("Failed to call cable plane model service");
				exit(1);
			}
			double length = srv_cable_plane.response.length;
			if (length < 0.02) {
				tube2_tube3();
			}

		}

		// region 1
		void tube3_pre()
		{	

			// GoToViewPosition();
			// ros::Duration(4.0).sleep();


			// // for finger safety, move the cable position for grasp
			// Gripper(0.77);
			// ros::Duration(1.0).sleep();

			geometry_msgs::Pose pose_target;

			// pose_target.position.x = 0.421;
			// pose_target.position.y = 0.088;
			// pose_target.position.z = 0.241;
			// pose_target.orientation.x = 0.921;
			// pose_target.orientation.y = 0.390;
			// pose_target.orientation.z = -0.005;
			// pose_target.orientation.w = 0.007;

			// Motion(pose_target,0.8);
			// ros::Duration(2.0).sleep();

			// pose_target.position.x = 0.379;
			// pose_target.position.y = 0.045;
			// pose_target.position.z = 0.241;
			// pose_target.orientation.x = 0.921;
			// pose_target.orientation.y = 0.390;
			// pose_target.orientation.z = 0.002;
			// pose_target.orientation.w = 0.006;

			// Motion(pose_target,0.8);
			// ros::Duration(2.0).sleep();

			GoToViewPosition();
			ros::Duration(5.0).sleep();

			// variables
			tf::StampedTransform transform_base_camera;
			tf::TransformListener listener_;
			tf::Transform transform_base_cable_temp;
			tf::Transform transform_base_cable;
			tf::Transform transform_base_cable_tip;
			tf::StampedTransform transform_camera_cable;
			tf::StampedTransform transform_tip_ee;
			tf::StampedTransform transform_base_tube;
			tf::Transform transform_base_Nee;
			tf::StampedTransform transform_base_ee;

			// get the info from the service
			double grasp_x,grasp_y,grasp_z, grasp_theta;
			double tip_x, tip_y, tip_z, tip_theta;
			srv_cable_plane.request.region = 1;
			if (cable_2d_client.call(srv_cable_plane))
			{
				ROS_INFO("Successed to call cable plane model service");
			}
			else
			{
				ROS_ERROR("Failed to call cable plane model service");
				exit(1);
			}
			grasp_x = srv_cable_plane.response.grasp_x;
			grasp_y = srv_cable_plane.response.grasp_y;
			grasp_z = srv_cable_plane.response.grasp_z;
			grasp_theta = srv_cable_plane.response.grasp_theta;
			tip_x = srv_cable_plane.response.tip_x;
			tip_y = srv_cable_plane.response.tip_y;
			tip_z = srv_cable_plane.response.tip_z;
			tip_theta = srv_cable_plane.response.tip_theta;

			// process the data		
			
			tf::Quaternion rot_quaternion;
			double r=0.0, p=0.0, y=0.0; 
			rot_quaternion.setRPY(r, p, y);
			rot_quaternion.normalize();

			transform_camera_cable.setOrigin(tf::Vector3(grasp_x, grasp_y, grasp_z));
			transform_camera_cable.setRotation(rot_quaternion);
			try{
		          listener_.waitForTransform("base_link", "camera_link",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "camera_link",ros::Time(0), transform_base_camera);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			transform_base_cable_temp = transform_base_camera * transform_camera_cable;

			r = 3.14159;
			p = 0.0;
			y = -(1.57+grasp_theta);
			rot_quaternion.setRPY(r, p, y);
			rot_quaternion.normalize();

			transform_base_cable.setOrigin(tf::Vector3(transform_base_cable_temp.getOrigin().x(),transform_base_cable_temp.getOrigin().y(),transform_base_cable_temp.getOrigin().z())); // offset finger to ee
			transform_base_cable.setRotation(rot_quaternion);
			br_cable.sendTransform(tf::StampedTransform(transform_base_cable,ros::Time::now(),"base_link", "cable_ee"));

			std::cout << "xyz: " << transform_base_cable.getOrigin().x() << ", " << transform_base_cable.getOrigin().y() << ", " << transform_base_cable.getOrigin().z() << std::endl;
			std::cout << "xyzw: " << transform_base_cable.getRotation().x() << ", " << transform_base_cable.getRotation().y() << ", " << transform_base_cable.getRotation().z() << ", " << transform_base_cable.getRotation().w() << std::endl;
			// calculation result xyz 0.3781 0.0612 0.2194; xyzw -0.5936 0.8047 0.0 0.0
			// test working: xyz 0.378 0.055 0.245; xyzw -0.603 0.797 0.0 0.019
			
			// get the tip frame
			transform_camera_cable.setOrigin(tf::Vector3(tip_x, tip_y, tip_z));
			transform_camera_cable.setRotation(rot_quaternion);
			try{
		          listener_.waitForTransform("base_link", "camera_link",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "camera_link",ros::Time(0), transform_base_camera);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			transform_base_cable_temp = transform_base_camera * transform_camera_cable;

			r = 3.14159;
			p = 0.0;
			y = -(1.57+tip_theta);
			rot_quaternion.setRPY(r, p, y);
			rot_quaternion.normalize();

			transform_base_cable_tip.setOrigin(tf::Vector3(transform_base_cable_temp.getOrigin().x(),transform_base_cable_temp.getOrigin().y(),transform_base_cable_temp.getOrigin().z())); // offset finger to ee
			transform_base_cable_tip.setRotation(rot_quaternion);
			br_cable.sendTransform(tf::StampedTransform(transform_base_cable_tip,ros::Time::now(),"base_link", "cable_tip"));
			br_cable.sendTransform(tf::StampedTransform(transform_base_cable,ros::Time::now(),"base_link", "cable_ee"));

			std::cout << "xyz: " << transform_base_cable_tip.getOrigin().x() << ", " << transform_base_cable_tip.getOrigin().y() << ", " << transform_base_cable_tip.getOrigin().z() << std::endl;
			std::cout << "xyzw: " << transform_base_cable_tip.getRotation().x() << ", " << transform_base_cable_tip.getRotation().y() << ", " << transform_base_cable_tip.getRotation().z() << ", " << transform_base_cable_tip.getRotation().w() << std::endl;
			try{
		          listener_.waitForTransform("cable_tip", "cable_ee",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("cable_tip", "cable_ee",ros::Time(0), transform_tip_ee);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}
			std::cout << "xyz: " << transform_tip_ee.getOrigin().x() << ", " << transform_tip_ee.getOrigin().y() << ", " << transform_tip_ee.getOrigin().z() << std::endl;
			std::cout << "xyzw: " << transform_tip_ee.getRotation().x() << ", " << transform_tip_ee.getRotation().y() << ", " << transform_tip_ee.getRotation().z() << ", " << transform_tip_ee.getRotation().w() << std::endl;

			try{
		          listener_.waitForTransform("base_link", "thick_tube3",ros::Time(0), ros::Duration(3.0));
		          listener_.lookupTransform("base_link", "thick_tube3",ros::Time(0), transform_base_tube);
			}
			catch (tf::TransformException ex) {
			  ROS_ERROR("%s",ex.what());
			}

			transform_base_Nee = transform_base_tube * transform_tip_ee;
			br_cable.sendTransform(tf::StampedTransform(transform_base_Nee,ros::Time::now(),"base_link", "ee_goal"));
			std::cout << "xyz: " << transform_base_Nee.getOrigin().x() << ", " << transform_base_Nee.getOrigin().y() << ", " << transform_base_Nee.getOrigin().z() << std::endl;
			std::cout << "xyzw: " << transform_base_Nee.getRotation().x() << ", " << transform_base_Nee.getRotation().y() << ", " << transform_base_Nee.getRotation().z() << ", " << transform_base_Nee.getRotation().w() << std::endl;
			// calculation result xyz 0.3914 0.0741 0.02877; xyzw -0.0722 0.9973 0.0 0.0008
			// test working: xyz 0.392 0.046 0.252; xyzw -0.024 1.000 0.016 0.004


			// go and grasp the cable on the board
			Gripper(0.6);
			ros::Duration(1.0).sleep();
			

		
			pose_target.position.x = transform_base_cable.getOrigin().x()-0.015;
			pose_target.position.y = transform_base_cable.getOrigin().y()+0.006;
			pose_target.position.z = 0.236+0.1;
			pose_target.orientation.x = transform_base_cable.getRotation().x();
			pose_target.orientation.y = transform_base_cable.getRotation().y();
			pose_target.orientation.z = transform_base_cable.getRotation().z();
			pose_target.orientation.w = transform_base_cable.getRotation().w();
			Motion(pose_target,0.8);
			ros::Duration(5.0).sleep();

			pose_target.position.x = transform_base_cable.getOrigin().x()-0.015;
			pose_target.position.y = transform_base_cable.getOrigin().y()+0.006;
			pose_target.position.z = 0.236;
			pose_target.orientation.x = transform_base_cable.getRotation().x();
			pose_target.orientation.y = transform_base_cable.getRotation().y();
			pose_target.orientation.z = transform_base_cable.getRotation().z();
			pose_target.orientation.w = transform_base_cable.getRotation().w();
			Motion(pose_target,0.8);
			ros::Duration(3.0).sleep();

			Gripper(0.77);
			ros::Duration(1.0).sleep();

			// // rotate the cable 
			// pose_target.position.x = 0.390;
			// pose_target.position.y = 0.048;
			// pose_target.position.z = 0.248;
			// pose_target.orientation.x = 0.010;
			// pose_target.orientation.y = 1.000;
			// pose_target.orientation.z = -0.004;
			// pose_target.orientation.w = 0.004;
			// Motion(pose_target,0.8);
			// ros::Duration(4.0).sleep();

			// // insert to the tube 3
			// try{
			// 	listener_.waitForTransform("base_link", "end_effector", ros::Time(0), ros::Duration(3.0));
			// 	listener_.lookupTransform("base_link", "end_effector", ros::Time(0), transform_base_ee);
			// }
			// catch (tf::TransformException ex) {
			// 	ROS_ERROR("%s", ex.what());
			// }

			// pose_target.position.x = transform_base_ee.getOrigin().x();
			// pose_target.position.y = transform_base_ee.getOrigin().y() + 0.022;
			// pose_target.position.z = transform_base_ee.getOrigin().z();
			// pose_target.orientation.x = transform_base_ee.getRotation().x();
			// pose_target.orientation.y = transform_base_ee.getRotation().y();
			// pose_target.orientation.z = transform_base_ee.getRotation().z();
			// pose_target.orientation.w = transform_base_ee.getRotation().w();
			// Motion(pose_target, 0.5);
			// ros::Duration(3.0).sleep();
			// Gripper(0.6);
			// ros::Duration(1.0).sleep();

			// try{
			// 	listener_.waitForTransform("base_link", "end_effector", ros::Time(0), ros::Duration(3.0));
			// 	listener_.lookupTransform("base_link", "end_effector", ros::Time(0), transform_base_ee);
			// }
			// catch (tf::TransformException ex) {
			// 	ROS_ERROR("%s", ex.what());
			// }

			// pose_target.position.x = transform_base_ee.getOrigin().x();
			// pose_target.position.y = transform_base_ee.getOrigin().y();
			// pose_target.position.z = transform_base_ee.getOrigin().z()+0.1;
			// pose_target.orientation.x = transform_base_ee.getRotation().x();
			// pose_target.orientation.y = transform_base_ee.getRotation().y();
			// pose_target.orientation.z = transform_base_ee.getRotation().z();
			// pose_target.orientation.w = transform_base_ee.getRotation().w();
			// Motion(pose_target, 0.8);
			// ros::Duration(3.0).sleep();


			// // go to init position
			// GoToInit_tube1_tube2();
			// ros::Duration(5.0).sleep();

/*	some hard coding, ignore		
			// pre-grasp
			pose_target.position.x = 0.394;
			pose_target.position.y = 0.058;
			pose_target.position.z = 0.298;
			pose_target.orientation.x = -0.532;
			pose_target.orientation.y = 0.847;
			pose_target.orientation.z = 0.003;
			pose_target.orientation.w = 0.010;

			Motion(pose_target,0.8);
			ros::Duration(2.0).sleep();

			// go and grasp
			pose_target.position.x = 0.392;
			pose_target.position.y = 0.057;
			pose_target.position.z = 0.238;
			pose_target.orientation.x = -0.532;
			pose_target.orientation.y = 0.847;
			pose_target.orientation.z = 0.003;
			pose_target.orientation.w = 0.010;

			Motion(pose_target,0.6);
			ros::Duration(3.0).sleep();
			Gripper(0.77);
			ros::Duration(0.5).sleep();

			// move/rotate the tip to the front of the tube 3
			
			// pose_target.position.x = transform_base_Nee.getOrigin().x();
			// pose_target.position.y = transform_base_Nee.getOrigin().y();
			// pose_target.position.z = transform_base_Nee.getOrigin().z()+0.215;
			// pose_target.orientation.x = transform_base_Nee.getRotation().x();
			// pose_target.orientation.y = transform_base_Nee.getRotation().y();
			// pose_target.orientation.z = transform_base_Nee.getRotation().z();
			// pose_target.orientation.w = transform_base_Nee.getRotation().w();

			pose_target.position.x = 0.390;
			pose_target.position.y = 0.042;
			pose_target.position.z = 0.250;
			pose_target.orientation.x = 0.135;
			pose_target.orientation.y = 0.991;
			pose_target.orientation.z = 0.009;
			pose_target.orientation.w = 0.006;
			Motion(pose_target,0.5);
			ros::Duration(2.0).sleep();

			// insert to the tube 3
			try{
				listener_.waitForTransform("base_link", "end_effector", ros::Time(0), ros::Duration(3.0));
				listener_.lookupTransform("base_link", "end_effector", ros::Time(0), transform_base_ee);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("%s", ex.what());
			}

			pose_target.position.x = transform_base_ee.getOrigin().x();
			pose_target.position.y = transform_base_ee.getOrigin().y() + 0.023;
			pose_target.position.z = transform_base_ee.getOrigin().z();
			pose_target.orientation.x = transform_base_ee.getRotation().x();
			pose_target.orientation.y = transform_base_ee.getRotation().y();
			pose_target.orientation.z = transform_base_ee.getRotation().z();
			pose_target.orientation.w = transform_base_ee.getRotation().w();
			Motion(pose_target, 0.5);
			ros::Duration(3.0).sleep();
			Gripper(0.6);
			ros::Duration(1.0).sleep();

			// go to pre-grasp
			pose_target.position.x = 0.394;
			pose_target.position.y = 0.058;
			pose_target.position.z = 0.298;
			pose_target.orientation.x = -0.532;
			pose_target.orientation.y = 0.847;
			pose_target.orientation.z = 0.003;
			pose_target.orientation.w = 0.010;

			Motion(pose_target,0.5);
			ros::Duration(2.0).sleep();

			// go to view position
			GoToViewPosition();
			ros::Duration(3.0).sleep();

*/
		}

		void tube3_ing()
		{
			// push the cable to thread through the tube 3

			push_once(15,2);
			GoToPushPosition();
			GoToViewPosition();
			srv_cable_plane.request.region = 2;
			if (cable_2d_client.call(srv_cable_plane))
			{
				ROS_INFO("Successed to call cable plane model service");
			}
			else
			{
				ROS_ERROR("Failed to call cable plane model service");
				exit(1);
			}
			double length = srv_cable_plane.response.length;
			if (length < 0.03) {
				tube3_ing();
			}

		}

		void tube3_post()
		{
			tf::StampedTransform transform_base_ee;
			// after the cable out of tube 3 for about 8cm, estimate the cable pose and grasp the cable
			Gripper(0.5);
			ros::Duration(1.0).sleep();
			geometry_msgs::Pose pose_target;

			// pre-grasp
			pose_target.position.x = 0.373;
			pose_target.position.y = 0.135;
			pose_target.position.z = 0.309;
			pose_target.orientation.x = -0.005;
			pose_target.orientation.y = 1.000;
			pose_target.orientation.z = -0.002;
			pose_target.orientation.w = -0.008;
			Motion(pose_target,0.8);
			ros::Duration(8.0).sleep();

			try{
				listener_.waitForTransform("base_link", "end_effector", ros::Time(0), ros::Duration(3.0));
				listener_.lookupTransform("base_link", "end_effector", ros::Time(0), transform_base_ee);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("%s", ex.what());
			}
			pose_target.position.x = transform_base_ee.getOrigin().x();
			pose_target.position.y = transform_base_ee.getOrigin().y();
			pose_target.position.z = 0.240;
			pose_target.orientation.x = transform_base_ee.getRotation().x();
			pose_target.orientation.y = transform_base_ee.getRotation().y();
			pose_target.orientation.z = transform_base_ee.getRotation().z();
			pose_target.orientation.w = transform_base_ee.getRotation().w();

			Motion(pose_target,0.5);
			ros::Duration(3.0).sleep();
			Gripper(0.77);
			ros::Duration(1.0).sleep();

			// use external force to stop stretching the cable
			// while (external_force_y < 10)
			// {
			// 	gen3_velocity_control(0.00, 0.01, 0.00, 0.00, 0.00, 0.00);
			// }

			// gen3_velocity_control(0.00, 0.00, 0.00, 0.00, 0.00, 0.00);


			// pull and tight the cable (pull about 23cm to the left)
			try{
				listener_.waitForTransform("base_link", "end_effector", ros::Time(0), ros::Duration(3.0));
				listener_.lookupTransform("base_link", "end_effector", ros::Time(0), transform_base_ee);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("%s", ex.what());
			}

			pose_target.position.x = transform_base_ee.getOrigin().x();
			pose_target.position.y = transform_base_ee.getOrigin().y() + 0.20;
			pose_target.position.z = transform_base_ee.getOrigin().z() + 0.01;
			pose_target.orientation.x = transform_base_ee.getRotation().x();
			pose_target.orientation.y = transform_base_ee.getRotation().y();
			pose_target.orientation.z = transform_base_ee.getRotation().z();
			pose_target.orientation.w = transform_base_ee.getRotation().w();
			Motion(pose_target, 0.8);
			ros::Duration(3.0).sleep();
			Gripper(0.3);
			ros::Duration(1.0).sleep();



		}

		void insert_final()
		{

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

	    double delta_roll, delta_pitch, delta_yaw; 
		double delta_x, delta_y, delta_z;
	    double roll_tip_ee, pitch_tip_ee, yaw_tip_ee;
		double roll_pre_ee, pitch_pre_ee, yaw_pre_ee;
		double roll_ee, pitch_ee, yaw_ee;
		double x_world, y_world, z_world;
	    tf::StampedTransform transform_tip_ee;
    	tf::StampedTransform transform_pre_ee;
    	tf::StampedTransform transform_tip;
    	tf::StampedTransform transform_pre_world;
    	tf::StampedTransform transform_tip_world;
    	tf::TransformListener listener_;
    	tf::TransformBroadcaster br_cable;
    	double external_force_y;
    	

	    //pid
	    float _dt = 0.1;
	    float _Kp;      // 2
	    float _Kd;    // 0.2
	    float _Ki;    
	    float _pre_error_x=0, _pre_error_y=0, _pre_error_z=0, _pre_error_roll=0, _pre_error_pitch=0, _pre_error_yaw=0;
	    float _integral_x=0, _integral_y=0, _integral_z=0, _integral_roll=0, _integral_pitch=0, _integral_yaw=0;
};


int main(int argc, char **argv)
{

	ros::init (argc, argv, "nist_trajopt");
	nist_task nist_cool;

	
	// // insert the cable and release the cable
	// nist_cool.Insert_visual_servoing("thick_tube1_pre");
	// nist_cool.Insert();
	// nist_cool.Gripper(0.0);
	// ros::Duration(1.0).sleep();

	// // push side of the cable to make the cable easy to grasp
	// nist_cool.tube1_tube2_pre();	

	// // insert the cable from tube1 to tube2
	// nist_cool.tube1_tube2_new();
	

	// // go to the push position and push the cable out of tube 2
	// nist_cool.tube2_tube3(); // push until length greater than 0.25
	// // nist_cool.push_once(13,2);
	// // nist_cool.GoToPushPosition();
	// // nist_cool.GoToViewPosition();

	// // into tube 3
	nist_cool.tube3_pre();

	// // pushing through tube 3
	// nist_cool.tube3_ing();

	// // after tube 3, go to ready insert to final position
	// nist_cool.tube3_post();
	
	return 0;
}