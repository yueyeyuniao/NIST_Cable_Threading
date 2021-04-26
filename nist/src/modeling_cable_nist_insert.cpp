#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <math.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>

#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <list>

#include <visualization_msgs/Marker.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <kinova_driver/kinova_ros_types.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <geometry_msgs/PointStamped.h>
#include <algorithm> // std::min_element, std::max_element

using namespace std;
geometry_msgs::Point center_object;
const double FINGER_MAX = 6400;
const double tip_length = 0.08;
const bool filter_plug = true;
bool gripper_closed = true;

class PointCloudProc
{
  public:
    PointCloudProc() : cloud_transformed_(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>),
                       cloud_hull(new pcl::PointCloud<pcl::PointXYZ>), cloud_raw_(new pcl::PointCloud<pcl::PointXYZ>)
    {

        //filter_range_ = 0.7;

        //planar_segment_src_ = nh_.advertiseService("planer_segment", &PointCloudProc::planarSegmentationCB, this);
        pc_sub_ = nh_.subscribe("/camera/depth/points", 1, &PointCloudProc::pointcloudcb, this);
        point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("transformed_point_cloud", 1000);
        seg_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("segmented_plane_point_cloud", 1000);
        filtered_point_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("filtered_point_cloud", 1000);
        marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        fixed_frame_ = "/base_link";
    }

    void pointcloudcb(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
    {   

        ros::Duration(0.3).sleep();        
    label:
        std::cout << "I am still in the loop" << std::endl;
        geometry_msgs::Point plane_normal;
        pcl::ExtractIndices<pcl::PointXYZ> extract_;
        pcl::ConvexHull<pcl::PointXYZ> chull;
        cloud_raw_ = msg;

        // cloud_transformed_ = cloud_raw_;
        // listener_.waitForTransform(fixed_frame_, "/kinect2_rgb_optical_frame", (ros::Time)(*cloud_raw_).header.stamp, ros::Duration(3.0));
        bool transform_success = pcl_ros::transformPointCloud(fixed_frame_, *cloud_raw_, *cloud_transformed_, listener_);
        //std::cout << transform_success << std::endl;
        //point_cloud_pub_.publish(cloud_transformed_);

        if (transform_success == 0) 
        {
          ROS_INFO("failed transform point cloud");
          return;
        }

        // get the position of the end-effector/finger_tip
        // listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
        // listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform);
        

        listener_.waitForTransform("/base_link", "finger",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("/base_link", "finger",ros::Time(0), transform);


        std::cout << "Position of endeffector: " << transform.getOrigin().x() << ", " << transform.getOrigin().y() << ", " << transform.getOrigin().z() << std::endl;
        // listener_.waitForTransform("/base_link", "j2n6s300_link_6",ros::Time(0), ros::Duration(3.0));
        // listener_.lookupTransform("/base_link", "j2n6s300_link_6",ros::Time(0), transform3);
        double finger_offset;
        if (gripper_closed == true)
        {
          finger_offset = 0.23;
        }
        else
        {
          finger_offset = 0.21;
        }


        //pub_plane.publish(cloud_transformed_);
        // filter point cloud based on detected object, gripper position, socket position
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_transformed_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(transform.getOrigin().x()-0.05, transform.getOrigin().x() + 0.05);
        pass.filter(*cloud_filtered_);
        pass.setInputCloud(cloud_filtered_);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(transform.getOrigin().y() - 0.2, transform.getOrigin().y() - 0.01); // 0.07 needs to change according to the ee's position
        pass.filter(*cloud_filtered_);
        pass.setInputCloud(cloud_filtered_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.12, transform.getOrigin().z()+0.01);  //0.35 is for testing disturbance
        pass.filter(*cloud_filtered_);

        ROS_INFO("Point cloud is filtered!");
        std::cout << cloud_filtered_->points.size() << " of points in the filtered point cloud" << std::endl;
        if (cloud_filtered_->points.size() == 0)
        {
          ROS_INFO("Point cloud is empty after filtering!");
          goto label;
        }


        // save the filtered point cloud to a txt
        // if (filter_flag == 0)
        // {
        //   //std::cout << cloud_filtered_->points.size() << std::endl;
        //   std::ofstream myfile;
        //   myfile.open ("/home/pengchang/Desktop/wire/filtered_wire_realsense_after_grasping_0123.txt", std::ofstream::out | std::ofstream::app);
        //   for (int i=0; i<cloud_filtered_->points.size(); i++)
        //   {
        //       myfile << cloud_filtered_->points[i].x << " " << cloud_filtered_->points[i].y << " " << cloud_filtered_->points[i].z << std::endl;      
        //   }
        //   myfile.close();
        //   filter_flag = 1;
        // }
        if (cloud_filtered_->points.size() != 0) {
        // fit the pointcloud
        CurveFit(cloud_filtered_);

        // publish filtered point cloud
        filtered_point_cloud_pub_.publish(cloud_filtered_);
        cloud_filtered_->points.clear();
        }
        // planar segmentation
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0); //z axis
        // // Create the segmentation object
        // pcl::SACSegmentation<pcl::PointXYZ> seg;
        // // Optional
        // seg.setOptimizeCoefficients(true);
        // // Mandatory set plane to be parallel to Z axis within a 15 degrees tolerance
        // seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
        // seg.setMaxIterations(500); // iteration limits decides segmentation goodness
        // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setAxis(axis);
        // seg.setEpsAngle(pcl::deg2rad(15.0f));
        // seg.setDistanceThreshold(0.01);
        // seg.setInputCloud(cloud_transformed_);
        // seg.segment(*inliers, *coefficients);

        // if (inliers->indices.size() == 0)
        //    {
        //        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        //        return;
        //    }

        // // extract inline points original point coulds
        // extract_.setInputCloud(cloud_transformed_);
        // extract_.setNegative(false);
        // extract_.setIndices(inliers);
        // extract_.filter(*cloud_plane);
        // ROS_INFO_STREAM("# of points in plane: " << cloud_plane->points.size());

        // // publish segmented point cloud
        // seg_point_cloud_pub_.publish(cloud_plane);

        // // Create a Convex Hull representation of the plane
        // chull.setInputCloud(cloud_plane);
        // chull.setDimension(2);
        // chull.reconstruct(*cloud_hull);

        // // Get plane center
        // Eigen::Vector4f center;
        // pcl::compute3DCentroid(*cloud_plane, center);
        // std::cout << "center: " << center[0] << "," << center[1] << "," << center[2] << std::endl;

        // // Get plane min and max values
        // Eigen::Vector4f min_vals, max_vals;
        // pcl::getMinMax3D(*cloud_plane, min_vals, max_vals); 

        // // Get plane polygon
        // for (int i = 0; i < cloud_hull->points.size(); i++)
        // {
        //     geometry_msgs::Point32 p;
        //     p.x = cloud_hull->points[i].x;
        //     p.y = cloud_hull->points[i].y;
        //     p.z = cloud_hull->points[i].z;
        // }  

        // // Get plane coefficients
        // // coefficients->values[0]

        // // Get plane normal
        // float length = sqrt(coefficients->values[0] * coefficients->values[0] +
        //                     coefficients->values[1] * coefficients->values[1] +
        //                     coefficients->values[2] * coefficients->values[2]);
        // plane_normal.x = coefficients->values[0] / length;
        // plane_normal.y = coefficients->values[1] / length;
        // plane_normal.z = coefficients->values[2] / length;
        // std::cout << "normal: " << plane_normal.x << "," << plane_normal.y << "," << plane_normal.z << std::endl;


    }
    void CurveFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {

      cout.precision(6);                        //set precision
      cout.setf(ios::fixed);


      xx.clear();
      yy.clear();
      zz.clear();
      aa.clear();
      bb.clear();
      av.clear();
      sparse_x.clear();
      sparse_y.clear();
      sparse_z.clear();
      points.points.clear();
      line_strip.points.clear();
      line_list.points.clear();

      for (int i=0; i<cloud->points.size(); i++)
      {
          xx.push_back(cloud->points[i].x);
          yy.push_back(cloud->points[i].y); 
          zz.push_back(cloud->points[i].z);      
      }
      
      N = xx.size(); // N is the no. of data pairs
      aa = PolyfitXY(yy,xx);  //x = aa[0]*y^0 + aa[1]*y^1 + aa[2]*y^2
      av.clear();
      bb = PolyfitXY(yy,zz);  //z = bb[0]*y^0 + bb[1]*y^1 + bb[2]*y^2

      std::cout << "POLYFIT PARAMETERS: a0 a1 a2 b0 b1 b2" << aa[0] << ", " << aa[1] << ", " << aa[2] << ", " << bb[0] << ", " << bb[1] << ", " << bb[2] << std::endl;
      // double tip_temp , end_temp;
      // if (yy[0] > yy[N-1]){
      //   tip_temp = yy[N-1];
      //   end_temp = yy[0];
      // }
      // else{
      //   tip_temp = yy[0];
      //   end_temp = yy[N-1];        
      // }

      // get the min and max
      double tip_temp , end_temp;
      tip_temp = *std::min_element(yy.begin(), yy.end());
      end_temp = *std::max_element(yy.begin(), yy.end());

      std::cout << "tip_temp:" << tip_temp << std::endl;

      // sample the wire
      int num_sample;

      num_sample = floor(N/60);

      double segment = (end_temp-tip_temp)/num_sample;
      for (double i = tip_temp; i < end_temp; i = i + segment)  //(-0.6 -0.386)
      {
        sparse_y.push_back(i);
        sparse_x.push_back(aa[0]+aa[1]*i+aa[2]*i*i);
        sparse_z.push_back(bb[0]+bb[1]*i+bb[2]*i*i);
      }


      std::cout << "Number of Sample Points after Filtering: " << sparse_x.size() << std::endl;

      // display the sparse points in rviz
      points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/base_link";
      points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
      points.ns = line_strip.ns = line_list.ns = "points_and_lines";
      points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;



      points.id = 0;
      line_strip.id = 1;
      line_list.id = 2;



      points.type = visualization_msgs::Marker::POINTS;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_list.type = visualization_msgs::Marker::LINE_LIST;

      // POINTS markers use x and y scale for width/height respectively
      points.scale.x = 0.02;
      points.scale.y = 0.02;

      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip.scale.x = 0.01;
      line_list.scale.x = 0.01;



      // Points are green
      points.color.g = 1.0f;
      points.color.a = 1.0;

      // Line strip is blue
      line_strip.color.b = 1.0;
      line_strip.color.a = 1.0;

      // Line list is red
      line_list.color.r = 1.0;
      line_list.color.a = 1.0;

      for (int i = 0; i < sparse_x.size()-1; i++)
      {
        geometry_msgs::Point p;
        p.x = sparse_x[i];
        p.y = sparse_y[i];
        p.z = sparse_z[i];
        points.points.push_back(p);
        line_strip.points.push_back(p);

        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.z += 0.04;
        line_list.points.push_back(p);
      }
      marker_pub.publish(points);
      marker_pub.publish(line_strip);
      marker_pub.publish(line_list);

      std::cout << "First point: " << points.points[0].x << "," << points.points[0].y << "," << points.points[0].z << std::endl;
      std::cout << "Second point: " << points.points[1].x << "," << points.points[1].y << "," << points.points[1].z << std::endl;
      
      // for the tip
      geometry_msgs::PointStamped points_root_0, points_root_1, points_ee_0, points_ee_1;
      points_root_0.header.frame_id = points_root_1.header.frame_id = "/base_link";
      points_root_0.header.stamp = points_root_1.header.stamp = ros::Time();
      points_root_0.point.x = points.points[0].x;
      points_root_1.point.x = points.points[1].x;
      points_root_0.point.y = points.points[0].y;
      points_root_1.point.y = points.points[1].y;
      points_root_0.point.z = points.points[0].z;
      points_root_1.point.z = points.points[1].z;
      listener2_.transformPoint("end_effector_default", points_root_0, points_ee_0);
      listener2_.transformPoint("end_effector_default", points_root_1, points_ee_1);

      std::cout << "Transformed first point: " << points_ee_0.point.x << "," << points_ee_0.point.y << "," << points_ee_0.point.z << std::endl;
      std::cout << "Transformed second point: " << points_ee_1.point.x << "," << points_ee_1.point.y << "," << points_ee_1.point.z << std::endl;

      // for the wire
      
      int selected0 = 3;
      int selected1 = 4;
      // if ((tip_temp +0.68) > 0.002){
      //    selected0 = 0;
	     //   selected1 = 1;
      // }

      geometry_msgs::PointStamped points_root_w0, points_root_w1, points_ee_w0, points_ee_w1;
      points_root_w0.header.frame_id = points_root_w1.header.frame_id = "/base_link";
      points_root_w0.header.stamp = points_root_w1.header.stamp = ros::Time();
      points_root_w0.point.x = points.points[selected0].x;
      points_root_w1.point.x = points.points[selected1].x;
      points_root_w0.point.y = points.points[selected0].y;
      points_root_w1.point.y = points.points[selected1].y;
      points_root_w0.point.z = points.points[selected0].z;
      points_root_w1.point.z = points.points[selected1].z;
      listener2_.transformPoint("end_effector_default", points_root_w0, points_ee_w0);
      listener2_.transformPoint("end_effector_default", points_root_w1, points_ee_w1);

      std::cout << "Transformed first grasp point: " << points_ee_w0.point.x << "," << points_ee_w0.point.y << "," << points_ee_w0.point.z << std::endl;
      std::cout << "Transformed second grasp point: " << points_ee_w1.point.x << "," << points_ee_w1.point.y << "," << points_ee_w1.point.z << std::endl;

      // Yaw = 0 (in the ee frame) // publish the tip frame
      // roll_tip = atan((points_ee_0.point.y-points_ee_1.point.y)/sqrt(pow((points_ee_0.point.x-points_ee_1.point.x),2)+pow((points_ee_0.point.z-points_ee_1.point.z),2)));
      // pitch_tip = atan((points_ee_0.point.x-points_ee_1.point.x)/(points_ee_0.point.z-points_ee_1.point.z));
      // yaw_tip = 0;
      // tf::Transform transform_tip;
      // transform_tip.setOrigin(tf::Vector3(points.points[0].x, points.points[0].y, points.points[0].z));
      // transform_tip.setRotation(tf::createQuaternionFromRPY(roll_tip, pitch_tip, yaw_tip));
      // br_tip.sendTransform(tf::StampedTransform(transform_tip, ros::Time::now(), "/endeffector_default", "wire_tip"));

      // roll = 1.57; // publish the tip frame
      pitch_tip = 0;
      yaw_tip = atan(-(points_ee_0.point.x-points_ee_1.point.x)/sqrt(pow((points_ee_0.point.y-points_ee_1.point.y),2)+pow((points_ee_0.point.z-points_ee_1.point.z),2)));
      roll_tip = 1.57 + atan((points_ee_0.point.z-points_ee_1.point.z)/(points_ee_0.point.y-points_ee_1.point.y));

      tf::Transform transform_tip;
      transform_tip.setOrigin(tf::Vector3(points_ee_0.point.x, points_ee_0.point.y, points_ee_0.point.z));
      transform_tip.setRotation(tf::createQuaternionFromRPY(roll_tip, pitch_tip, yaw_tip));
      br_tip.sendTransform(tf::StampedTransform(transform_tip, ros::Time::now(), "end_effector_default", "cable_tip_insert"));

      // roll = 0; // publish the wire frame
      pitch_wire = 0;
      yaw_wire = atan(-(points_ee_w0.point.x-points_ee_w1.point.x)/sqrt(pow((points_ee_w0.point.y-points_ee_w1.point.y),2)+pow((points_ee_w0.point.z-points_ee_w1.point.z),2)));
      roll_wire = 1.57 + atan((points_ee_w0.point.z-points_ee_w1.point.z)/(points_ee_w0.point.y-points_ee_w1.point.y));
      tf::Transform transform_wire;
      transform_wire.setOrigin(tf::Vector3(points_ee_w0.point.x, points_ee_w1.point.y, points_ee_w0.point.z));
      transform_wire.setRotation(tf::createQuaternionFromRPY(roll_wire, pitch_wire, yaw_wire));
      br_wire.sendTransform(tf::StampedTransform(transform_wire, ros::Time::now(), "end_effector_default", "cable"));



    }

    std::vector<double> PolyfitXY(std::vector<double> x, std::vector<double> y)
    {
      int i,j,k;
      int n = 2; // n is the degree of polynomial
      double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      for (i=0;i<2*n+1;i++)
      {
          X[i]=0;
          for (j=0;j<N;j++)
              X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
      }
      double B[n+1][n+2];
      double a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
      for (i=0;i<=n;i++)
          for (j=0;j<=n;j++)
              B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
      double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      for (i=0;i<n+1;i++)
      {    
          Y[i]=0;
          for (j=0;j<N;j++)
          Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
      }
      for (i=0;i<=n;i++)
          B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
      n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
      cout<<"\nThe Normal(Augmented Matrix) is as follows:\n";    
      for (i=0;i<n;i++)            //print the Normal-augmented matrix
      {
          for (j=0;j<=n;j++)
              cout<<B[i][j]<<setw(16);
          cout<<"\n";
      }    
      for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
          for (k=i+1;k<n;k++)
              if (B[i][i]<B[k][i])
                  for (j=0;j<=n;j++)
                  {
                      double temp=B[i][j];
                      B[i][j]=B[k][j];
                      B[k][j]=temp;
                  }
      
      for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
          for (k=i+1;k<n;k++)
              {
                  double t=B[k][i]/B[i][i];
                  for (j=0;j<=n;j++)
                      B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
              }
      for (i=n-1;i>=0;i--)                //back-substitution
      {                        //x is an array whose values correspond to the values of x,y,z..
          a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
          for (j=0;j<n;j++)
              if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                  a[i]=a[i]-B[i][j]*a[j];
          a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
      }
      cout<<"\nThe values of the coefficients are as follows:\n";
      for (i=0;i<n;i++)
          cout<<"x^"<<i<<"="<<a[i]<<endl;            // Print the values of x^0,x^1,x^2,x^3,....    
      cout<<"\nHence the fitted Polynomial is given by:\ny=";
      for (i=0;i<n;i++)
          cout<<" + ("<<a[i]<<")"<<"x^"<<i;
      cout<<"\n";

      for (i=0;i<(n+1);i++)
      {
        av.push_back(a[i]);
      }
      return av;
    }


  private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Publisher plane_pub_;
    ros::Publisher point_cloud_pub_, seg_point_cloud_pub_, filtered_point_cloud_pub_;
    ros::Publisher marker_pub;
    ros::Publisher display_publisher;
    ros::ServiceServer planar_segment_src_;

    std::string fixed_frame_;


    tf::TransformListener listener_;
    tf::StampedTransform transform;

    tf::TransformListener listener2_;
    tf::StampedTransform transform2; 


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_, cloud_transformed_, cloud_filtered_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull;
    pcl::ConvexHull<pcl::PointXYZ> chull;
    int filter_flag = 0;
    //float filter_range_;
    //for curve fitting
    std::vector<double> xx;
    std::vector<double> yy;
    std::vector<double> zz;
    std::vector<double> aa;
    std::vector<double> bb;
    std::vector<double> av;
    std::vector<double> sparse_x;
    std::vector<double> sparse_y;
    std::vector<double> sparse_z;
    visualization_msgs::Marker points, line_strip, line_list;
    int n, N;
    tf::TransformBroadcaster br_tip, br_wire;
    float roll_tip, pitch_tip, yaw_tip;
    float roll_wire, pitch_wire, yaw_wire;
    geometry_msgs::Pose target_pose_wire1, target_pose_wire2, target_pose_wire3;
    bool isGrasp = false;
    bool isOut = false;
    double tip_initial;

    // new
    bool first_reading = true;



};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_segmentation");

    center_object.x = 0;
    center_object.y = 0;
    center_object.z = 0;
    PointCloudProc pc_tools;
    ROS_INFO("Initialized");
    ros::Rate r(60);
    ros::spin();
    return 0;
}
