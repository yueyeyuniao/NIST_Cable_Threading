#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
#include "nist/cable_plane_color.h"


static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;
// region 1
// max horizontal index x 333 (x zhou)
// max vertical index v 160 (y zhou)

// region 2
// max horizontal index x 369 (x zhou)
// max vertical index v 166 (y zhou)

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr depth_img_cv;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber info_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;

  Mat imgThresholded;
  int iLowH = 0;
  int iHighH = 0;

  int iLowS = 0; 
  int iHighS = 0;

  int iLowV = 0;
  int iHighV = 0;

  vector<int> pixel_x; // x of zip-tie in the 2d image
  vector<int> pixel_y; // y of zip-tie in the 2d image
  vector<float> co_x; // x coordinate in 3d with respect to camera
  vector<float> co_y; // y coordinate in 3d with respect to camera
  vector<float> co_z; // z coordinate in 3d with respect to camera
  image_geometry::PinholeCameraModel pin_hole_camera;

  std::vector<double> aa;
  std::vector<double> av;
  std::vector<double> xx;
  std::vector<double> yy;
  std::vector<double> zz;
  std::vector<double> sparse_x;
  std::vector<double> sparse_y;
  std::vector<double> sparse_z;
  int N;

  //ros::ServiceServer srv;
 

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    info_sub_ = nh_.subscribe("/camera/color/camera_info", 10, &ImageConverter::info_callback, this);
    image_sub_ = it_.subscribe("/camera/color/image_rect_color", 1, &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1, &ImageConverter::depthCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    //srv = nh_.advertiseService("/nist/cable_plane_color", &ImageConverter::image_process_srv, this);
    /*
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    //cv::namedWindow(OPENCV_WINDOW);
    */
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    pin_hole_camera.fromCameraInfo(msg);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(300);

    // // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());

    ////////////////////////////////////////////////////////////////////// color detection

    Mat imgHSV;

    cv::cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV


    //inRange(imgHSV, Scalar(0,0,0,0), Scalar(180,255,33.16,0), imgThresholded);  //black
    
    // inRange(imgHSV, Scalar(78,158,124), Scalar(138,255,255), imgThresholded);  //blue

    //test
    inRange(imgHSV, Scalar(110,150,150), Scalar(130,255,255), imgThresholded);


    //cout<<imgThresholded.size << endl; // 1080*1920 hd
    

    // delete redundent pixels outside of the cable box 2 // 333, 160
    // for (int i=0; i<639;i++) {
    //   for (int j=0; j<479; j++){
    //     if ((i<333-100) || (i>=333) || (j<160-100) || (j>=160)){
    //       imgThresholded.at<bool>(j,i)=0;
    //     }
    //   }
    // }

    // delete redundent pixels outside of the cable box 3 // 369, 166
    // for (int i=0; i<639;i++) {
    //   for (int j=0; j<479; j++){
    //     if ((i<369) || (i>=369+100) || (j<166-100) || (j>=166)){
    //       imgThresholded.at<bool>(j,i)=0;
    //     }
    //   }
    // }

    // cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    // cv::imshow("Original", cv_ptr->image); //show the original image

    // if (waitKey(300) >= 17) //wait for 'esc' key press for 300ms. If 'esc' key is pressed, break loop
    //    {
    //         cout << "esc key is pressed by user" << endl;
    //         exit(1);
    //    }
	

  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    try
    {
      depth_img_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //cv::imshow("Depth", depth_img_cv->image); //show the depth image

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
    // cout<<"\nThe Normal(Augmented Matrix) is as follows:\n";    
    // for (i=0;i<n;i++)            //print the Normal-augmented matrix
    // {
    //     for (j=0;j<=n;j++)
    //         cout<<B[i][j]<<setw(16);
    //     cout<<"\n";
    // }    
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
    // cout<<"\nThe values of the coefficients are as follows:\n";
    // for (i=0;i<n;i++)
    //     cout<<"x^"<<i<<"="<<a[i]<<endl;            // Print the values of x^0,x^1,x^2,x^3,....    
    // cout<<"\nHence the fitted Polynomial is given by:\ny=";
    // for (i=0;i<n;i++)
    //     cout<<" + ("<<a[i]<<")"<<"x^"<<i;
    // cout<<"\n";

    for (i=0;i<(n+1);i++)
    {
      av.push_back(a[i]);
    }
    return av;
  }


  bool image_process_srv(nist::cable_plane_color::Request &req, nist::cable_plane_color::Response &res)
  {
    pixel_x.clear();
    pixel_y.clear();

    aa.clear();
    av.clear();
    xx.clear();
    yy.clear();
    sparse_x.clear();
    sparse_y.clear();
    sparse_z.clear();

    if (req.region == 1){
      int pixel_v = 244;
      int pixel_h = 703;

      for (int i=pixel_h; i<(pixel_h+1279-pixel_h); i++) {
          for (int j=(pixel_v-pixel_v); j<pixel_v; j++){
            if (imgThresholded.at<bool>(j,i) != 0) { 
              pixel_x.push_back(i);
              pixel_y.push_back(j);
            }
          }
      }
      //std::cout << pixel_x.size() << std::endl;
      if (pixel_x.size() != 0){

        std::vector<tf::Vector3> cable_v;
        tf::Vector3 cable_point;
        int pixel_size = pixel_x.size();
        float depth;

        for (int i = 0; i < pixel_size; i++){
          cv::Point2d pixel_point(pixel_x[i], pixel_y[i]);      
          depth = depth_img_cv->image.at<short int>(pixel_point);
          //cout << "depth: " << depth << endl;
          if (depth > 0.2){
            cv::Point3d xyz = pin_hole_camera.projectPixelTo3dRay(pixel_point);
            cv::Point3d coordinate = xyz * depth;
            float co_x_temp = coordinate.x/1000;
            float co_y_temp = coordinate.y/1000;
            float co_z_temp = coordinate.z/1000;
            //cout << "xyz: " << co_x_temp << " ," << co_y_temp << " ," << co_z_temp << endl;
            cable_point.setX(co_x_temp);
            cable_point.setY(co_y_temp);
            cable_point.setZ(co_z_temp);
            cable_v.push_back(cable_point);       
          }
        }

        //curve fit
        N = cable_v.size(); // N is the no. of data pairs
        for(int i=0; i<N; i++)
        {
          xx.push_back(cable_v[i].getX());
          yy.push_back(cable_v[i].getY());
          zz.push_back(cable_v[i].getZ());
        }

        double sum_z, avg_z;
        for(int i=0; i<N; i++)
        {
          sum_z = sum_z + zz[i];
        }
        avg_z = sum_z / N;
        

        cout << avg_z << endl;


        // if (axis == "y"){
        //   aa = PolyfitXY(yy,xx);  //x = aa[0]*y^0 + aa[1]*y^1 + aa[2]*y^2
        // }

        aa = PolyfitXY(xx,yy);  //y = aa[0]*x^0 + aa[1]*x^1 + aa[2]*x^2
  
        
        
        // get the min and max
        double tip_temp , end_temp;
        tip_temp = *std::max_element(xx.begin(), xx.end());
        end_temp = *std::min_element(xx.begin(), xx.end());

        //std::cout << "tip_temp:" << tip_temp << std::endl;

        // sample the wire
        int num_sample;

        num_sample = floor(N/100);

        double segment = (double)(tip_temp-end_temp)/num_sample;

        for (double i = tip_temp; i > end_temp; i = i - segment) 
          {
            sparse_x.push_back(i);
            sparse_y.push_back(aa[0]+aa[1]*i+aa[2]*i*i);
            sparse_z.push_back(avg_z);
          }
        

        // curve fit done

        float msg_grasp_x, msg_grasp_y, msg_grasp_z, msg_grasp_theta, msg_length;
        float msg_tip_x, msg_tip_y, msg_tip_z, msg_tip_theta;
        int size = sparse_x.size();

        msg_grasp_x = sparse_x[floor(size/2)];
        msg_grasp_y = sparse_y[floor(size/2)];
        msg_grasp_z = sparse_z[floor(size/2)];

        msg_tip_x = sparse_x[0];
        msg_tip_y = sparse_y[0];
        msg_tip_z = sparse_z[0];

        msg_grasp_theta = atan((sparse_y[floor(size/2)]-sparse_y[floor(size/2)+1])/(sparse_x[floor(size/2)]-sparse_x[floor(size/2)+1]));
        msg_length = sqrt(pow((sparse_x[0]-sparse_x[size-1]),2)+pow((sparse_y[0]-sparse_y[size-1]),2));
        msg_tip_theta = atan((sparse_y[0]-sparse_y[1])/(sparse_x[0]-sparse_x[floor(1)]));

        cout << "grasp point: " << msg_grasp_x << ", " << msg_grasp_y << ", " << msg_grasp_z << endl;
        cout << "tip point: " << msg_tip_x << ", " << msg_tip_y << ", " << msg_tip_z << endl;
        cout << "size: " << size << endl;
        cout << "grasp theta: " << msg_grasp_theta << endl;
        cout << "tip theta: " << msg_tip_theta << endl;
        cout << "length: " << msg_length << endl;
      
        res.grasp_x = msg_grasp_x;
        res.grasp_y = msg_grasp_y;
        res.grasp_z = msg_grasp_z;
        res.grasp_theta = msg_grasp_theta;
        res.tip_x = msg_tip_x;
        res.tip_y = msg_tip_y;
        res.tip_z = msg_tip_z;
        res.tip_theta = msg_tip_theta;
        res.length = msg_length;
        res.result = true;
      }
      else{
        res.grasp_x = 0;
        res.grasp_y = 0;
        res.grasp_z = 0;
        res.grasp_theta = 0;
        res.tip_x = 0;
        res.tip_y = 0;
        res.tip_z = 0;
        res.tip_theta = 0;
        res.length = 0;
        res.result = true;
        cout << "grasp point: " << res.grasp_x << ", " << res.grasp_y << ", " << res.grasp_z << endl;
        cout << "tip point: " << res.tip_x << ", " << res.tip_y << ", " << res.tip_z << endl;
        cout << "size: " << 0 << endl;
        cout << "grasp theta: " << res.grasp_theta << endl;
        cout << "tip theta: " << res.tip_theta << endl;
        cout << "length: " << res.length << endl;
      }
    }

    if (req.region == 2) {
      int pixel_v = 364;
      int pixel_h = 600;

      for (int i=pixel_h; i<(pixel_h+1279-pixel_h); i++) {
          for (int j=pixel_v; j<(pixel_v+719-pixel_v); j++){
            if (imgThresholded.at<bool>(j,i) != 0) { 
              pixel_x.push_back(i);
              pixel_y.push_back(j);
            }
          }
        }
      //std::cout << pixel_x.size() << std::endl;

      if (pixel_x.size() != 0){
        std::vector<tf::Vector3> cable_v;
        tf::Vector3 cable_point;
        int pixel_size = pixel_x.size();
        float depth;

        for (int i = 0; i < pixel_size; i++){
          cv::Point2d pixel_point(pixel_x[i], pixel_y[i]);      
          depth = depth_img_cv->image.at<short int>(pixel_point);
          //cout << "depth: " << depth << endl;
          if (depth > 0.2){
            cv::Point3d xyz = pin_hole_camera.projectPixelTo3dRay(pixel_point);
            cv::Point3d coordinate = xyz * depth;
            float co_x_temp = coordinate.x/1000;
            float co_y_temp = coordinate.y/1000;
            float co_z_temp = coordinate.z/1000;
            //cout << "xyz: " << co_x_temp << " ," << co_y_temp << " ," << co_z_temp << endl;
            cable_point.setX(co_x_temp);
            cable_point.setY(co_y_temp);
            cable_point.setZ(co_z_temp);
            cable_v.push_back(cable_point);       
          }
        }

        //curve fit
        N = cable_v.size(); // N is the no. of data pairs
        for(int i=0; i<N; i++)
        {
          xx.push_back(cable_v[i].getX());
          yy.push_back(cable_v[i].getY());
          zz.push_back(cable_v[i].getZ());
        }

        double sum_z, avg_z;
        for(int i=0; i<N; i++)
        {
          sum_z = sum_z + zz[i];
        }
        avg_z = sum_z / N;
        

        cout << avg_z << endl;
   
        aa = PolyfitXY(yy,xx);  //x = aa[0]*y^0 + aa[1]*y^1 + aa[2]*y^2
       
        // get the min and max
        double tip_temp , end_temp;

        tip_temp = *std::max_element(yy.begin(), yy.end());
        end_temp = *std::min_element(yy.begin(), yy.end());


        

        //std::cout << "tip_temp:" << tip_temp << std::endl;

        // sample the wire
        int num_sample;

        num_sample = floor(N/100);

        double segment = (double)(tip_temp-end_temp)/num_sample;

        for (double i = tip_temp; i > end_temp; i = i - segment) 
        {
          sparse_y.push_back(i);
          sparse_x.push_back(aa[0]+aa[1]*i+aa[2]*i*i);
          sparse_z.push_back(avg_z);
        }
        // curve fit done

        float msg_grasp_x, msg_grasp_y, msg_grasp_z, msg_grasp_theta, msg_length;
        float msg_tip_x, msg_tip_y, msg_tip_z, msg_tip_theta;
        int size = sparse_x.size();

        // msg_grasp_x = sparse_x[floor(size/2)];
        // msg_grasp_y = sparse_y[floor(size/2)];
        // msg_grasp_z = sparse_z[floor(size/2)];

        // grasp tip for tube 3 insertion
        msg_grasp_x = sparse_x[1];
        msg_grasp_y = sparse_y[1];
        msg_grasp_z = sparse_z[1];


        msg_tip_x = sparse_x[0];
        msg_tip_y = sparse_y[0];
        msg_tip_z = sparse_z[0];

        msg_grasp_theta = -atan((sparse_x[floor(size/2)]-sparse_x[floor(size/2)+1])/(sparse_y[floor(size/2)]-sparse_y[floor(size/2)+1]));
        msg_length = sqrt(pow((sparse_x[0]-sparse_x[size-1]),2)+pow((sparse_y[0]-sparse_y[size-1]),2));
        msg_tip_theta = -atan((sparse_x[0]-sparse_x[1])/(sparse_y[0]-sparse_y[1]));

        cout << "grasp point: " << msg_grasp_x << ", " << msg_grasp_y << ", " << msg_grasp_z << endl;
        cout << "tip point: " << msg_tip_x << ", " << msg_tip_y << ", " << msg_tip_z << endl;
        cout << "size: " << size << endl;
        cout << "grasp theta: " << msg_grasp_theta << endl;
        cout << "tip theta: " << msg_tip_theta << endl;
        cout << "length: " << msg_length << endl;
      
        res.grasp_x = msg_grasp_x;
        res.grasp_y = msg_grasp_y;
        res.grasp_z = msg_grasp_z;
        res.grasp_theta = msg_grasp_theta;
        res.tip_x = msg_tip_x;
        res.tip_y = msg_tip_y;
        res.tip_z = msg_tip_z;
        res.tip_theta = msg_tip_theta;
        res.length = msg_length;
        res.result = true;
      }
      else{      
        res.grasp_x = 0;
        res.grasp_y = 0;
        res.grasp_z = 0;
        res.grasp_theta = 0;
        res.tip_x = 0;
        res.tip_y = 0;
        res.tip_z = 0;
        res.tip_theta = 0;
        res.length = 0;
        res.result = true;
        cout << "grasp point: " << res.grasp_x << ", " << res.grasp_y << ", " << res.grasp_z << endl;
        cout << "tip point: " << res.tip_x << ", " << res.tip_y << ", " << res.tip_z << endl;
        cout << "size: " << 0 << endl;
        cout << "grasp theta: " << res.grasp_theta << endl;
        cout << "tip theta: " << res.tip_theta << endl;
        cout << "length: " << res.length << endl;
      }
    }

    return res.result;
  }

  
 };


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  

  ros::NodeHandle n;
  ImageConverter ic;
  ros::ServiceServer srv = n.advertiseService("/nist/cable_plane_color", &ImageConverter::image_process_srv, &ic);
  ROS_INFO("Ready to get the cable info on the plane.");
  ros::spin();
  return 0;
}
