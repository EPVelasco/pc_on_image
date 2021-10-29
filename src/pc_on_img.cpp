#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/filters/filter.h>
#include <opencv2/core/core.hpp>
#include <math.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pcOnimg_pub;
ros::Publisher pub;

float maxlen =10;
float minlen = 0.1;
float max_FOV = 1.6;
float min_FOV = 0.9;
std::string imgTopic = "/velodyne_points";
std::string pcTopic = "/camera/color/image_raw";

Eigen::MatrixXf Tlc(3,1); // translation matrix lidar-camera
Eigen::MatrixXf Rlc(3,3); // rotation matrix lidar-camera
Eigen::MatrixXf Mc(3,4);  // camera calibration matrix

void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& in_pc2 , const ImageConstPtr& in_image)
{


    cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

  //Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*in_pc2,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr msg_pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*msg_pointCloud);
  ///

  ////// filter point cloud
  if (msg_pointCloud == NULL) return;

  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);

  cloud_out->header.frame_id = "velodyne";
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg_pointCloud, *cloud_in, indices);

  for (int i = 0; i < (int) cloud_in->points.size(); i++)
  {
      double distance = sqrt(cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y);
      if(distance<minlen || distance>maxlen || cloud_in->points[i].x<0)
          continue;
      float ang = ( atan (cloud_in->points[i].x/ cloud_in->points[i].y));
      if(cloud_in->points[i].y<0)
        ang = M_PI+ ang;
      if (ang<min_FOV || ang> max_FOV)
          continue;
      cloud_out->push_back(cloud_in->points[i]);
  }

  Eigen::MatrixXf RTlc(4,4); // translation matrix lidar-camera
  RTlc<<   Rlc(0), Rlc(3) , Rlc(6) ,Tlc(0)
          ,Rlc(1), Rlc(4) , Rlc(7) ,Tlc(1)
          ,Rlc(2), Rlc(5) , Rlc(8) ,Tlc(2)
          ,0       , 0        , 0  , 1    ;

  int sizeLidar = (int) cloud_out->points.size();
  Eigen::MatrixXf Lidar_camera(3,sizeLidar);
  Eigen::MatrixXf pointCloud_matrix(4,sizeLidar);

  for (int i = 0; i < sizeLidar; i++)
  {
      pointCloud_matrix(0,i) = -cloud_out->points[i].y;
      pointCloud_matrix(1,i) = -cloud_out->points[i].z;
      pointCloud_matrix(2,i) = cloud_out->points[i].x;
      pointCloud_matrix(3,i) = 1.0;
  }

  Lidar_camera = Mc * (RTlc * pointCloud_matrix);

  int px_var = 0;
  int py_var = 0;
  unsigned int cols = in_image->width;
  unsigned int rows = in_image->height;

  for (int i=0;i<sizeLidar;i++)

  {
      px_var = (int)(Lidar_camera(0,i)/Lidar_camera(2,i));
      py_var = (int)(Lidar_camera(1,i)/Lidar_camera(2,i));

      if(px_var<0.0 || px_var>cols || py_var<0.0 || py_var>rows)
          continue;
      int color_dis_x = (int)(255*((cloud_out->points[i].x)/maxlen));
      int color_dis_z = (int)(255*((cloud_out->points[i].x)/20.0));
      if(color_dis_z>255)
          color_dis_z = 255;

      cv::circle(cv_ptr->image, cv::Point(px_var, py_var), 5, CV_RGB(255-color_dis_x,(int)(color_dis_z),color_dis_x),cv::FILLED);
  }

   pcOnimg_pub.publish(cv_ptr->toImageMsg());
   pcl_conversions::toPCL(ros::Time::now(), cloud_out->header.stamp);
   pub.publish (cloud_out);

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "pontCloudOntImage");
  ros::NodeHandle nh;  

  /// Load Parameters

  nh.getParam("/maxlen", maxlen);
  nh.getParam("/minlen", minlen);
  nh.getParam("/max_ang_FOV", max_FOV);
  nh.getParam("/min_ang_FOV", min_FOV);
  nh.getParam("/pcTopic", pcTopic);
  nh.getParam("/imgTopic", imgTopic);

  XmlRpc::XmlRpcValue param;

  nh.getParam("/matrix_file/tlc", param);
  Tlc <<  (double)param[0]
         ,(double)param[1]
         ,(double)param[2];

  nh.getParam("/matrix_file/rlc", param);


  Rlc <<  (double)param[0] ,(double)param[1] ,(double)param[2]
         ,(double)param[3] ,(double)param[4] ,(double)param[5]
         ,(double)param[6] ,(double)param[7] ,(double)param[8];

  nh.getParam("/matrix_file/camera_matrix", param);

  Mc  <<  (double)param[0] ,(double)param[1] ,(double)param[2] ,(double)param[3]
         ,(double)param[4] ,(double)param[5] ,(double)param[6] ,(double)param[7]
         ,(double)param[8] ,(double)param[9] ,(double)param[10],(double)param[11];

  message_filters::Subscriber<PointCloud2> pc_sub(nh, pcTopic , 1);
  message_filters::Subscriber<Image> img_sub(nh, imgTopic, 1);

  typedef sync_policies::ApproximateTime<PointCloud2, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, img_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  pcOnimg_pub = nh.advertise<sensor_msgs::Image>("/pcOnImage_image", 1);

  pub = nh.advertise<PointCloud> ("/points2", 1);

  ros::spin();
  //return 0;
}
