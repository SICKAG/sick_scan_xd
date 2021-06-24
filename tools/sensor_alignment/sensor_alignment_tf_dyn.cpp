#include <ros/ros.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <dynamic_reconfigure/server.h>
#include <sick_scan/tf_dynConfig.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

Eigen::Affine3d child_2_parent;

class ConfigTrans
{
public:
    std::string parentFrame;
    std::string childFrame;
    double trans_x;
    double trans_y;
    double trans_z;
    double trans_rot_x;
    double trans_rot_y;
    double trans_rot_z;
};

ConfigTrans cfgTrans;


// for setting region of interest
Eigen::Vector4f min_point;
Eigen::Vector4f max_point;

tf2_ros::Buffer tf_buffer;


void callback(sick_scan_fusion::tf_dynConfig &config, uint32_t level)
{
    cfgTrans.parentFrame = config.parent_frame;
    cfgTrans.childFrame = config.child_frame;

    cfgTrans.trans_x = config.tf_x;
    cfgTrans.trans_y = config.tf_y;
    cfgTrans.trans_z = config.tf_z;
    cfgTrans.trans_rot_x = config.tf_rot_x * M_PI / 180.0;
    cfgTrans.trans_rot_y = config.tf_rot_y * M_PI / 180.0;
    cfgTrans.trans_rot_z = config.tf_rot_z * M_PI / 180.0;
  }

int main (int argc, char** argv)
{
    // initialize
    ros::init (argc, argv, "sick_scan_fusion_tf_dyn");
    ros::NodeHandle nh;

    tf2_ros::TransformListener tf_listener(tf_buffer);


    child_2_parent = Eigen::Affine3d::Identity();
   
    // dynamic reconfigure
    dynamic_reconfigure::Server<sick_scan_fusion::tf_dynConfig> server;
    dynamic_reconfigure::Server<sick_scan_fusion::tf_dynConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);


    unsigned seq= 0;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if ((seq % 20) == 0)
        {
        printf("Par. %s Child: %s Trans x: %0.3lf\n", cfgTrans.childFrame.c_str(), cfgTrans.parentFrame.c_str(), cfgTrans.trans_x);
        }static tf2_ros::TransformBroadcaster br;
        child_2_parent = Eigen::Affine3d::Identity();
        child_2_parent.translate(Eigen::Vector3d( cfgTrans.trans_x,0,0));
        child_2_parent.translate(Eigen::Vector3d(0,  cfgTrans.trans_y,0));
        child_2_parent.translate(Eigen::Vector3d(0, 0, cfgTrans.trans_z));
        // pitch
        child_2_parent.rotate(Eigen::AngleAxisd(cfgTrans.trans_rot_x, Eigen::Vector3d(1, 0, 0)));
        // yaw
        child_2_parent.rotate(Eigen::AngleAxisd(cfgTrans.trans_rot_y, Eigen::Vector3d(0, 1, 0)));
        // yaw
        child_2_parent.rotate(Eigen::AngleAxisd(cfgTrans.trans_rot_z, Eigen::Vector3d(0, 0, 1)));

        geometry_msgs::TransformStamped transform_base_trans = tf2::eigenToTransform(child_2_parent);
        transform_base_trans.header.frame_id = cfgTrans.parentFrame;
        transform_base_trans.child_frame_id = cfgTrans.childFrame;
        transform_base_trans.header.stamp = ros::Time::now();
        transform_base_trans.header.seq = seq;
        br.sendTransform(transform_base_trans);

        ros::spinOnce();
        loop_rate.sleep();
        ++seq;
    }
}

