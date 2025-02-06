#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common_headers.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <string>
#include <vector>

#include "lx_camera/lx_camera.h"
#include "utils/json.hpp"

tf::Transform PoseToTf(const Eigen::Matrix4f &pose) {
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  Eigen::Matrix3f rotation_matrix = pose.topLeftCorner(3, 3);
  Eigen::Quaternionf quat(rotation_matrix);
  q.setW(quat.w());
  q.setX(quat.x());
  q.setY(quat.y());
  q.setZ(quat.z());
  transform.setRotation(q);
  return transform;
}

LxCamera::LxCamera() {
  nh_ = new ros::NodeHandle("~");
  ReadParam();

  // advertise
  image_transport::ImageTransport it(*nh_);
  pub_rgb_ = it.advertise("LxCamera_Rgb", 1);
  pub_amp_ = it.advertise("LxCamera_Amp", 1);
  pub_depth_ = it.advertise("LxCamera_Depth", 1);
  pub_error_ = nh_->advertise<std_msgs::String>("LxCamera_Error", 1);
  pub_pallet_ = nh_->advertise<lx_camera_ros::Pallet>("LxCamera_Pallet", 1);
  pub_tof_info_ =
      nh_->advertise<sensor_msgs::CameraInfo>("LxCamera_TofInfo", 1);
  pub_rgb_info_ =
      nh_->advertise<sensor_msgs::CameraInfo>("LxCamera_RgbInfo", 1);
  pub_temper_ =
      nh_->advertise<lx_camera_ros::FrameRate>("LxCamera_FrameRate", 1);
  pub_obstacle_ =
      nh_->advertise<lx_camera_ros::Obstacle>("LxCamera_Obstacle", 1);
  pub_cloud_ = nh_->advertise<sensor_msgs::PointCloud2>("LxCamera_Cloud", 1);

  ros::ServiceServer cmd =
      nh_->advertiseService("LxCamera_LxCmd", &LxCamera::LxCmd, this);
  ros::ServiceServer lxi =
      nh_->advertiseService("LxCamera_LxInt", &LxCamera::LxInt, this);
  ros::ServiceServer lxb =
      nh_->advertiseService("LxCamera_LxBool", &LxCamera::LxBool, this);
  ros::ServiceServer lxf =
      nh_->advertiseService("LxCamera_LxFloat", &LxCamera::LxFloat, this);
  ros::ServiceServer lxs =
      nh_->advertiseService("LxCamera_LxString", &LxCamera::LxString, this);

  // set sdk log
  ROS_INFO("Api version: %s", DcGetApiVersion());
  DcSetInfoOutput(1, true, log_path_.c_str());

  if (SearchAndOpenDevice()) {
    if (raw_param_) {
      SetParam();
    }

    Check("LX_INT_ALGORITHM_MODE",
          DcSetIntValue(handle_, LX_INT_ALGORITHM_MODE, inside_app_));
    Check("LX_BOOL_ENABLE_3D_DEPTH_STREAM",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_DEPTH_STREAM,
                         is_xyz_ || is_depth_));
    Check("LX_BOOL_ENABLE_3D_AMP_STREAM",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_AMP_STREAM, is_amp_));
    Check("LX_BOOL_ENABLE_2D_STREAM",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_STREAM, is_rgb_));
    Check("LX_INT_WORK_MODE",
          DcSetIntValue(handle_, LX_INT_WORK_MODE, lx_work_mode_));

    if (!is_start_) {
      Start();
    }
    Run();
  }
}

LxCamera::~LxCamera() {
  DcStopStream(handle_);
  DcCloseDevice(handle_);
}

bool LxCamera::SearchAndOpenDevice() {
  // find device
  int search_num = 5;
  int devnum = 0;
  LxDeviceInfo *devlist = nullptr;
  Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
  while (!devnum) {
    if (!search_num) {
      break;
    }
    Check("FIND_DEVICE", DcGetDeviceList(&devlist, &devnum));
    ROS_ERROR("Found device failed. retry...");
    usleep(1000 * 1000);
    search_num--;
  }

  // open device
  int is_open = -1;
  LxDeviceInfo info;
  if (ip_.empty())
    ip_ = "0";
  auto mode =
      ip_.size() < 8 ? LX_OPEN_MODE::OPEN_BY_INDEX : LX_OPEN_MODE::OPEN_BY_IP;
  if (LX_SUCCESS != DcOpenDevice(mode, ip_.c_str(), &handle_, &info)) {
    ROS_ERROR("Open device failed!");
    return false;
  }

  ROS_INFO("Open device success:"
           "\ndevice handle:             %lld"
           "\ndevice name:               %s"
           "\ndevice id:                 %s"
           "\ndevice ip:                 %s"
           "\ndevice sn:                 %s"
           "\ndevice mac:                %s"
           "\ndevice firmware version:   %s"
           "\ndevice algorithm version:  %s",
           handle_, info.name, info.id, info.ip, info.sn, info.mac,
           info.firmware_ver, info.algor_ver);
  return true;
}

int LxCamera::Start() {
  LxIntValueInfo int_value;
  DcGetIntValue(handle_, LX_INT_3D_IMAGE_WIDTH, &int_value);
  tof_camera_info_.width = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_3D_IMAGE_HEIGHT, &int_value);
  tof_camera_info_.height = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_WIDTH, &int_value);
  rgb_camera_info_.width = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_HEIGHT, &int_value);
  rgb_camera_info_.height = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_DATA_TYPE, &int_value);
  rgb_type_ = int_value.cur_value;
  DcGetIntValue(handle_, LX_INT_2D_IMAGE_CHANNEL, &int_value);
  rgb_channel_ = int_value.cur_value;

  DcGetBoolValue(handle_, LX_BOOL_ENABLE_3D_DEPTH_STREAM, (bool *)&is_depth_);
  DcGetBoolValue(handle_, LX_BOOL_ENABLE_3D_AMP_STREAM, (bool *)&is_amp_);
  DcGetBoolValue(handle_, LX_BOOL_ENABLE_2D_STREAM, (bool *)&is_rgb_);
  DcGetIntValue(handle_, LX_INT_ALGORITHM_MODE, &int_value);
  inside_app_ = int_value.cur_value;

  // get image params
  if (is_depth_ || is_amp_ || is_xyz_) {
    float *intr = nullptr;
    DcGetPtrValue(handle_, LX_PTR_3D_INTRIC_PARAM, (void **)&intr);
    tof_camera_info_.D =
        std::vector<double>{intr[4], intr[5], intr[6], intr[7], intr[8]};
    tof_camera_info_.K = boost::array<double, 9>{
        intr[0], 0, intr[2], 0, intr[1], intr[3], 0, 0, 1};
    tof_camera_info_.header.frame_id = "intrinsic_depth";
  }

  if (is_rgb_) {
    float *intr = nullptr;
    DcGetPtrValue(handle_, LX_PTR_2D_INTRIC_PARAM, (void **)&intr);
    rgb_camera_info_.D =
        std::vector<double>{intr[4], intr[5], intr[6], intr[7], intr[8]};
    rgb_camera_info_.K = boost::array<double, 9>{
        intr[0], 0, intr[2], 0, intr[1], intr[3], 0, 0, 1};
    rgb_camera_info_.header.frame_id = "intrinsic_rgb";
  }

  auto ret = DcStartStream(handle_);
  is_start_ = (ret == LX_SUCCESS);
  return static_cast<int>(ret);
}

int LxCamera::Stop() {
  auto ret = DcStopStream(handle_);
  if (is_start_ && ret == LX_SUCCESS) {
    is_start_ = false;
  }
  return static_cast<int>(ret);
}

void LxCamera::Run() {
  Eigen::Matrix3f R = (Eigen::AngleAxisf(install_yaw_ / 180.f * M_PI,
                                         Eigen::Vector3f::UnitZ()) *
                       Eigen::AngleAxisf(install_pitch_ / 180.f * M_PI,
                                         Eigen::Vector3f::UnitY()) *
                       Eigen::AngleAxisf(install_roll_ / 180.f * M_PI,
                                         Eigen::Vector3f::UnitX()))
                          .toRotationMatrix();
  Eigen::Vector3f t{install_x_, install_y_, install_z_};
  Eigen::Matrix4f ext_base_tof = Eigen::Matrix4f::Identity();
  ext_base_tof.block(0, 0, 3, 3) = R;
  ext_base_tof.block(0, 3, 3, 1) = t;
  tf::Transform tf_ext_base_tof = PoseToTf(ext_base_tof);
  ROS_INFO_STREAM("ext_base_tof:" << ext_base_tof);

  Eigen::Matrix4f ext_tof_rgb = Eigen::Matrix4f::Identity();
  tf::Transform tf_ext_tof_rgb;
  if ((is_xyz_ || is_depth_ || is_amp_) && is_rgb_) {
    Eigen::Matrix4f ext_rgb_tof = Eigen::Matrix4f::Identity();
    float *ext_param     = nullptr;
    DcGetPtrValue(handle_, LX_PTR_3D_EXTRIC_PARAM, (void **)&ext_param);
    ext_rgb_tof << ext_param[0], ext_param[1], ext_param[2],
        ext_param[9] * 0.001, ext_param[3], ext_param[4], ext_param[5],
        ext_param[10] * 0.001, ext_param[6], ext_param[7], ext_param[8],
        ext_param[11] * 0.001, 0, 0, 0, 1;
    ext_tof_rgb = ext_rgb_tof.inverse();
    tf_ext_tof_rgb = PoseToTf(ext_tof_rgb);
    ROS_INFO_STREAM("ext_rgb_tof:" << ext_rgb_tof);
  }

  ros::Rate loop_rate(20);
  while (ros::ok()) {
    FrameInfo *one_frame = nullptr;
    if (Check("LX_CMD_GET_NEW_FRAME", DcSetCmd(handle_, LX_CMD_GET_NEW_FRAME)))
      continue;
    if (Check("LX_PTR_FRAME_DATA",
              DcGetPtrValue(handle_, LX_PTR_FRAME_DATA, (void **)&one_frame)))
      continue;
    float dep_fps = 0.0, amp_fps = 0.0, rgb_fps = 0.0, temp = 0.0;
    auto now_time = ros::Time::now();
    LxFloatValueInfo f_val;

    if (is_xyz_) {
      float *xyz_data = nullptr;
      if (DcGetPtrValue(handle_, LX_PTR_XYZ_DATA, (void **)&xyz_data) ==
          LX_SUCCESS) {
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto buff_len = tof_camera_info_.width * tof_camera_info_.height;
        cloud->points.resize(buff_len);
        if (lx_tof_unit_) {
          for (long i = 0; i < buff_len; i++) {
            long index = 3 * i;
            cloud->points[i].x = xyz_data[index] / 1000.f;
            cloud->points[i].y = xyz_data[index + 1] / 1000.f;
            cloud->points[i].z = xyz_data[index + 2] / 1000.f;
          }
        } else
          memcpy(cloud->points.data(), xyz_data, buff_len * 3 * 4);
        cloud->width = tof_camera_info_.width;
        cloud->height = tof_camera_info_.height;
        pcl::toROSMsg(*cloud, msg_cloud);
        msg_cloud.header.stamp = ros::Time().fromSec(
            one_frame->depth_data.sensor_timestamp / 1000000.0);
        msg_cloud.header.frame_id = "mrdvs_tof";
        pub_cloud_.publish(msg_cloud);
      } else
        ROS_WARN("%s", std::string("Cloud point data is empty!").c_str());
    }

    if (is_depth_) {
      void *dep_data = one_frame->depth_data.frame_data;
      if (dep_data) {
        cv::Mat dep_img(one_frame->depth_data.frame_height,
                        one_frame->depth_data.frame_width, CV_16UC1, dep_data);
        auto msg_depth =
            cv_bridge::CvImage(std_msgs::Header(), "16UC1", dep_img)
                .toImageMsg();
        msg_depth->header.stamp = ros::Time().fromSec(
            one_frame->depth_data.sensor_timestamp / 1000000.0);
        msg_depth->header.frame_id = "mrdvs_tof";
        pub_depth_.publish(msg_depth);
      } else
        ROS_WARN("%s", std::string("Depth image is empty!").c_str());
      Check("LX_FLOAT_3D_DEPTH_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_3D_DEPTH_FPS, &f_val));
      dep_fps = f_val.cur_value;
    }

    if (is_amp_) {
      void *amp_data = one_frame->amp_data.frame_data;
      if (amp_data) {
        cv::Mat amp_img(one_frame->amp_data.frame_height,
                        one_frame->amp_data.frame_width, CV_16UC1, amp_data);
        auto msg_amp = cv_bridge::CvImage(std_msgs::Header(), "16UC1", amp_img)
                           .toImageMsg();
        msg_amp->header.stamp = ros::Time().fromSec(
            one_frame->amp_data.sensor_timestamp / 1000000.0);
        msg_amp->header.frame_id = "mrdvs_tof";
        pub_amp_.publish(msg_amp);
      } else
        ROS_WARN("%s", std::string("Amplitude image is empty!").c_str());
      Check("LX_FLOAT_3D_AMPLITUDE_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_3D_AMPLITUDE_FPS, &f_val));
      amp_fps = f_val.cur_value;
    }

    if (is_rgb_) {
      void *rgb_data = one_frame->rgb_data.frame_data;
      if (rgb_data) {
        cv::Mat rgb_pub;
        auto _type = CV_MAKETYPE(rgb_type_, rgb_channel_);
        cv::Mat rgb_img(one_frame->rgb_data.frame_height,
                        one_frame->rgb_data.frame_width, _type, rgb_data);
        rgb_img.convertTo(rgb_pub, CV_8UC1, rgb_type_ == CV_16U ? 0.25 : 1);

        std::string type = rgb_channel_ == 3 ? "bgr8" : "mono8";
        sensor_msgs::ImagePtr msg_rgb =
            cv_bridge::CvImage(std_msgs::Header(), type, rgb_pub).toImageMsg();
        msg_rgb->header.stamp = ros::Time().fromSec(
            one_frame->rgb_data.sensor_timestamp / 1000000.0);
        msg_rgb->header.frame_id = "mrdvs_rgb";
        pub_rgb_.publish(msg_rgb);
      } else
        ROS_WARN("%s", std::string("RGB image is empty!").c_str());
      Check("LX_FLOAT_2D_IMAGE_FPS",
            DcGetFloatValue(handle_, LX_FLOAT_2D_IMAGE_FPS, &f_val));
      rgb_fps = f_val.cur_value;
    }
    if (is_amp_ || is_depth_ || is_xyz_)
      pub_tof_info_.publish(tof_camera_info_);
    if (is_rgb_)
      pub_rgb_info_.publish(rgb_camera_info_);

    // pub rate and temperature
    Check("LX_FLOAT_DEVICE_TEMPERATURE",
          DcGetFloatValue(handle_, LX_FLOAT_DEVICE_TEMPERATURE, &f_val));
    temp = f_val.cur_value;
    lx_camera_ros::FrameRate fr;
    fr.header.frame_id = "mrdvs";
    fr.header.stamp = now_time;
    fr.amp = amp_fps;
    fr.rgb = rgb_fps;
    fr.depth = dep_fps;
    fr.temperature = temp;
    pub_temper_.publish(fr);

    // pub TF
    PubTf(tf_ext_base_tof, "base_link", "mrdvs_tof");
    if ((is_xyz_ || is_depth_ || is_amp_) && is_rgb_) {
      PubTf(tf_ext_tof_rgb, "mrdvs_tof", "mrdvs_rgb");
    }

    int ret = 0;
    void *app_ptr = one_frame->app_data.frame_data;
    switch (inside_app_) {
    case MODE_AVOID_OBSTACLE: {
      lx_camera_ros::Obstacle result;
      result.header.frame_id = "mrdvs";
      result.header.stamp =
          ros::Time().fromSec(one_frame->app_data.sensor_timestamp / 1000000.0);
      Check("GetObstacleIO", DcSpecialControl(handle_, "GetObstacleIO",
                                              (void *)&result.io_output));
      if (ret || !app_ptr) {
        result.status = -1;
        pub_obstacle_.publish(result);
        break;
      }
      LxAvoidanceOutput *lao = (LxAvoidanceOutput *)app_ptr;
      result.status = lao->state;
      result.box_number = lao->number_box;
      for (int i = 0; i < lao->number_box; i++) {
        auto raw_box = lao->obstacleBoxs[i];
        lx_camera_ros::ObstacleBox box;
        box.width = raw_box.width;
        box.depth = raw_box.depth;
        box.height = raw_box.height;
        for (int t = 0; t < 3; t++)
          box.center[t] = raw_box.center[t];
        for (int t = 0; t < 9; t++)
          box.rotation[t] = raw_box.pose.R[t];
        for (int t = 0; t < 3; t++)
          box.translation[t] = raw_box.pose.T[t];
        result.box.push_back(box);
      }
      pub_obstacle_.publish(result);
      break;
    }
    case MODE_PALLET_LOCATE: {
      if (ret || !app_ptr)
        break;
      lx_camera_ros::Pallet result;
      result.header.frame_id = "mrdvs";
      result.header.stamp =
          ros::Time().fromSec(one_frame->app_data.sensor_timestamp / 1000000.0);
      LxPalletPose *lao = (LxPalletPose *)app_ptr;
      result.status = lao->return_val;
      result.x = lao->x;
      result.y = lao->y;
      result.yaw = lao->yaw;
      pub_pallet_.publish(result);
      break;
    }
    case MODE_VISION_LOCATION: {
      if (ret || !app_ptr)
        break;
      LxLocation *val = (LxLocation *)app_ptr;
      if (!val->status) {
        geometry_msgs::PoseStamped alg_val;
        alg_val.header.frame_id = "mrdvs";
        alg_val.header.stamp = ros::Time().fromSec(
            one_frame->app_data.sensor_timestamp / 1000000.0);

        geometry_msgs::Quaternion q;
        q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, val->theta);
        alg_val.pose.position.x = val->x;
        alg_val.pose.position.y = val->y;
        alg_val.pose.position.z = 0;
        alg_val.pose.orientation.x = q.x;
        alg_val.pose.orientation.y = q.y;
        alg_val.pose.orientation.z = q.z;
        alg_val.pose.orientation.w = q.w;
        pub_location_.publish(alg_val);
      }
      break;
    }
    case MODE_AVOID_OBSTACLE2: {
      lx_camera_ros::Obstacle result;
      result.header.frame_id = "mrdvs";
      result.header.stamp =
          ros::Time().fromSec(one_frame->app_data.sensor_timestamp / 1000000.0);
      Check("GetObstacleIO", DcSpecialControl(handle_, "GetObstacleIO",
                                              (void *)&result.io_output));
      if (ret || !app_ptr) {
        result.status = -1;
        pub_obstacle_.publish(result);
        break;
      }
      LxAvoidanceOutputN *lao = (LxAvoidanceOutputN *)app_ptr;
      result.status = lao->state;
      result.box_number = lao->number_box;
      for (int i = 0; i < lao->number_box; i++) {
        auto raw_box = lao->obstacleBoxs[i];
        lx_camera_ros::ObstacleBox box;
        box.width = raw_box.width;
        box.depth = raw_box.depth;
        box.height = raw_box.height;
        for (int t = 0; t < 3; t++)
          box.center[t] = raw_box.center[t];
        for (int t = 0; t < 9; t++)
          box.rotation[t] = raw_box.pose.R[t];
        for (int t = 0; t < 3; t++)
          box.translation[t] = raw_box.pose.T[t];
        result.box.push_back(box);
      }
      pub_obstacle_.publish(result);
      break;
    }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int LxCamera::Check(std::string Command, int state) {
  LX_STATE lx_state = static_cast<LX_STATE>(state);
  if (LX_SUCCESS == lx_state)
    return state;

  const char *m = DcGetErrorString(lx_state); //获取错误信息
  std_msgs::String msg;
  setlocale(LC_ALL, "");
  msg.data = "#Command: " + Command +
             " #error code: " + std::to_string(lx_state) + " #report: " + m;
  pub_error_.publish(msg); //推送错误信息
  ROS_ERROR("%s", msg.data.c_str());
  return state;
}

void LxCamera::ReadParam() {
  nh_->param<std::string>("ip", ip_, "");
  nh_->param<std::string>("log_path", log_path_, "./");
  ROS_INFO("ip: %s", ip_.c_str());
  ROS_INFO("Log file path: %s", log_path_.c_str());
  nh_->param<int>("is_xyz", is_xyz_, 1);
  nh_->param<int>("is_depth", is_depth_, 0);
  nh_->param<int>("is_amp", is_amp_, 0);
  nh_->param<int>("is_rgb", is_rgb_, 0);
  nh_->param<int>("raw_param", raw_param_, 0);
  ROS_INFO("publish xyz: %d", is_xyz_);
  ROS_INFO("publish depth: %d", is_depth_);
  ROS_INFO("publish amp: %d", is_amp_);
  ROS_INFO("publish rgb: %d", is_rgb_);
  ROS_INFO("raw_param: %d", raw_param_);

  nh_->param<int>("lx_2d_binning", lx_2d_binning_, 0);
  nh_->param<int>("lx_2d_undistort", lx_2d_undistort_, 1);
  nh_->param<int>("lx_2d_undistort_scale", lx_2d_undistort_scale_, 0);
  nh_->param<int>("lx_2d_auto_exposure", lx_2d_auto_exposure_, 1);
  nh_->param<int>("lx_2d_auto_exposure_value", lx_2d_auto_exposure_value_, 50);
  nh_->param<int>("lx_2d_exposure", lx_2d_exposure_, 11000);
  nh_->param<int>("lx_2d_gain", lx_2d_gain_, 114);
  ROS_INFO("lx_2d_binning: %d", lx_2d_binning_);
  ROS_INFO("lx_2d_undistort: %d", lx_2d_undistort_);
  ROS_INFO("lx_2d_undistort_scale: %d", lx_2d_undistort_scale_);
  ROS_INFO("lx_2d_auto_exposure: %d", lx_2d_auto_exposure_);
  ROS_INFO("lx_2d_auto_exposure_value: %d", lx_2d_auto_exposure_value_);
  ROS_INFO("lx_2d_exposure: %d", lx_2d_exposure_);
  ROS_INFO("lx_2d_gain: %d", lx_2d_gain_);

  nh_->param<int>("lx_rgb_to_tof", lx_rgb_to_tof_, 0);
  nh_->param<int>("lx_3d_binning", lx_3d_binning_, 0);
  nh_->param<int>("lx_mulit_mode", lx_mulit_mode_, 0);
  nh_->param<int>("lx_3d_undistort", lx_3d_undistort_, 0);
  nh_->param<int>("lx_3d_undistort_scale", lx_3d_undistort_scale_, 0);
  nh_->param<int>("lx_hdr", lx_hdr_, 0);
  nh_->param<int>("lx_3d_auto_exposure", lx_3d_auto_exposure_, 1);
  nh_->param<int>("lx_3d_auto_exposure_value", lx_3d_auto_exposure_value_, 50);
  nh_->param<int>("lx_3d_first_exposure", lx_3d_first_exposure_, 1600);
  nh_->param<int>("lx_3d_second_exposure", lx_3d_second_exposure_, 500);
  nh_->param<int>("lx_3d_gain", lx_3d_gain_, 600);
  ROS_INFO("lx_rgb_to_tof: %d", lx_rgb_to_tof_);
  ROS_INFO("lx_3d_binning: %d", lx_3d_binning_);
  ROS_INFO("lx_mulit_mode: %d", lx_mulit_mode_);
  ROS_INFO("lx_3d_undistort: %d", lx_3d_undistort_);
  ROS_INFO("lx_3d_undistort_scale: %d", lx_3d_undistort_scale_);
  ROS_INFO("lx_hdr: %d", lx_hdr_);
  ROS_INFO("lx_3d_auto_exposure: %d", lx_3d_auto_exposure_);
  ROS_INFO("lx_3d_auto_exposure_value: %d", lx_3d_auto_exposure_value_);
  ROS_INFO("lx_3d_first_exposure: %d", lx_3d_first_exposure_);
  ROS_INFO("lx_3d_second_exposure: %d", lx_3d_second_exposure_);
  ROS_INFO("lx_3d_gain: %d", lx_3d_gain_);

  nh_->param<int>("lx_tof_unit", lx_tof_unit_, 0);
  nh_->param<int>("lx_min_depth", lx_min_depth_, 0);
  nh_->param<int>("lx_max_depth", lx_max_depth_, 3000);
  nh_->param<int>("lx_work_mode", lx_work_mode_, 0);
  nh_->param<int>("lx_application", inside_app_, 0);
  ROS_INFO("lx_tof_unit: %d", lx_tof_unit_);
  ROS_INFO("lx_min_depth: %d", lx_min_depth_);
  ROS_INFO("lx_max_depth: %d", lx_max_depth_);
  ROS_INFO("lx_work_mode: %d", lx_work_mode_);
  ROS_INFO("lx_application mode: %d", inside_app_);

  nh_->param<float>("x", install_x_, 0.0);
  nh_->param<float>("y", install_y_, 0);
  nh_->param<float>("z", install_z_, 0);
  nh_->param<float>("roll", install_roll_, 0);
  nh_->param<float>("pitch", install_pitch_, 0);
  nh_->param<float>("yaw", install_yaw_, 0);
  ROS_INFO("x: %f", install_x_);
  ROS_INFO("y: %f", install_y_);
  ROS_INFO("z: %f", install_z_);
  ROS_INFO("yaw: %f", install_yaw_);
  ROS_INFO("roll: %f", install_roll_);
  ROS_INFO("pitch: %f", install_pitch_);
}

void LxCamera::SetParam() {
  ROS_WARN("SetParam now!");
  Check("LX_INT_WORK_MODE", DcSetIntValue(handle_, LX_INT_WORK_MODE, 0));
  Check("LX_BOOL_ENABLE_2D_TO_DEPTH",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_TO_DEPTH, 0));
  Check("LX_INT_2D_BINNING_MODE",
        DcSetIntValue(handle_, LX_INT_2D_BINNING_MODE, lx_2d_binning_));
  Check("LX_BOOL_ENABLE_2D_UNDISTORT",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_UNDISTORT, lx_2d_undistort_));
  Check("LX_INT_2D_UNDISTORT_SCALE",
        DcSetIntValue(handle_, LX_INT_2D_UNDISTORT_SCALE,
                      lx_2d_undistort_scale_));
  Check("LX_BOOL_ENABLE_2D_AUTO_EXPOSURE",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_AUTO_EXPOSURE,
                       lx_2d_auto_exposure_));
  if (!lx_2d_auto_exposure_) {
    Check("LX_INT_2D_MANUAL_EXPOSURE",
          DcSetIntValue(handle_, LX_INT_2D_MANUAL_EXPOSURE, lx_2d_exposure_));
    Check("LX_INT_2D_MANUAL_GAIN",
          DcSetIntValue(handle_, LX_INT_2D_MANUAL_GAIN, lx_2d_gain_));
  } else
    Check("LX_INT_2D_AUTO_EXPOSURE_LEVEL",
          DcSetIntValue(handle_, LX_INT_2D_AUTO_EXPOSURE_LEVEL,
                        lx_2d_auto_exposure_value_));

  Check("LX_BOOL_ENABLE_2D_TO_DEPTH",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_2D_TO_DEPTH, lx_rgb_to_tof_));
  Check("LX_INT_3D_BINNING_MODE",
        DcSetIntValue(handle_, LX_INT_3D_BINNING_MODE, lx_3d_binning_));
  Check("LX_BOOL_ENABLE_MULTI_MACHINE",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_MULTI_MACHINE, lx_mulit_mode_));
  Check("LX_BOOL_ENABLE_3D_UNDISTORT",
        DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_UNDISTORT, lx_3d_undistort_));
  Check("LX_INT_3D_UNDISTORT_SCALE",
        DcSetIntValue(handle_, LX_INT_3D_UNDISTORT_SCALE,
                      lx_3d_undistort_scale_));
  if (!lx_3d_auto_exposure_) {
    Check("LX_BOOL_ENABLE_3D_AUTO_EXPOSURE",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE,
                         lx_3d_auto_exposure_));
    Check("LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR, lx_hdr_));
    Check("LX_INT_FIRST_EXPOSURE",
          DcSetIntValue(handle_, LX_INT_FIRST_EXPOSURE, lx_3d_first_exposure_));
    Check(
        "LX_INT_SECOND_EXPOSURE",
        DcSetIntValue(handle_, LX_INT_SECOND_EXPOSURE, lx_3d_second_exposure_));
    Check("LX_INT_GAIN", DcSetIntValue(handle_, LX_INT_GAIN, lx_3d_gain_));
  } else {
    Check("LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_MULTI_EXPOSURE_HDR, 0));
    Check("LX_BOOL_ENABLE_3D_AUTO_EXPOSURE",
          DcSetBoolValue(handle_, LX_BOOL_ENABLE_3D_AUTO_EXPOSURE,
                         lx_3d_auto_exposure_));
    Check("LX_INT_3D_AUTO_EXPOSURE_LEVEL",
          DcSetIntValue(handle_, LX_INT_3D_AUTO_EXPOSURE_LEVEL,
                        lx_3d_auto_exposure_value_));
  }

  Check("LX_INT_MIN_DEPTH",
        DcSetIntValue(handle_, LX_INT_MIN_DEPTH, lx_min_depth_));
  Check("LX_INT_MAX_DEPTH",
        DcSetIntValue(handle_, LX_INT_MAX_DEPTH, lx_max_depth_));
  ROS_WARN("SetParam done!");
}

bool LxCamera::LxString(lx_camera_ros::LxString::Request &req,
                        lx_camera_ros::LxString::Response &res) {
  if (req.is_set)
    res.result.ret = DcSetStringValue(handle_, req.cmd, req.val.c_str());

  char *buf = nullptr;
  if (!req.is_set)
    res.result.ret = DcGetStringValue(handle_, req.cmd, &buf);
  res.result.msg = DcGetErrorString((LX_STATE)res.result.ret);
  if (buf)
    res.val = std::string(buf);
  return true;
}

bool LxCamera::LxFloat(lx_camera_ros::LxFloat::Request &req,
                       lx_camera_ros::LxFloat::Response &res) {
  if (req.is_set)
    res.result.ret = DcSetFloatValue(handle_, req.cmd, req.val);

  LxFloatValueInfo float_value;
  auto ret = DcGetFloatValue(handle_, req.cmd, &float_value);
  if (!req.is_set)
    res.result.ret = ret;
  res.cur_value = float_value.cur_value;
  res.max_value = float_value.max_value;
  res.min_value = float_value.min_value;
  res.available = float_value.set_available;
  res.result.msg = DcGetErrorString((LX_STATE)res.result.ret);
  return true;
}

bool LxCamera::LxBool(lx_camera_ros::LxBool::Request &req,
                      lx_camera_ros::LxBool::Response &res) {
  if (req.is_set)
    res.result.ret = DcSetBoolValue(handle_, req.cmd, req.val);

  bool val;
  auto ret = DcGetBoolValue(handle_, req.cmd, &val);
  res.val = val;
  if (!req.is_set)
    res.result.ret = ret;
  res.result.msg = DcGetErrorString((LX_STATE)res.result.ret);
  return true;
}

bool LxCamera::LxCmd(lx_camera_ros::LxCmd::Request &req,
                     lx_camera_ros::LxCmd::Response &res) {
  auto Pub = [&](std::string msg, int ret) {
    lx_camera_ros::Result result;
    result.ret = ret;
    result.msg = msg;
    res.result.push_back(result);
  };

  if (req.cmd == 1) {
    auto ret = static_cast<LX_STATE>(Start());
    Pub(DcGetErrorString(ret), ret);
  } else if (req.cmd == 2) {
    auto ret = static_cast<LX_STATE>(Stop());
    Pub(DcGetErrorString(ret), ret);
  } else if (req.cmd) {
    auto ret = static_cast<LX_STATE>(DcSetCmd(handle_, req.cmd));
    Pub(DcGetErrorString(ret), ret);
  } else {
    std::vector<std::pair<std::string, int>> cmd_vec;
    auto add = [&](std::string str, int cmd) {
      std::pair<std::string, int> val(str, cmd);
      cmd_vec.push_back(val);
    };
    add("INT      FIRST_EXPOSURE", 1001);
    add("INT      SECOND_EXPOSURE", 1002);
    add("INT      THIRD_EXPOSURE", 1003);
    add("INT      FOURTH_EXPOSURE", 1004);
    add("INT      GAIN", 1005);
    add("INT      MIN_DEPTH", 1011);
    add("INT      MAX_DEPTH", 1012);
    add("INT      MIN_AMPLITUDE", 1013);
    add("INT      MAX_AMPLITUDE", 1014);
    add("INT      CODE_MODE", 1016);
    add("INT      WORK_MODE", 1018);
    add("INT      LINK_SPEED", 1019);
    add("INT      3D_IMAGE_WIDTH", 1021);
    add("INT      3D_IMAGE_HEIGHT", 1022);
    add("INT      3D_IMAGE_OFFSET_X", 1023);
    add("INT      3D_IMAGE_OFFSET_Y", 1024);
    add("INT      3D_BINNING_MODE", 1025);
    add("INT      3D_DEPTH_DATA_TYPE", 1026);
    add("INT      3D_AMPLITUDE_CHANNEL", 1031);
    add("INT      3D_AMPLITUDE_GET_TYPE", 1032);
    add("INT      3D_AMPLITUDE_EXPOSURE", 1033);
    add("INT      3D_AMPLITUDE_INTENSITY", 1034);
    add("INT      3D_AMPLITUDE_DATA_TYPE", 1035);
    add("INT      3D_AUTO_EXPOSURE_LEVEL", 1036);
    add("INT      3D_AUTO_EXPOSURE_MAX", 1037);
    add("INT      3D_AUTO_EXPOSURE_MIN", 1038);
    add("INT      2D_IMAGE_WIDTH", 1041);
    add("INT      2D_IMAGE_HEIGHT", 1042);
    add("INT      2D_IMAGE_OFFSET_X", 1043);
    add("INT      2D_IMAGE_OFFSET_Y", 1044);
    add("INT      2D_BINNING_MODE", 1045);
    add("INT      2D_IMAGE_CHANNEL", 1046);
    add("INT      2D_IMAGE_DATA_TYPE", 1047);
    add("INT      2D_MANUAL_EXPOSURE", 1051);
    add("INT      2D_MANUAL_GAIN", 1052);
    add("INT      2D_ENCODE_TYPE", 1053);
    add("INT      2D_AUTO_EXPOSURE_LEVEL", 1054);
    add("INT      TOF_GLOBAL_OFFSET", 1061);
    add("INT      3D_UNDISTORT_SCALE", 1062);
    add("INT      ALGORITHM_MODE", 1065);
    add("INT      MODBUS_ADDR", 1066);
    add("INT      HEART_TIME", 1067);
    add("INT      GVSP_PACKET_SIZE", 1068);
    add("INT      TRIGGER_MODE", 1069);
    add("INT      CALCULATE_UP", 1070);
    add("INT      CAN_BAUD_RATE", 1072);
    add("INT      CUSTOM_PARAM_GROUP", 1075);
    add("FLOAT    FILTER_LEVEL", 2001);
    add("FLOAT    EST_OUT_EXPOSURE", 2002);
    add("FLOAT    LIGHT_INTENSITY", 2003);
    add("FLOAT    3D_DEPTH_FPS", 2004);
    add("FLOAT    3D_AMPLITUDE_FPS", 2005);
    add("FLOAT    2D_IMAGE_FPS", 2006);
    add("FLOAT    DEVICE_TEMPERATURE", 2007);
    add("BOOL     CONNECT_STATE", 3001);
    add("BOOL     ENABLE_3D_DEPTH_STREAM", 3002);
    add("BOOL     ENABLE_3D_AMP_STREAM", 3003);
    add("BOOL     ENABLE_3D_AUTO_EXPOSURE", 3006);
    add("BOOL     ENABLE_3D_UNDISTORT", 3007);
    add("BOOL     ENABLE_ANTI_FLICKER", 3008);
    add("BOOL     ENABLE_2D_STREAM", 3011);
    add("BOOL     ENABLE_2D_AUTO_EXPOSURE", 3012);
    add("BOOL     ENABLE_2D_UNDISTORT", 3015);
    add("BOOL     ENABLE_2D_TO_DEPTH", 3016);
    add("BOOL     ENABLE_BACKGROUND_AMP", 3017);
    add("BOOL     ENABLE_MULTI_MACHINE", 3018);
    add("BOOL     ENABLE_MULTI_EXPOSURE_HDR", 3019);
    add("BOOL     ENABLE_SYNC_FRAME", 3020);
    add("std::string   DEVICE_VERSION", 4001);
    add("std::string   DEVICE_LOG_NAME", 4002);
    add("std::string   FIRMWARE_NAME", 4003);
    add("std::string   FILTER_PARAMS", 4004);
    add("std::string   ALGORITHM_PARAMS", 4005);
    add("std::string   ALGORITHM_VERSION", 4006);
    add("std::string   DEVICE_OS_VERSION", 4007);
    add("CMD      GET_PARAM_LIST", 0);
    add("CMD      START_STREAM", 1);
    add("CMD      STOP_STREAM", 2);
    add("CMD      GET_NEW_FRAME", 5001);
    add("CMD      RETURN_VERSION", 5002);
    add("CMD      RESTART_DEVICE", 5003);
    add("CMD      WHITE_BALANCE", 5004);
    add("CMD      RESET_PARAM", 5007);
    add("CMD      CALIB_EXTRIC", 5008);

    for (auto &i : cmd_vec)
      while (i.first.length() < 40)
        i.first.push_back(' ');
    for (auto &i : cmd_vec)
      Pub(i.first, i.second);
  }

  return true;
}

bool LxCamera::LxInt(lx_camera_ros::LxInt::Request &req,
                     lx_camera_ros::LxInt::Response &res) {
  if (req.is_set)
    res.result.ret = DcSetIntValue(handle_, req.cmd, req.val);

  LxIntValueInfo int_value;
  auto ret = DcGetIntValue(handle_, req.cmd, &int_value);
  if (!req.is_set)
    res.result.ret = ret;
  res.cur_value = int_value.cur_value;
  res.max_value = int_value.max_value;
  res.min_value = int_value.min_value;
  res.available = int_value.set_available;
  res.result.msg = DcGetErrorString((LX_STATE)res.result.ret);
  return true;
}

void LxCamera::PubTf(const tf::Transform &transform, const std::string &frame_id, const std::string &child_frame_id) {
  static tf::TransformBroadcaster br;
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}
