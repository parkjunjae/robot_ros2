#include "parameters.h"

bool odom_only;
std::string odom_header_frame_id, odom_child_frame_id;

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;

std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
bool use_imu_as_input, space_down_sample, publish_odometry_without_downsample;
int init_map_size, con_frame_num;
double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
float plane_thr;
double filter_size_surf_min, filter_size_map_min, fov_deg;
double cube_len;
float DET_RANGE;
bool imu_en, gravity_align, non_station_start;
double imu_time_inte;
double laser_point_cov, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov;
int lidar_type, pcd_save_interval;
std::vector<double> gravity_init, gravity;
std::vector<double> extrinT;
std::vector<double> extrinR;
bool runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
double time_lag_imu_to_lidar = 0.0;

void readParameters(shared_ptr<rclcpp::Node> &nh)
{
    p_pre.reset(new Preprocess());

    nh->declare_parameter<bool>("odom_only", false);
    nh->declare_parameter<std::string>("odom_header_frame_id", "map");
    nh->declare_parameter<std::string>("odom_child_frame_id", "odom");

    nh->declare_parameter<bool>("prop_at_freq_of_imu", true);
    nh->declare_parameter<bool>("use_imu_as_input", true);
    nh->declare_parameter<bool>("check_satu", true);
    nh->declare_parameter<int>("init_map_size", 100);
    nh->declare_parameter<bool>("space_down_sample", true);
    nh->declare_parameter<double>("mapping.satu_acc", 3.0);
    nh->declare_parameter<double>("mapping.satu_gyro", 35.0);
    nh->declare_parameter<double>("mapping.acc_norm", 1.0);
    nh->declare_parameter<float>("mapping.plane_thr", 0.05f);
    nh->declare_parameter<int>("point_filter_num", 2);
    nh->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    nh->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    nh->declare_parameter<bool>("common.con_frame", false);
    nh->declare_parameter<int>("common.con_frame_num", 1);
    nh->declare_parameter<bool>("common.cut_frame", false);
    nh->declare_parameter<double>("common.cut_frame_time_interval", 0.1);
    nh->declare_parameter<double>("common.time_lag_imu_to_lidar", 0.0);
    nh->declare_parameter<double>("filter_size_surf", 0.5);
    nh->declare_parameter<double>("filter_size_map", 0.5);
    nh->declare_parameter<double>("cube_side_length", 200);
    nh->declare_parameter<float>("mapping.det_range", 300.f);
    nh->declare_parameter<double>("mapping.fov_degree", 180);
    nh->declare_parameter<bool>("mapping.imu_en", true);
    nh->declare_parameter<bool>("mapping.start_in_aggressive_motion", false);
    nh->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    nh->declare_parameter<double>("mapping.imu_time_inte", 0.005);
    nh->declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
    nh->declare_parameter<double>("mapping.acc_cov_input", 0.1);
    nh->declare_parameter<double>("mapping.vel_cov", 20);
    nh->declare_parameter<double>("mapping.gyr_cov_input", 0.1);
    nh->declare_parameter<double>("mapping.gyr_cov_output", 0.1);
    nh->declare_parameter<double>("mapping.acc_cov_output", 0.1);
    nh->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    nh->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    nh->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
    nh->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
    nh->declare_parameter<double>("preprocess.blind", 1.0);
    nh->declare_parameter<int>("preprocess.lidar_type", 1);
    nh->declare_parameter<int>("preprocess.scan_line", 16);
    nh->declare_parameter<int>("preprocess.scan_rate", 10);
    nh->declare_parameter<int>("preprocess.timestamp_unit", 1);
    nh->declare_parameter<double>("mapping.match_s", 81);
    nh->declare_parameter<bool>("mapping.gravity_align", true);
    nh->declare_parameter<std::vector<double>>("mapping.gravity", {0, 0, -9.810});
    nh->declare_parameter<std::vector<double>>("mapping.gravity_init", {0, 0, -9.810});
    nh->declare_parameter<std::vector<double>>("mapping.extrinsic_T", {0, 0, 0});
    nh->declare_parameter<std::vector<double>>("mapping.extrinsic_R", {1, 0, 0, 0, 1, 0, 0, 0, 1});
    nh->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
    nh->declare_parameter<bool>("publish.path_en", true);
    nh->declare_parameter<bool>("publish.scan_publish_en", true);
    nh->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    nh->declare_parameter<bool>("runtime_pos_log_enable", false);
    nh->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    nh->declare_parameter<int>("pcd_save.interval", -1);

    // 使用get_parameter方法获取参数值
    nh->get_parameter("odom_only", odom_only);
    nh->get_parameter("odom_header_frame_id", odom_header_frame_id);
    nh->get_parameter("odom_child_frame_id", odom_child_frame_id);

    nh->get_parameter("prop_at_freq_of_imu", prop_at_freq_of_imu);
    nh->get_parameter("use_imu_as_input", use_imu_as_input);
    nh->get_parameter("check_satu", check_satu);
    nh->get_parameter("init_map_size", init_map_size);
    nh->get_parameter("space_down_sample", space_down_sample);
    nh->get_parameter("mapping.satu_acc", satu_acc);
    nh->get_parameter("mapping.satu_gyro", satu_gyro);
    nh->get_parameter("mapping.acc_norm", acc_norm);
    nh->get_parameter("mapping.plane_thr", plane_thr);
    nh->get_parameter("point_filter_num", p_pre->point_filter_num);
    nh->get_parameter("common.lid_topic", lid_topic);
    nh->get_parameter("common.imu_topic", imu_topic);
    nh->get_parameter("common.con_frame", con_frame);
    nh->get_parameter("common.con_frame_num", con_frame_num);
    nh->get_parameter("common.cut_frame", cut_frame);
    nh->get_parameter("common.cut_frame_time_interval", cut_frame_time_interval);
    nh->get_parameter("common.time_lag_imu_to_lidar", time_lag_imu_to_lidar);
    nh->get_parameter("filter_size_surf", filter_size_surf_min);
    nh->get_parameter("filter_size_map", filter_size_map_min);
    nh->get_parameter("cube_side_length", cube_len);
    nh->get_parameter("mapping.det_range", DET_RANGE);
    nh->get_parameter("mapping.fov_degree", fov_deg);
    nh->get_parameter("mapping.imu_en", imu_en);
    nh->get_parameter("mapping.start_in_aggressive_motion", non_station_start);
    nh->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);
    nh->get_parameter("mapping.imu_time_inte", imu_time_inte);
    nh->get_parameter("mapping.lidar_meas_cov", laser_point_cov);
    nh->get_parameter("mapping.acc_cov_input", acc_cov_input);
    nh->get_parameter("mapping.vel_cov", vel_cov);
    nh->get_parameter("mapping.gyr_cov_input", gyr_cov_input);
    nh->get_parameter("mapping.gyr_cov_output", gyr_cov_output);
    nh->get_parameter("mapping.acc_cov_output", acc_cov_output);
    nh->get_parameter("mapping.b_gyr_cov", b_gyr_cov);
    nh->get_parameter("mapping.b_acc_cov", b_acc_cov);
    nh->get_parameter("mapping.imu_meas_acc_cov", imu_meas_acc_cov);
    nh->get_parameter("mapping.imu_meas_omg_cov", imu_meas_omg_cov);
    nh->get_parameter("preprocess.blind", p_pre->blind);
    nh->get_parameter("preprocess.lidar_type", lidar_type);
    nh->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
    nh->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
    nh->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);
    nh->get_parameter("mapping.match_s", match_s);
    nh->get_parameter("mapping.gravity_align", gravity_align);
    nh->get_parameter("mapping.gravity", gravity);
    nh->get_parameter("mapping.gravity_init", gravity_init);
    nh->get_parameter("mapping.extrinsic_T", extrinT);
    nh->get_parameter("mapping.extrinsic_R", extrinR);
    nh->get_parameter("odometry.publish_odometry_without_downsample", publish_odometry_without_downsample);
    nh->get_parameter("publish.path_en", path_en);
    nh->get_parameter("publish.scan_publish_en", scan_pub_en);
    nh->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);
    nh->get_parameter("runtime_pos_log_enable", runtime_pos_log);
    nh->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
    nh->get_parameter("pcd_save.interval", pcd_save_interval);
}
