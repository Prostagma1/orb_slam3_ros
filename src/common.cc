/**
* 
* Common functions and variables across all modes (mono/stereo, with or w/o imu)
*
*/

#include "common.h"
#include <pcl_ros/point_cloud.h> // Для работы с PCL в ROS
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>      // Для сохранения PCD
#include <pcl_conversions/pcl_conversions.h> // Для конвертации
#include <pcl/filters/statistical_outlier_removal.h> // Фильтр SOR
#include <pcl/filters/radius_outlier_removal.h>     // <--- ДОБАВИТЬ Фильтр ROR
#include "orb_slam3_ros/SaveMap.h" // Ваш сервис
#include <ros/node_handle.h> // Для доступа к параметрам
#include <string>            // Для работы со строками (путь, тип фильтра)
// #include <filesystem>        // Для создания директории (требует C++17)
#include <boost/filesystem.hpp> 
#include <algorithm>         // Для std::transform (преобразование строки в нижний регистр)

// Variables for ORB-SLAM3
ORB_SLAM3::System* pSLAM;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::NOT_SET;

// Variables for ROS
std::string world_frame_id, cam_frame_id, imu_frame_id;
ros::Publisher pose_pub, odom_pub, kf_markers_pub;
ros::Publisher tracked_mappoints_pub, all_mappoints_pub;
ros::Publisher tracked_keypoints_pub;
image_transport::Publisher tracking_img_pub;

//////////////////////////////////////////////////
// Main functions
//////////////////////////////////////////////////
bool save_map_srv(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res)
{
    // --- 0. Инициализация и проверка ---
    if (!pSLAM) {
        ROS_ERROR("SaveMap Service: SLAM system is not initialized!");
        res.success = false;
        return false;
    }

    if (req.name.empty()) {
        ROS_ERROR("SaveMap Service: Map name cannot be empty!");
        res.success = false;
        return false;
    }

    ROS_INFO("SaveMap Service: Received request to save map components with base name '%s'", req.name.c_str());

    // Получаем NodeHandle для доступа к параметрам (используем приватный)
    ros::NodeHandle nh("~");

    // --- Параметры из launch файла ---
    std::string save_path = ".";
    bool enable_filter = true;
    std::string filter_type_str = "sor"; // Тип фильтра по умолчанию ("sor", "ror", "none")

    // Параметры SOR
    int sor_mean_k = 50;
    double sor_stddev_mul_thresh = 1.0;

    // Параметры ROR
    double ror_radius_search = 0.1; // Радиус поиска по умолчанию
    int ror_min_neighbors = 5;     // Мин. соседей в радиусе по умолчанию

    nh.param<std::string>("save_map_path", save_path, ".");
    nh.param<bool>("save_map_filter/enable", enable_filter, true);
    nh.param<std::string>("save_map_filter/type", filter_type_str, "sor");

    // Преобразуем тип фильтра в нижний регистр для сравнения без учета регистра
    std::transform(filter_type_str.begin(), filter_type_str.end(), filter_type_str.begin(), ::tolower);

    // Читаем параметры конкретного фильтра только если он выбран
    if (enable_filter) {
        if (filter_type_str == "sor") {
            nh.param<int>("save_map_filter/sor/mean_k", sor_mean_k, 50);
            nh.param<double>("save_map_filter/sor/stddev_thresh", sor_stddev_mul_thresh, 1.0);
            ROS_INFO("Save map filter type: SOR (MeanK=%d, StddevThresh=%.2f)", sor_mean_k, sor_stddev_mul_thresh);
        } else if (filter_type_str == "ror") {
            nh.param<double>("save_map_filter/ror/radius_search", ror_radius_search, 0.1);
            nh.param<int>("save_map_filter/ror/min_neighbors", ror_min_neighbors, 5);
            ROS_INFO("Save map filter type: ROR (Radius=%.3f, MinNeighbors=%d)", ror_radius_search, ror_min_neighbors);
        } else if (filter_type_str == "none") {
             ROS_INFO("Save map filter type: None (filtering disabled by type parameter)");
             enable_filter = false; // Отключаем фильтрацию если явно указано "none"
        } else {
            ROS_WARN("Unknown filter type '%s' specified. Disabling filtering.", filter_type_str.c_str());
            enable_filter = false; // Отключаем фильтрацию при неизвестном типе
        }
    } else {
        ROS_INFO("Save map filtering is disabled by enable parameter.");
    }
    ROS_INFO("Save map path: '%s'", save_path.c_str());


    // Создаем директорию для сохранения
    try {
         if (!save_path.empty() && save_path != ".") {
              boost::filesystem::create_directories(save_path); // Boost
         }
    } catch (const std::exception& e) {
         ROS_ERROR("Failed to create save directory '%s': %s", save_path.c_str(), e.what());
         save_path = ".";
         ROS_WARN("Falling back to saving in the current directory.");
    }


    // --- 1. Получение "сырого" облака точек ---
    ROS_INFO("Retrieving map points as PCL cloud...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_raw;
    try {
         map_cloud_raw = pSLAM->GetAllMapPointsAsPCL();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while calling GetAllMapPointsAsPCL: %s", e.what());
        map_cloud_raw = nullptr;
    }

    if (!map_cloud_raw || map_cloud_raw->empty()) {
        ROS_ERROR("Map point cloud is empty or could not be retrieved. Cannot save map.");
        res.success = false;
        return false;
    }
    ROS_INFO("Successfully retrieved %ld raw map points.", map_cloud_raw->size());


    // --- 2. Сохранение "сырого" облака ---
    std::string pcd_filename_raw = save_path + "/" + req.name + "_raw.pcd";
    bool raw_saved = false;
    ROS_INFO("Saving raw map point cloud to %s", pcd_filename_raw.c_str());
    try {
        if (pcl::io::savePCDFileBinary(pcd_filename_raw, *map_cloud_raw) == 0) {
            ROS_INFO("Raw map point cloud successfully saved.", pcd_filename_raw.c_str());
            raw_saved = true;
        } else {
            ROS_ERROR("Failed to save raw map point cloud.", pcd_filename_raw.c_str());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while saving raw PCD file %s: %s", pcd_filename_raw.c_str(), e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception while saving raw PCD file %s", pcd_filename_raw.c_str());
    }


    // --- 3. Фильтрация и Сохранение отфильтрованного облака ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    bool filtered_saved = false; // Флаг успеха сохранения отфильтрованного облака

    if (enable_filter) {
        ROS_INFO("Applying filter '%s'...", filter_type_str.c_str());
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>()); // Временное облако для результата фильтрации

        try {
            if (filter_type_str == "sor") {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(map_cloud_raw);
                sor.setMeanK(sor_mean_k);
                sor.setStddevMulThresh(sor_stddev_mul_thresh);
                sor.filter(*temp_cloud);
            } else if (filter_type_str == "ror") {
                pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
                ror.setInputCloud(map_cloud_raw);
                ror.setRadiusSearch(ror_radius_search);
                ror.setMinNeighborsInRadius(ror_min_neighbors);
                ror.filter(*temp_cloud);
            }
            // Если нужно будет добавить другие фильтры, использовать else if

            map_cloud_filtered = temp_cloud; // Переносим результат во внешнюю переменную
            ROS_INFO("Filtering complete. Points before: %ld, Points after: %ld",
                     map_cloud_raw->size(), map_cloud_filtered->size());

            if (map_cloud_filtered->empty()) {
                 ROS_WARN("Filtered point cloud is empty! Check filter parameters. Skipping save of filtered cloud.");
            } else {
                 // Сохраняем отфильтрованное облако
                 std::string pcd_filename_filtered = save_path + "/" + req.name + "_" + filter_type_str + "_filtered.pcd"; // Добавляем тип фильтра в имя файла
                 ROS_INFO("Saving filtered map point cloud to %s", pcd_filename_filtered.c_str());
                 try {
                     if (pcl::io::savePCDFileBinary(pcd_filename_filtered, *map_cloud_filtered) == 0) {
                         ROS_INFO("Filtered map point cloud successfully saved.");
                         filtered_saved = true;
                     } else {
                         ROS_ERROR("Failed to save filtered map point cloud.");
                     }
                 } catch (const std::exception& e) {
                     ROS_ERROR("Exception while saving filtered PCD file %s: %s", pcd_filename_filtered.c_str(), e.what());
                 } catch (...) {
                     ROS_ERROR("Unknown exception while saving filtered PCD file %s", pcd_filename_filtered.c_str());
                 }
            }

        } catch (const std::exception& e) {
             ROS_ERROR("Exception during filtering process: %s", e.what());
        } catch (...) {
             ROS_ERROR("Unknown exception during filtering process.");
        }
    } else {
        ROS_INFO("Filtering is disabled. Only raw point cloud will be saved.");
        // filtered_saved остается false, так как фильтрация не проводилась
    }


    // --- 4. Определение общего результата ---
    // Успех, если сырое облако сохранилось И (фильтрация была выключена ИЛИ (она была включена И успешно завершилась сохранением))
    res.success = raw_saved && (!enable_filter || (enable_filter && filtered_saved));

    if (res.success) {
         ROS_INFO("SaveMap service finished successfully.");
    } else {
         ROS_ERROR("SaveMap service finished with errors.");
    }

    return res.success;
}

bool save_traj_srv(orb_slam3_ros::SaveMap::Request &req, orb_slam3_ros::SaveMap::Response &res)
{
    const string cam_traj_file = req.name + "_cam_traj.txt";
    const string kf_traj_file = req.name + "_kf_traj.txt";

    try {
        pSLAM->SaveTrajectoryEuRoC(cam_traj_file);
        pSLAM->SaveKeyFrameTrajectoryEuRoC(kf_traj_file);
        res.success = true;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        res.success = false;
    } catch (...) {
        std::cerr << "Unknows exeption" << std::endl;
        res.success = false;
    }

    if (!res.success)
        ROS_ERROR("Estimated trajectory could not be saved.");

    return res.success;
}

void setup_services(ros::NodeHandle &node_handler, std::string node_name)
{
    static ros::ServiceServer save_map_service = node_handler.advertiseService(node_name + "/save_map", save_map_srv);
    static ros::ServiceServer save_traj_service = node_handler.advertiseService(node_name + "/save_traj", save_traj_srv);
}

void setup_publishers(ros::NodeHandle &node_handler, image_transport::ImageTransport &image_transport, std::string node_name)
{
    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped>(node_name + "/camera_pose", 1);

    tracked_mappoints_pub = node_handler.advertise<sensor_msgs::PointCloud2>(node_name + "/tracked_points", 1);

    tracked_keypoints_pub = node_handler.advertise<sensor_msgs::PointCloud2>(node_name + "/tracked_key_points", 1);

    all_mappoints_pub = node_handler.advertise<sensor_msgs::PointCloud2>(node_name + "/all_points", 1);

    tracking_img_pub = image_transport.advertise(node_name + "/tracking_image", 1);

    kf_markers_pub = node_handler.advertise<visualization_msgs::Marker>(node_name + "/kf_markers", 1000);

    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO || sensor_type == ORB_SLAM3::System::IMU_RGBD)
    {
        odom_pub = node_handler.advertise<nav_msgs::Odometry>(node_name + "/body_odom", 1);
    }
}

void publish_topics(ros::Time msg_time, Eigen::Vector3f Wbb)
{
    Sophus::SE3f Twc = pSLAM->GetCamTwc();

    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0,0)) // avoid publishing NaN
        return;
    
    // Common topics
    publish_camera_pose(Twc, msg_time);
    publish_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);

    publish_tracking_img(pSLAM->GetCurrentFrame(), msg_time);

    publish_keypoints(pSLAM->GetTrackedMapPoints(), pSLAM->GetTrackedKeyPoints(), msg_time);

    publish_tracked_points(pSLAM->GetTrackedMapPoints(), msg_time);
    publish_all_points(pSLAM->GetAllMapPoints(), msg_time);
    publish_kf_markers(pSLAM->GetAllKeyframePoses(), msg_time);

    // IMU-specific topics
    if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO || sensor_type == ORB_SLAM3::System::IMU_RGBD)
    {
        // Body pose and translational velocity can be obtained from ORB-SLAM3
        Sophus::SE3f Twb = pSLAM->GetImuTwb();
        Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

        // IMU provides body angular velocity in body frame (Wbb) which is transformed to world frame (Wwb)
        Sophus::Matrix3f Rwb = Twb.rotationMatrix();
        Eigen::Vector3f Wwb = Rwb * Wbb;

        publish_tf_transform(Twb, world_frame_id, imu_frame_id, msg_time);
        publish_body_odom(Twb, Vwb, Wwb, msg_time);
    }
}

void publish_body_odom(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, ros::Time msg_time)
{
    nav_msgs::Odometry odom_msg;
    odom_msg.child_frame_id = imu_frame_id;
    odom_msg.header.frame_id = world_frame_id;
    odom_msg.header.stamp = msg_time;

    odom_msg.pose.pose.position.x = Twb_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Twb_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Twb_SE3f.translation().z();

    odom_msg.pose.pose.orientation.w = Twb_SE3f.unit_quaternion().coeffs().w();
    odom_msg.pose.pose.orientation.x = Twb_SE3f.unit_quaternion().coeffs().x();
    odom_msg.pose.pose.orientation.y = Twb_SE3f.unit_quaternion().coeffs().y();
    odom_msg.pose.pose.orientation.z = Twb_SE3f.unit_quaternion().coeffs().z();

    odom_msg.twist.twist.linear.x = Vwb_E3f.x();
    odom_msg.twist.twist.linear.y = Vwb_E3f.y();
    odom_msg.twist.twist.linear.z = Vwb_E3f.z();

    odom_msg.twist.twist.angular.x = ang_vel_body.x();
    odom_msg.twist.twist.angular.y = ang_vel_body.y();
    odom_msg.twist.twist.angular.z = ang_vel_body.z();

    odom_pub.publish(odom_msg);
}

void publish_camera_pose(Sophus::SE3f Tcw_SE3f, ros::Time msg_time)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = world_frame_id;
    pose_msg.header.stamp = msg_time;

    pose_msg.pose.position.x = Tcw_SE3f.translation().x();
    pose_msg.pose.position.y = Tcw_SE3f.translation().y();
    pose_msg.pose.position.z = Tcw_SE3f.translation().z();

    pose_msg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
    pose_msg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
    pose_msg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
    pose_msg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

    pose_pub.publish(pose_msg);
}

void publish_tf_transform(Sophus::SE3f T_SE3f, string frame_id, string child_frame_id, ros::Time msg_time)
{
    tf::Transform tf_transform = SE3f_to_tfTransform(T_SE3f);

    static tf::TransformBroadcaster tf_broadcaster;

    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, msg_time, frame_id, child_frame_id));
}

void publish_tracking_img(cv::Mat image, ros::Time msg_time)
{
    std_msgs::Header header;

    header.stamp = msg_time;

    header.frame_id = world_frame_id;

    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    tracking_img_pub.publish(rendered_image_msg);
}

void publish_keypoints(std::vector<ORB_SLAM3::MapPoint*> tracked_map_points, std::vector<cv::KeyPoint> tracked_keypoints, ros::Time msg_time)
{   
    std::vector<cv::KeyPoint> finalKeypoints;

    int numKFs = tracked_keypoints.size();

    if (tracked_keypoints.empty())
        return;

    for (size_t i = 0; i < tracked_map_points.size(); i++) {
        if (tracked_map_points[i]) {  // if the MapPoint pointer is not nullptr
            finalKeypoints.push_back(tracked_keypoints[i]);
        }
    }

    sensor_msgs::PointCloud2 cloud = keypoints_to_pointcloud(finalKeypoints, msg_time);

    tracked_keypoints_pub.publish(cloud);
}

void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint*> tracked_points, ros::Time msg_time)
{
    // Получаем текущую позу камеры (World -> Camera)
    Sophus::SE3f Twc = pSLAM->GetCamTwc(); 
    // Проверяем на NaN перед инвертированием (хотя проверка есть в publish_topics)
    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0,0)) 
        return;
    Sophus::SE3f Tcw = Twc.inverse(); 

    // Вызываем обновленную функцию
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time, Tcw, cam_frame_id); // <--- ИЗМЕНЕНО
    
    tracked_mappoints_pub.publish(cloud);
}

void publish_all_points(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud_world(map_points, msg_time);
    
    all_mappoints_pub.publish(cloud);
}

// More details: http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html
void publish_kf_markers(std::vector<Sophus::SE3f> vKFposes, ros::Time msg_time)
{
    int numKFs = vKFposes.size();
    if (numKFs == 0)
        return;
    
    visualization_msgs::Marker kf_markers;
    kf_markers.header.frame_id = world_frame_id;
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = ros::Duration();

    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    for (int i = 0; i <= numKFs; i++)
    {
        geometry_msgs::Point kf_marker;
        kf_marker.x = vKFposes[i].translation().x();
        kf_marker.y = vKFposes[i].translation().y();
        kf_marker.z = vKFposes[i].translation().z();
        kf_markers.points.push_back(kf_marker);
    }
    
    kf_markers_pub.publish(kf_markers);
}

//////////////////////////////////////////////////
// Miscellaneous functions
//////////////////////////////////////////////////

sensor_msgs::PointCloud2 keypoints_to_pointcloud(std::vector<cv::KeyPoint>& keypoints, ros::Time msg_time) {
    const int num_channels = 3; // x y z

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id; 
    cloud.height = 1;
    cloud.width = keypoints.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);


    std::string channel_id[] = { "x", "y", "z" };

    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++) {
        float data_array[num_channels] = {
            keypoints[i].pt.x,
            keypoints[i].pt.y,
            0.0f // Z value is 0 for 2D keypoints
        };

        memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
    return cloud;

}

sensor_msgs::PointCloud2 mappoint_to_pointcloud_world(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);


    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

            tf::Vector3 point_translation(P3Dw.x(), P3Dw.y(), P3Dw.z());

            float data_array[num_channels] = {
                point_translation.x(),
                point_translation.y(),
                point_translation.z()
            };

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

sensor_msgs::PointCloud2 mappoint_to_pointcloud(
    std::vector<ORB_SLAM3::MapPoint*> map_points, 
    ros::Time msg_time,
    const Sophus::SE3f& Tcw, 
    const std::string& camera_frame_id)
{
    const int num_channels = 3; // x y z
    std::vector<Eigen::Vector3f> points_in_camera_frame; // Вектор для хранения точек в кадре камеры
    points_in_camera_frame.reserve(map_points.size());

    // Сначала преобразуем все валидные точки
    for (const auto& mp : map_points)
    {
        if (mp && !mp->isBad()) // Добавим проверку isBad() для надежности
        {
            Eigen::Vector3f P3Dw = mp->GetWorldPos(); // Используем float, т.к. Tcw - SE3f
            Eigen::Vector3f P3Dc = Tcw * P3Dw; // Преобразуем в кадр камеры
            points_in_camera_frame.push_back(P3Dc);
        }
    }

    if (points_in_camera_frame.empty())
    {
         sensor_msgs::PointCloud2 cloud;
         cloud.header.stamp = msg_time;
         cloud.header.frame_id = camera_frame_id; 
         cloud.height = 1;
         cloud.width = 0;
         cloud.point_step = num_channels * sizeof(float);
         cloud.row_step = 0;
         return cloud;
    }

    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = msg_time;
    cloud.header.frame_id = camera_frame_id; 
    cloud.height = 1;
    cloud.width = points_in_camera_frame.size(); 
    cloud.is_bigendian = false;
    cloud.is_dense = true; 
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};
    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);
    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        float data_array[num_channels] = {
            points_in_camera_frame[i].x(), 
            points_in_camera_frame[i].y(), 
            points_in_camera_frame[i].z() 
        };
        memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }

    return cloud;
}

cv::Mat SE3f_to_cvMat(Sophus::SE3f T_SE3f)
{
    cv::Mat T_cvmat;

    Eigen::Matrix4f T_Eig3f = T_SE3f.matrix();
    cv::eigen2cv(T_Eig3f, T_cvmat);
    
    return T_cvmat;
}

tf::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf::Matrix3x3 R_tf(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
    );

    tf::Vector3 t_tf(
        t_vec(0),
        t_vec(1),
        t_vec(2)
    );

    return tf::Transform(R_tf, t_tf);
}
