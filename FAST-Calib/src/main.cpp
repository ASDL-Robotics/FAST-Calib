/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include <rclcpp/rclcpp.hpp>

// ---------------------------------------------------------------------------
// saveDebugClouds: write all intermediate clouds to <output>/debug/ so they
// are available for post-mortem inspection regardless of whether calibration
// succeeded or failed.
// ---------------------------------------------------------------------------
static void saveDebugClouds(
    const std::string& output_path,
    const LidarDetectPtr& lidarDetectPtr,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& qr_centers,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidar_centers,
    const cv::Mat& qr_image)
{
    std::string debug_dir = output_path;
    if (debug_dir.back() != '/') debug_dir += '/';
    debug_dir += "debug/";
    std::filesystem::create_directories(debug_dir);

    auto save_pcd = [&](const std::string& name, auto cloud) {
        if (!cloud || cloud->empty()) {
            std::cout << BOLDYELLOW << "[Debug] " << name << ": empty, skipping." << RESET << std::endl;
            return;
        }
        std::string path = debug_dir + name;
        if (pcl::io::savePCDFileBinary(path, *cloud) == 0)
            std::cout << BOLDGREEN << "[Debug] Saved " << name
                      << " (" << cloud->size() << " pts) → " << path << RESET << std::endl;
        else
            std::cerr << BOLDRED << "[Debug] Failed to save " << name << RESET << std::endl;
    };

    save_pcd("filtered_cloud.pcd",   lidarDetectPtr->getFilteredCloud());
    save_pcd("plane_cloud.pcd",      lidarDetectPtr->getPlaneCloud());
    save_pcd("edge_cloud.pcd",       lidarDetectPtr->getEdgeCloud());
    save_pcd("aligned_cloud.pcd",    lidarDetectPtr->getAlignedCloud());
    save_pcd("center_z0_cloud.pcd",  lidarDetectPtr->getCenterZ0Cloud());
    save_pcd("lidar_centers.pcd",    lidar_centers);
    save_pcd("qr_centers.pcd",       qr_centers);

    if (!qr_image.empty()) {
        std::string img_path = debug_dir + "qr_detect.png";
        if (cv::imwrite(img_path, qr_image))
            std::cout << BOLDGREEN << "[Debug] Saved qr_detect.png → " << img_path << RESET << std::endl;
        else
            std::cerr << BOLDRED << "[Debug] Failed to save qr_detect.png" << RESET << std::endl;
    }

    std::cout << BOLDCYAN << "[Debug] Intermediate clouds saved to: " << debug_dir << RESET << std::endl;
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mono_qr_pattern");

    // Load parameters
    Params params = loadParameters(node);

    // Initialize QR detection and LiDAR detection
    QRDetectPtr qrDetectPtr;
    qrDetectPtr.reset(new QRDetect(node, params));

    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr.reset(new LidarDetect(node, params));

    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr.reset(new DataPreprocess(params));

    // Abort early if data loading failed
    if (!dataPreprocessPtr->ok_)
    {
        RCLCPP_ERROR(node->get_logger(), "[Main] Data loading failed. Aborting.");
        rclcpp::shutdown();
        return 1;
    }

    // Read image and point cloud
    cv::Mat img_input = dataPreprocessPtr->img_input_;
    pcl::PointCloud<Common::Point>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;

    if (img_input.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "[Main] Image is empty after loading. Aborting.");
        rclcpp::shutdown();
        return 1;
    }
    if (cloud_input->empty())
    {
        RCLCPP_ERROR(node->get_logger(), "[Main] Point cloud is empty after loading. Aborting.");
        rclcpp::shutdown();
        return 1;
    }
    
    // Detect QR codes
    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);

    if (qr_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
        RCLCPP_ERROR(node->get_logger(),
            "[Main] QR detection failed: found %zu circle centers, expected %d. "
            "Check image path, marker IDs, min_detected_markers, and target geometry params.",
            qr_center_cloud->size(), TARGET_NUM_CIRCLES);
        // Save whatever debug clouds exist so far (LiDAR not yet run — all empty)
        saveDebugClouds(params.output_path, lidarDetectPtr, qr_center_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>),
                        qrDetectPtr->imageCopy_);
        rclcpp::shutdown();
        return 1;
    }

    // Detect LiDAR data
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_center_cloud->reserve(4);
    
    switch (dataPreprocessPtr->lidar_type_)
    {
        case LiDARType::Solid:
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
            break;

        case LiDARType::Mech:
            lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
            break;

        default:
            RCLCPP_ERROR(node->get_logger(), "[Main] Unknown LiDAR type. Aborting.");
            rclcpp::shutdown();
            return 1;
    }

    if (lidar_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
        RCLCPP_ERROR(node->get_logger(),
            "[Main] LiDAR detection failed: found %zu circle centers, expected %d. "
            "Check filter bounds (x/y/z_min/max), circle_radius, and delta_*_circles params. "
            "Filtered cloud: %zu pts, plane cloud: %zu pts, edge cloud: %zu pts.",
            lidar_center_cloud->size(), TARGET_NUM_CIRCLES,
            lidarDetectPtr->getFilteredCloud()->size(),
            lidarDetectPtr->getPlaneCloud()->size(),
            lidarDetectPtr->getEdgeCloud()->size());
        // Save all intermediate clouds — this is the most useful failure case to inspect
        saveDebugClouds(params.output_path, lidarDetectPtr, qr_center_cloud,
                        lidar_center_cloud, qrDetectPtr->imageCopy_);
        rclcpp::shutdown();
        return 1;
    }

    // Sort detected circle centers from QR and LiDAR
    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_centers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    sortPatternCenters(qr_center_cloud, qr_centers, "camera");
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

    if (qr_centers->size() != TARGET_NUM_CIRCLES || lidar_centers->size() != TARGET_NUM_CIRCLES)
    {
        RCLCPP_ERROR(node->get_logger(),
            "[Main] Sorting failed: qr_centers=%zu, lidar_centers=%zu (expected %d each). Aborting.",
            qr_centers->size(), lidar_centers->size(), TARGET_NUM_CIRCLES);
        rclcpp::shutdown();
        return 1;
    }

    // Save intermediate results: sorted LiDAR and QR circle centers
    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    // Save debug clouds to disk (always — useful for validation even on success)
    saveDebugClouds(params.output_path, lidarDetectPtr, qr_centers,
                    lidar_centers, qrDetectPtr->imageCopy_);

    // Calculate extrinsic parameters
    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    // Transform LiDAR point cloud to QR coordinate system
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);
    
    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0) 
    {
      std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
      << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

    auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

    if (!DEBUG)
    {
        RCLCPP_INFO(node->get_logger(), "[Main] Calibration complete. Exiting.");
        rclcpp::shutdown();
        return 0;
    }

    // DEBUG: publish intermediate clouds for visualization in RViz.
    // Spins for up to 60 seconds then exits cleanly.
    RCLCPP_INFO(node->get_logger(), "[Main] DEBUG mode: publishing results for 60 s. Press Ctrl-C to exit sooner.");
    rclcpp::Rate rate(1);
    auto deadline = node->get_clock()->now() + rclcpp::Duration::from_seconds(60.0);
    while (rclcpp::ok() && node->get_clock()->now() < deadline)
    {
        // Publish QR detection results
        sensor_msgs::msg::PointCloud2 qr_centers_msg;
        pcl::toROSMsg(*qr_centers, qr_centers_msg);
        qr_centers_msg.header.stamp = node->get_clock()->now();
        qr_centers_msg.header.frame_id = "map";
        qrDetectPtr->qr_pub_->publish(qr_centers_msg);

        // Publish LiDAR detection results
        sensor_msgs::msg::PointCloud2 lidar_centers_msg;
        pcl::toROSMsg(*lidar_centers, lidar_centers_msg);
        lidar_centers_msg.header = qr_centers_msg.header;
        lidarDetectPtr->center_pub_->publish(lidar_centers_msg);

        // Publish intermediate results
        sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getFilteredCloud(), filtered_cloud_msg);
        filtered_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->filtered_pub_->publish(filtered_cloud_msg);

        sensor_msgs::msg::PointCloud2 plane_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getPlaneCloud(), plane_cloud_msg);
        plane_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->plane_pub_->publish(plane_cloud_msg);

        sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getAlignedCloud(), aligned_cloud_msg);
        aligned_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->aligned_pub_->publish(aligned_cloud_msg);

        sensor_msgs::msg::PointCloud2 edge_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getEdgeCloud(), edge_cloud_msg);
        edge_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->edge_pub_->publish(edge_cloud_msg);

        sensor_msgs::msg::PointCloud2 lidar_centers_z0_msg;
        pcl::toROSMsg(*lidarDetectPtr->getCenterZ0Cloud(), lidar_centers_z0_msg);
        lidar_centers_z0_msg.header = qr_centers_msg.header;
        lidarDetectPtr->center_z0_pub_->publish(lidar_centers_z0_msg);

        // Publish transformed LiDAR point cloud
        sensor_msgs::msg::PointCloud2 aligned_lidar_centers_msg;
        pcl::toROSMsg(*aligned_lidar_centers, aligned_lidar_centers_msg);
        aligned_lidar_centers_msg.header = qr_centers_msg.header;
        aligned_lidar_centers_pub->publish(aligned_lidar_centers_msg);

        // Publish colored point cloud
        sensor_msgs::msg::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
        colored_cloud_msg.header = qr_centers_msg.header;
        colored_cloud_pub->publish(colored_cloud_msg);
      
      rclcpp::spin_some(node);
      rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}