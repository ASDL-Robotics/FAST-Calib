/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

// ---------------------------------------------------------------------------
// publishDebugClouds: spin for `duration_sec` seconds publishing all
// intermediate clouds to RViz topics. Called on both success and failure so
// the pipeline state is always inspectable without transferring files.
// ---------------------------------------------------------------------------
static void publishDebugClouds(
    std::shared_ptr<rclcpp::Node> node,
    const LidarDetectPtr& lidarDetectPtr,
    const QRDetectPtr& qrDetectPtr,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& qr_centers,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& lidar_centers,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_lidar_centers,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
    double duration_sec)
{
    auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

    rclcpp::Rate rate(1);
    auto deadline = node->get_clock()->now() + rclcpp::Duration::from_seconds(duration_sec);

    while (rclcpp::ok() && node->get_clock()->now() < deadline)
    {
        auto stamp = node->get_clock()->now();
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = "map";

        auto publish = [&](auto pub, auto cloud) {
            if (!cloud || cloud->empty()) return;
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            msg.header = header;
            pub->publish(msg);
        };

        publish(lidarDetectPtr->filtered_pub_,  lidarDetectPtr->getFilteredCloud());
        publish(lidarDetectPtr->plane_pub_,      lidarDetectPtr->getPlaneCloud());
        publish(lidarDetectPtr->aligned_pub_,    lidarDetectPtr->getAlignedCloud());
        publish(lidarDetectPtr->edge_pub_,       lidarDetectPtr->getEdgeCloud());
        publish(lidarDetectPtr->center_z0_pub_,  lidarDetectPtr->getCenterZ0Cloud());
        publish(lidarDetectPtr->center_pub_,     lidar_centers);
        publish(qrDetectPtr->qr_pub_,            qr_centers);
        publish(aligned_lidar_centers_pub,       aligned_lidar_centers);
        publish(colored_cloud_pub,               colored_cloud);

        rclcpp::spin_some(node);
        rate.sleep();
    }
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

    // Preallocate clouds referenced across all exit paths
    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Helper: publish debug state for duration_sec then exit with rc.
    // Only publishes when params.debug is true (set via 'debug: true' in qr_params.yaml).
    auto exit_with_debug = [&](int rc, double duration_sec) -> int {
        if (params.debug) {
            std::string label = (rc == 0) ? "success" : "FAILURE";
            RCLCPP_INFO(node->get_logger(),
                "[Main] Publishing debug clouds for %.0f s (%s). "
                "Subscribe in RViz: /filtered_cloud /plane_cloud /edge_cloud "
                "/aligned_cloud /center_z0_cloud /center_cloud /qr_cloud. "
                "Press Ctrl-C to exit sooner.",
                duration_sec, label.c_str());
            publishDebugClouds(node, lidarDetectPtr, qrDetectPtr,
                               qr_center_cloud, lidar_center_cloud,
                               aligned_lidar_centers, colored_cloud,
                               duration_sec);
        }
        rclcpp::shutdown();
        return rc;
    };

    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr.reset(new DataPreprocess(params));

    // Abort early if data loading failed (no clouds to publish)
    if (!dataPreprocessPtr->ok_)
    {
        RCLCPP_ERROR(node->get_logger(), "[Main] Data loading failed. Aborting.");
        rclcpp::shutdown();
        return 1;
    }

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

    // --- QR detection ---
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);

    if (qr_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
        RCLCPP_ERROR(node->get_logger(),
            "[Main] QR detection failed: found %zu circle centers, expected %d. "
            "Check image path, marker IDs, min_detected_markers, and target geometry params.",
            qr_center_cloud->size(), TARGET_NUM_CIRCLES);
        return exit_with_debug(1, 30.0);
    }

    // --- LiDAR detection ---
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
        return exit_with_debug(1, 30.0);
    }

    // --- Sort centers ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_centers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    sortPatternCenters(qr_center_cloud, qr_centers, "camera");
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

    if (qr_centers->size() != TARGET_NUM_CIRCLES || lidar_centers->size() != TARGET_NUM_CIRCLES)
    {
        RCLCPP_ERROR(node->get_logger(),
            "[Main] Sorting failed: qr_centers=%zu, lidar_centers=%zu (expected %d each). Aborting.",
            qr_centers->size(), lidar_centers->size(), TARGET_NUM_CIRCLES);
        return exit_with_debug(1, 30.0);
    }

    // Promote sorted centers into the shared pointers so exit_with_debug publishes them
    *qr_center_cloud = *qr_centers;
    *lidar_center_cloud = *lidar_centers;

    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    // --- Compute extrinsics ---
    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

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

    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_,
                             qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

    return exit_with_debug(0, 60.0);
}