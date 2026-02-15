/**
 * Multi-Drone ArUco Detector - C++ port of aruco_mission_node.py
 * Subscribes to per-drone image, camera_info, pose; publishes TargetInfo
 */

 #include <cv_bridge/cv_bridge.h>
 #include <geometry_msgs/msg/point.hpp>
 #include <geometry_msgs/msg/pose_stamped.hpp>
 #include <icuas25_msgs/msg/target_info.hpp>
 #include <opencv2/aruco.hpp>
 #include <opencv2/opencv.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/camera_info.hpp>
 #include <sensor_msgs/msg/image.hpp>
 #include <std_msgs/msg/bool.hpp>
 #include <tf2/LinearMath/Matrix3x3.h>
 #include <tf2/LinearMath/Quaternion.h>
 
 #include <atomic>
 #include <cmath>
 #include <map>
 #include <memory>
 #include <mutex>
 #include <string>
 #include <unordered_set>
 #include <vector>
 
 // ============================================================
 // GROUND TRUTH DATA (Extracted from Gazebo World XML)
 // format: id -> (x, y, z)
 // ============================================================
//  static const std::map<int, std::tuple<double, double, double>> GROUND_TRUTH{
//      {11, {-5.9245, -1.3903, 2.0560}}, {14, {7.6041, 2.8089, 2.3750}},
//      {16, {-3.8641, 6.9430, 1.7940}},  {19, {4.8932, -4.3532, 1.7024}},
//      {15, {8.1615, 8.7107, 1.9291}},   {13, {4.8295, 1.5269, 1.0629}},
//      {18, {-2.6701, 7.2078, 2.9439}},  {20, {-7.6504, -8.5481, 2.3014}},
//      {17, {1.2507, -6.5897, 1.7486}},  {4, {0.5277, -4.3226, 3.1164}},
//      {29, {7.6705, -5.2531, 2.0552}},  {21, {-7.4942, -9.0051, 2.9806}},
//      {39, {-7.7655, -9.0323, 0.8734}}, {24, {-1.9260, -2.1796, 0.9641}},
//      {38, {-1.7333, -4.9403, 2.1630}}, {9, {-2.0129, -2.0421, 1.9796}},
//      {6, {0.4546, -3.8881, 1.0082}},   {36, {6.9151, -8.2716, 1.3943}},
//      {35, {-5.8848, -1.2585, 0.6699}}, {8, {5.9808, 5.9191, 1.8864}},
//      {25, {6.5083, -2.1766, 1.0218}},  {37, {-1.2361, -4.8519, 1.2981}},
//      {2, {-2.2373, 1.9367, 0.9951}},   {32, {6.0451, 6.4071, 1.0640}},
//      {26, {-2.1589, 7.1959, 1.2590}},  {22, {-8.6736, 4.2784, 0.5363}},
//      {7, {0.7506, -3.8684, 2.2749}},   {27, {-8.8289, 4.4761, 1.4376}},
//  };


 
 // ============================================================
 // PER-DRONE CONTEXT
 // ============================================================
 struct DroneContext {
   std::string cf;
   geometry_msgs::msg::PoseStamped::SharedPtr pose;
   cv::Mat K;
   cv::Mat D;
   std::unordered_set<int> marker_seen;
 };
 using DroneContextPtr = std::shared_ptr<DroneContext>;
 
 // ============================================================
 // MARKER DETECTION TRACKING FOR MULTI-DRONE AVERAGING
 // ============================================================
 struct MarkerDetection {
   double x, y, z;
   std::string drone_id;
   rclcpp::Time timestamp;
   double distance; // Distance from drone to marker
 };
 
 struct MarkerAggregator {
   std::vector<MarkerDetection> detections;
   size_t last_published_count;
   rclcpp::Time first_detection_time;
   // Last averaged values (updated when we compute average; used at RTH)
   bool has_last_avg = false;
   double last_avg_x = 0, last_avg_y = 0, last_avg_z = 0;
 
   MarkerAggregator() : last_published_count(0) {}
 };
 
 // ============================================================
 // MULTI-DRONE ARUCO DETECTOR
 // ============================================================
 class MultiArucoDetector : public rclcpp::Node {
 public:
   MultiArucoDetector() : Node("multi_aruco_detector") {
     // Config
     marker_size_ = 0.25;
     cf_ids_ = {"cf_1", "cf_2", "cf_3", "cf_4", "cf_5"};
     detection_timeout_ = 2.0; // seconds to wait for multi-drone detections
     outlier_threshold_ = 2.5; // standard deviations for outlier rejection
 
     // ArUco
     aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
     aruco_params_ = cv::aruco::DetectorParameters::create();
 
     target_found_pub_ =
         this->create_publisher<icuas25_msgs::msg::TargetInfo>("/target_found", 10);

     rth_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
         "RTH_STATE", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
           rth_active_.store(msg->data);
         });
 
     // Timer for periodic averaging and publishing
     publish_timer_ = this->create_wall_timer(
         std::chrono::milliseconds(500),
         std::bind(&MultiArucoDetector::checkAndPublishPendingMarkers, this));
 
     // Per-drone setup
     for (const auto &cf : cf_ids_) {
       auto ctx = std::make_shared<DroneContext>();
       ctx->cf = cf;
       drones_[cf] = ctx;
 
       // Subscribers
       auto sub_opt = rclcpp::SubscriptionOptions();
 
       image_subs_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
           std::string("/") + cf + "/image", rclcpp::SensorDataQoS(),
           [this, cf](const sensor_msgs::msg::Image::SharedPtr msg) {
             imageCb(msg, cf);
           },
           sub_opt));
 
       cam_info_subs_.push_back(
           this->create_subscription<sensor_msgs::msg::CameraInfo>(
               std::string("/") + cf + "/camera_info", 10,
               [this, cf](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                 cameraInfoCb(msg, cf);
               },
               sub_opt));
 
       pose_subs_.push_back(
           this->create_subscription<geometry_msgs::msg::PoseStamped>(
               std::string("/") + cf + "/pose", 10,
               [this, cf](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                 poseCb(msg, cf);
               },
               sub_opt));  
     }
 
     RCLCPP_INFO(this->get_logger(),
                 "Multi-Drone ArUco Detector Started (C++)");
   }
 
 private:
   void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg,
                     const std::string &cf) {
     std::lock_guard<std::mutex> lock(mutex_);
     auto it = drones_.find(cf);
     if (it == drones_.end())
       return;
     auto &ctx = it->second;
     if (ctx->K.empty()) {
       ctx->K = cv::Mat(3, 3, CV_64F);
       for (int i = 0; i < 9; ++i)
         ctx->K.at<double>(i / 3, i % 3) = msg->k[i];
       ctx->D = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
       for (size_t i = 0; i < msg->d.size(); ++i)
         ctx->D.at<double>(0, i) = msg->d[i];
     }
   }
 
   void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
               const std::string &cf) {
     std::lock_guard<std::mutex> lock(mutex_);
     auto it = drones_.find(cf);
     if (it != drones_.end())
       it->second->pose = msg;
   }
 
   void imageCb(const sensor_msgs::msg::Image::SharedPtr msg,
                const std::string &cf) {
     DroneContextPtr ctx;
     {
       std::lock_guard<std::mutex> lock(mutex_);
       auto it = drones_.find(cf);
       if (it == drones_.end())
         return;
       ctx = it->second;
       if (!ctx->pose || ctx->K.empty())
         return;
     }
 
     cv_bridge::CvImageConstPtr cv_ptr;
     try {
       cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
     } catch (const cv_bridge::Exception &e) {
       RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
       return;
     }
 
     cv::Mat gray;
     cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
 
     std::vector<std::vector<cv::Point2f>> corners;
     std::vector<int> ids;
     cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);
 
     if (ids.empty())
       return;
 
     std::vector<cv::Vec3d> rvecs, tvecs;
     cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, ctx->K, ctx->D,
                                          rvecs, tvecs);
 
     std::lock_guard<std::mutex> lock(mutex_);
     for (size_t i = 0; i < ids.size(); ++i) {
       int mid = ids[i];
       if (ctx->marker_seen.count(mid) == 0) ctx->marker_seen.insert(mid);
       const cv::Vec3d &tvec = tvecs[i];
       publishTarget(cf, mid, tvec[0], tvec[1], tvec[2], ctx);
     }
   }
 
   void publishTarget(const std::string &cf, int marker_id, double tx, double ty,
                      double tz, DroneContextPtr ctx) {
     const auto &pos = ctx->pose->pose.position;
     const auto &q = ctx->pose->pose.orientation;
 
     // Quaternion to yaw
     double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
 
     // Camera -> Drone frame (optical to body)
     // R_cam_to_drone = [[0,0,1],[-1,0,0],[0,-1,0]]
     double mx_d = tz;
     double my_d = -tx;
     double mz_d = -ty;
 
     // Drone -> World (yaw only)
     double c = std::cos(yaw), s = std::sin(yaw);
     double mx_w = c * mx_d - s * my_d;
     double my_w = s * mx_d + c * my_d;
     double mz_w = mz_d;
 
     double world_x = pos.x + mx_w;
     double world_y = pos.y + my_w;
     double world_z = pos.z + mz_w;
 
     // Calculate distance from drone to marker for weighted averaging
     double distance = std::sqrt(mx_w * mx_w + my_w * my_w + mz_w * mz_w);
 
     storeDetection(marker_id, cf, world_x, world_y, world_z, distance);
     RCLCPP_INFO(this->get_logger(), "[%s] Marker %d @ (%.2f, %.2f, %.2f)",
                 cf.c_str(), marker_id, world_x, world_y, world_z);
   }
 
   void storeDetection(int marker_id, const std::string &drone_id, double x,
                       double y, double z, double distance) {
     auto &agg = marker_detections_[marker_id];
     MarkerDetection det;
     det.x = x;
     det.y = y;
     det.z = z;
     det.drone_id = drone_id;
     det.timestamp = this->now();
     det.distance = distance;
 
     agg.detections.push_back(det);
     if (agg.detections.size() == 1)
       agg.first_detection_time = this->now();

     double elapsed = (this->now() - agg.first_detection_time).seconds();
     if (elapsed > detection_timeout_)
       publishAveragedMarker(marker_id, agg);
   }
 
   void checkAndPublishPendingMarkers() {
     std::lock_guard<std::mutex> lock(mutex_);
     auto now = this->now();

     // When RTH is active: publish last averaged value for ALL detected markers
     // in ascending marker ID order (same as logging order)
     if (rth_active_.load()) {
       std::vector<int> marker_ids;
       for (const auto &[mid, agg] : marker_detections_) {
         if (!agg.detections.empty()) marker_ids.push_back(mid);
       }
       std::sort(marker_ids.begin(), marker_ids.end());

       size_t count = 0;
       for (int marker_id : marker_ids) {
         auto &agg = marker_detections_[marker_id];
         double ax, ay, az;
         if (agg.has_last_avg) {
           ax = agg.last_avg_x;
           ay = agg.last_avg_y;
           az = agg.last_avg_z;
         } else {
           // No averaged value yet: use simple mean of all detections
           double sx = 0, sy = 0, sz = 0;
           for (const auto &d : agg.detections) {
             sx += d.x;
             sy += d.y;
             sz += d.z;
           }
           size_t n = agg.detections.size();
           ax = sx / n;
           ay = sy / n;
           az = sz / n;
         }
         icuas25_msgs::msg::TargetInfo msg;
         msg.id = marker_id;
         msg.location.x = static_cast<float>(ax);
         msg.location.y = static_cast<float>(ay);
         msg.location.z = static_cast<float>(az);
         target_found_pub_->publish(msg);
         count++;
         std::string err_msg;
        //  auto gt_it = GROUND_TRUTH.find(marker_id);
        //  if (gt_it != GROUND_TRUTH.end()) {
        //    auto [gt_x, gt_y, gt_z] = gt_it->second;
        //    double dist_err = std::sqrt((ax - gt_x) * (ax - gt_x) +
        //                                (ay - gt_y) * (ay - gt_y) +
        //                                (az - gt_z) * (az - gt_z));
        //    err_msg = " | Error: " + std::to_string(dist_err * 100.0).substr(0, 6) + " cm";
        //  } else {
        //    err_msg = " | Error: GT Not Found";
        //  }
         RCLCPP_INFO(this->get_logger(),
                     "RTH: Marker %d avg @ (%.2f, %.2f, %.2f)%s",
                     marker_id, ax, ay, az, err_msg.c_str());
       }
       if (count > 0) {
         RCLCPP_INFO(this->get_logger(),
                     "RTH: Published %zu marker(s) to /target_found", count);
       }
       return;
     }

     for (auto &[marker_id, agg] : marker_detections_) {
       if (agg.detections.empty())
         continue;
       if (agg.detections.size() <= agg.last_published_count)
         continue;
       double elapsed = (now - agg.first_detection_time).seconds();
       if (elapsed > detection_timeout_)
         publishAveragedMarker(marker_id, agg);
       else if (agg.detections.size() >= cf_ids_.size())
         publishAveragedMarker(marker_id, agg);
     }
   }
 
   void publishAveragedMarker(int marker_id, MarkerAggregator &agg) {
     if (agg.detections.empty())
       return;
 
     size_t n = agg.detections.size();
 
     // Step 1: Calculate simple average for outlier detection
     double mean_x = 0, mean_y = 0, mean_z = 0;
     for (const auto &det : agg.detections) {
       mean_x += det.x;
       mean_y += det.y;
       mean_z += det.z;
     }
     mean_x /= n;
     mean_y /= n;
     mean_z /= n;
 
     // Step 2: Calculate standard deviation
     double std_dev = 0;
     for (const auto &det : agg.detections) {
       double dx = det.x - mean_x;
       double dy = det.y - mean_y;
       double dz = det.z - mean_z;
       std_dev += dx * dx + dy * dy + dz * dz;
     }
     std_dev = std::sqrt(std_dev / n);
 
     // Step 3: Filter outliers and compute weighted average
     double weighted_x = 0, weighted_y = 0, weighted_z = 0;
     double total_weight = 0;

     for (const auto &det : agg.detections) {
       double dx = det.x - mean_x;
       double dy = det.y - mean_y;
       double dz = det.z - mean_z;
       double deviation = std::sqrt(dx * dx + dy * dy + dz * dz);
       if (n > 2 && deviation > outlier_threshold_ * std_dev)
         continue;
       double weight = 1.0 / (det.distance + 0.1);
       weighted_x += weight * det.x;
       weighted_y += weight * det.y;
       weighted_z += weight * det.z;
       total_weight += weight;
     }
 
     if (total_weight == 0) {
       RCLCPP_WARN(this->get_logger(),
                   "Marker %d: All detections rejected as outliers!", marker_id);
       agg.last_published_count = agg.detections.size(); // Mark as processed
       return;
     }
 
     double avg_x = weighted_x / total_weight;
     double avg_y = weighted_y / total_weight;
     double avg_z = weighted_z / total_weight;

     agg.last_avg_x = avg_x;
     agg.last_avg_y = avg_y;
     agg.last_avg_z = avg_z;
     agg.has_last_avg = true;
     // Update last published count (allows re-publishing with more drones)
     agg.last_published_count = agg.detections.size();
   }
 
   double marker_size_;
   std::vector<std::string> cf_ids_;
   cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
   cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
 
   std::map<std::string, DroneContextPtr> drones_;
   rclcpp::Publisher<icuas25_msgs::msg::TargetInfo>::SharedPtr target_found_pub_;
   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rth_state_sub_;
   std::atomic<bool> rth_active_{false};

   std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
       image_subs_;
   std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>
       cam_info_subs_;
   std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
       pose_subs_;
 
   // Multi-drone detection aggregation
   std::map<int, MarkerAggregator> marker_detections_; // marker_id -> aggregator
   rclcpp::TimerBase::SharedPtr publish_timer_;
   double detection_timeout_;
   double outlier_threshold_;
 
   std::mutex mutex_;
 };
 
 int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<MultiArucoDetector>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
 }