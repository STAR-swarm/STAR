// ros
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "common/homography.h"

#include <Eigen/Dense>

typedef cv::Vec<float, 5> Vec5f;

#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}


typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

void getPose(const matd_t& H,
             const Mat3& Pinv,
             geometry_msgs::msg::Transform& t,
             const double size)
{
    // compute extrinsic camera parameter
    // https://dsp.stackexchange.com/a/2737/31703
    // H = K * T  =>  T = K^(-1) * H
    const Mat3 T = Pinv * Eigen::Map<const Mat3>(H.data);
    Mat3 R;
    R.col(0) = T.col(0).normalized();
    R.col(1) = T.col(1).normalized();
    R.col(2) = R.col(0).cross(R.col(1));

    // rotate by half rotation about x-axis to have z-axis
    // point upwards orthogonal to the tag plane
    R.col(1) *= -1;
    R.col(2) *= -1;

    // the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
    // hence the scale is half of the edge size
    const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm()) / 2.0) * (size / 2.0);

    const Eigen::Quaterniond q(R);

    t.translation.x = tt.x();
    t.translation.y = tt.y();
    t.translation.z = tt.z();
    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
}

Eigen::Matrix4d getRelativeTransform(
  std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints,
  cv::Matx33d cameraMatrix, Vec5f distCoeffs)
{
  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  cv::Mat rvec, tvec;
  
  // cv::Vecf distCoeffs(0, 0, 0, 0);  // distortion coefficients
  // TODO(H-HChen) Perhaps something like SOLVEPNP_EPNP would be faster? Would
  // need to first check WHAT is a bottleneck in this code, and only
  // do this if PnP solution is the bottleneck.
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d wRo;

  wRo << R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), R(0, 0), R(0, 1), R(0, 2);

  Eigen::Matrix4d T;  // homogeneous transformation matrix
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0, 0, 0, 1;
  return T;
}

void addObjectPoints(
  double s, cv::Matx44d T_oi, std::vector<cv::Point3d> & objectPoints)
{
    // Add to object point vector the tag corner coordinates in the bundle frame
    // Going counterclockwise starting from the bottom left corner
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, -s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, -s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, s, 0, 1));
}

void addImagePoints(
  apriltag_detection_t * detection, std::vector<cv::Point2d> & imagePoints)
{
    // Add to image point vector the tag corners in the image frame
    // Going counterclockwise starting from the bottom left corner
    double tag_x[4] = {-1, 1, 1, -1};
    double tag_y[4] = {1, 1, -1, -1};  // Negated because AprilTag tag local
                                        // frame has y-axis pointing DOWN
                                        // while we use the tag local frame
                                        // with y-axis pointing UP
    for (int i = 0; i < 4; i++) {
        // Homography projection taking tag local frame coordinates to image pixels
        double im_x, im_y;
        homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
        imagePoints.push_back(cv::Point2d(im_x, im_y));
    }
}

geometry_msgs::msg::TransformStamped makeTagPose(
  const Eigen::Matrix4d & transform, const Eigen::Quaternion<double> rot_quaternion,
  const std_msgs::msg::Header & header)
{
    geometry_msgs::msg::TransformStamped tf_;
    tf_.header = header;

    Eigen::Affine3d t;
    t.translation() = Eigen::Vector3d(
        transform(0, 3), transform(1, 3), transform(2, 3));

    t.linear() = rot_quaternion.toRotationMatrix();

    t = t.inverse();

    //===== Position and orientation
    // tf_.transform.translation.x = transform(0, 3);
    // tf_.transform.translation.y = transform(1, 3);
    // tf_.transform.translation.z = transform(2, 3);

    // tf_.transform.rotation.x = rot_quaternion.z();
    // tf_.transform.rotation.y = rot_quaternion.x();
    // tf_.transform.rotation.z = rot_quaternion.y();

    // tf_.transform.rotation.w = rot_quaternion.w();

    tf_.transform.translation.x = t.translation().x();
    tf_.transform.translation.y = t.translation().y();
    tf_.transform.translation.z = t.translation().z();

    tf_.transform.rotation.x = rot_quaternion.z();
    tf_.transform.rotation.y = rot_quaternion.x();
    tf_.transform.rotation.z = rot_quaternion.y();

    tf_.transform.rotation.w = rot_quaternion.w();
    return tf_;
}


class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    bool use_opencv_pnp = false;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    std::function<void(apriltag_family_t*)> tf_destructor;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    // topics
    sub_cam(image_transport::create_camera_subscription(this, "image_rect", 
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2), 
        declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    tf_broadcaster(this)
{
    // read-only parameters
    std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // whether to use OpenCV PnP or original by Christian Rauch
    use_opencv_pnp = declare_parameter("use_pnp", false, 
        descr("whether to use opencv pnp", false));

    tag_family = this->get_parameter("family").get_parameter_value().get<std::string>();
    tag_edge_size = this->get_parameter("size").get_parameter_value().get<double>();

    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));
    declare_parameter("detector.min_cluster_pixels", td->qtp.min_cluster_pixels, descr("reject quads containing too few pixels"));
    declare_parameter("detector.critical_rad", td->qtp.critical_rad, descr("Reject quads where pairs of edges have angles that are close to straight or close to 180 degrees"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }
}

AprilTagNode::~AprilTagNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // precompute inverse projection matrix
    const Mat3 Pinv = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(msg_ci->p.data()).leftCols<3>().inverse();
    
    // cv::Matx33d cameraMatrix(fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Matx33d cameraMatrix(
        msg_ci->k[0], 0, msg_ci->k[2], 
        0, msg_ci->k[4], msg_ci->k[5], 
        0, 0, 1);

    Vec5f distCoeffs(
        msg_ci->d[0], msg_ci->d[1], msg_ci->d[2], msg_ci->d[3], msg_ci->d[4]
    );

    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;
    msg_detections.header.stamp = this->get_clock()->now();

    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);        

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        msg_detection.size = tag_edge_size;
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        
        if (use_opencv_pnp)
        {
            std::vector<cv::Point3d> standaloneTagObjectPoints;
            std::vector<cv::Point2d> standaloneTagImagePoints;
            double tag_half_size = tag_edge_size / 2.0;
            addObjectPoints(tag_half_size, cv::Matx44d::eye(), standaloneTagObjectPoints);
            addImagePoints(det, standaloneTagImagePoints);

            Eigen::Matrix4d transform;
            transform = getRelativeTransform(
                standaloneTagObjectPoints, standaloneTagImagePoints, 
                cameraMatrix, distCoeffs);
            Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
            Eigen::Quaternion<double> rot_quaternion(rot);
            geometry_msgs::msg::TransformStamped tag_pose =
                makeTagPose(transform, rot_quaternion, msg_img->header);

            msg_detection.pose.pose.position.x = transform(0, 3);
            msg_detection.pose.pose.position.y = transform(1, 3);
            msg_detection.pose.pose.position.z = transform(2, 3);

            msg_detection.pose.pose.orientation = tag_pose.transform.rotation;
        }
        else
        {
            geometry_msgs::msg::Transform transform;
            getPose(*(det->H), Pinv, 
                transform, tag_sizes.count(det->id) ? 
                tag_sizes.at(det->id) : tag_edge_size);
            
            msg_detection.pose.pose.position.x = transform.translation.x;
            msg_detection.pose.pose.position.y = transform.translation.y;
            msg_detection.pose.pose.position.z = transform.translation.z;

            msg_detection.pose.pose.orientation = transform.rotation;
        }

        msg_detections.detections.push_back(msg_detection);

        // RCLCPP_INFO(get_logger(), "tag %d, pose (%.3lf, %.3lf, %.3lf)", det->id, transform(0, 3), transform(1, 3), transform(2, 3));

        // tfs.push_back(tf);
    }

    pub_detections->publish(msg_detections);
    tf_broadcaster.sendTransform(tfs);

    apriltag_detections_destroy(detections);
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("detector.min_cluster_pixels", td->qtp.min_cluster_pixels)
        IF("detector.critical_rad", td->qtp.critical_rad)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}
