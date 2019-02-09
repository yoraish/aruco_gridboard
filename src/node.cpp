
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>

#include "node.h"
#include "names.h"

static const std::string OPENCV_WINDOW = "Image window";

namespace aruco_gridboard{
Node::Node() :
    n_("~"),
    queue_size_(1),
    board_path_(),
    model_description_(),
    detector_param_path_(),
    camera_frame_name_(),
    debug_display_(false),
    status_tracker_(false),
    cv_ptr(),
    image_header_(),
    got_image_(false),
    lastHeaderSeq_(0)
{
    //get the tracker configuration file
    //this file contains all of the tracker's parameters, they are not passed to ros directly.
    n_.param<std::string>("board_path", board_path_, "");
    n_.param<bool>("debug_display", debug_display_, false);
    n_.param<std::string>("detector_param_path", detector_param_path_, "");
    n_.param<std::string>("camera_frame_name", camera_frame_name_, "camera");
    n_.param("frequency", freq_, 30);
    n_.param("camera_offset_x", camera_offset_x_, 0.0);
    n_.param("camera_offset_y", camera_offset_y_, 0.0);
    n_.param("camera_offset_z", camera_offset_z_, 0.0);
    ROS_INFO("Detector parameter file =%s",detector_param_path_.c_str());
    ROS_INFO("Board config file: =%s",board_path_.c_str());
}

Node::~Node()
{
}

void Node::waitForImage(){
    while ( ros::ok ()){
        if(got_image_) return;
        ros::spinOnce();
    }
}

//Read parameters from a file
static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

void Node::imageCallback(const sensor_msgs::ImageConstPtr & image) {
    image_header_ = image->header;
    try
    {
        boost::mutex::scoped_lock(lock_);
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    got_image_ = true;
}

void Node::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_info) {
    try
    {
        camera_model_.fromCameraInfo(cam_info);

        camMatrix_ = cv::Mat(camera_model_.fullIntrinsicMatrix());
        distCoeffs_= cv::Mat(camera_model_.distortionCoeffs());
        if (distCoeffs_.size[1] < 4)
            distCoeffs_ = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
        //ROS_INFO("Camera info %f", distCoeffs_.at<double>(0, 0));
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}

cv::Vec4d rotationMatrixToQuaternion(cv::Mat &R)
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);

    double Q[4];
    if (trace > 0.0)
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    }
    else
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
    return cv::Vec4d(Q[0], Q[1], Q[2], Q[3]);
}

void Node::spin(){

    ros::Subscriber caminfo_sub = n_.subscribe(camera_info_topic, 1, &Node::camInfoCallback, this);
    image_transport::ImageTransport imt(n_);
    image_transport::Subscriber img_sub = imt.subscribe(image_topic, 1, &Node::imageCallback, this);

    // Define Publisher
    ros::Publisher object_pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(object_position_topic, queue_size_);
    ros::Publisher camera_pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(camera_position_topic, queue_size_);
    ros::Publisher status_publisher = n_.advertise<std_msgs::Int8>(status_topic, queue_size_);

    //wait for an image to be ready
    waitForImage();

    geometry_msgs::PoseStamped msg_pose;
    geometry_msgs::PoseStamped msg_camera_pose;
    geometry_msgs::PoseStamped msg_camera_pose_0;

    msg_camera_pose_0.pose.position.x = 0;
    msg_camera_pose_0.pose.position.y = 0;
    msg_camera_pose_0.pose.position.z = 0;
    msg_camera_pose_0.pose.orientation.x = 0.0;
    msg_camera_pose_0.pose.orientation.y = 0.0;
    msg_camera_pose_0.pose.orientation.z = 0.707106781187;
    msg_camera_pose_0.pose.orientation.w = 0.707106781187;

    std_msgs:: Int8 status;
    ros::Rate rate(freq_);
    unsigned int cont = 0;

    // Read config file describing the board
    cv::FileStorage fs(board_path_, cv::FileStorage::READ);

    float mm_px =  fs["mm_per_unit"] ;
    mm_px *= 0.001;

    // Parse corners
    cv::FileNode corners_node = fs["corners"];
    cv::FileNodeIterator it = corners_node.begin(), it_end = corners_node.end();
    int idx = 0;
    std::vector<std::vector<float> > lbpval;
    std::vector< std::vector<cv::Point3f> > objPoints_;
    for( ; it != it_end; ++it, idx++ )
    {
        // std::cout << "Reaning corner #" << idx << ": ";
        (*it) >> lbpval;
        //std::cout << lbpval[0][0] << std::endl;
        std::vector<cv::Point3f> points;
        points.push_back(cv::Point3f(mm_px*lbpval[0][0], mm_px*lbpval[0][1], mm_px*lbpval[0][2]));
        points.push_back(cv::Point3f(mm_px*lbpval[1][0], mm_px*lbpval[1][1], mm_px*lbpval[1][2]));
        points.push_back(cv::Point3f(mm_px*lbpval[2][0], mm_px*lbpval[2][1], mm_px*lbpval[2][2]));
        points.push_back(cv::Point3f(mm_px*lbpval[3][0], mm_px*lbpval[3][1], mm_px*lbpval[3][2]));
        objPoints_.push_back(points);
    }

    // Parse ids
    std::vector< int > ids_;
    fs["ids"]  >> ids_;

    fs.release();

    // Create a board
    //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(1));
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(16));
    cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(objPoints_,dictionary,ids_);

    //objPoints_[0] = objPoints_[8];
    //ids_[0] = ids_[8];
    //objPoints_.resize(1);
    //ids_.resize(1);

    cv::Vec3d rvec, tvec;

    for (unsigned int i = 0; i < objPoints_.size(); i++)
    {
        for (unsigned int j = 0; j< objPoints_[i].size(); j++)
            std::cout<< objPoints_[i][j] << " ";
        std::cout << std::endl;
    }

    for (unsigned int i = 0; i < ids_.size(); i++)
    {
        std::cout<< ids_[i] << std::endl;
    }

    while(ros::ok()){

        if (lastHeaderSeq_ != image_header_.seq)
        {

            // Get the picture
            cv::Mat imageCopy;

            {
                boost::mutex::scoped_lock(lock_);
                cv_ptr->image.copyTo(imageCopy);
            }

            // Load parameters for the detector
            cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
            bool readOk = readDetectorParameters(detector_param_path_, detectorParams);
            if(!readOk) {
                std::cerr << "Invalid detector parameters file" << std::endl;
                return ;
            }
            detectorParams->cornerRefinementMethod = 1; // do corner refinement in markers

            // Now detect the markers
            std::vector< int > ids;
            std::vector< std::vector< cv::Point2f > > corners, rejected;

            //cv::Ptr<cv::aruco::Dictionary> dictionary  = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(0));
            cv::aruco::detectMarkers(imageCopy, board->dictionary, corners, ids, detectorParams, rejected);
            if (debug_display_) {
                std::cout << "Found  " << corners.size() << " markers." << std::endl;
                //std::cout << "Found  " << rejected.size() << " rejected." << std::endl;
            }

            // Now estimate the pose of the board
            int markersOfBoardDetected = 0;

            if(ids.size() > 0)
                markersOfBoardDetected = cv::aruco::estimatePoseBoard(corners, ids, board, camMatrix_, distCoeffs_, rvec, tvec);

            if (debug_display_) {
                std::cout << "size ids found:" <<ids.size() <<" " <<  board->ids.size()<< std::endl;
                std::cout << "markersOfBoardDetected:" <<markersOfBoardDetected << std::endl;
                std::cout << "objPoints:" <<corners.size() <<" " <<   board->objPoints.size()<< std::endl;
                //std::cout << "objPoints:" <<corners[0][0] <<" " <<   board->objPoints[0][0]<< std::endl;
            }

            if(markersOfBoardDetected )
            {
                if (debug_display_) {
                    std::cout << "r:" <<rvec << std::endl;
                    std::cout << "t:" <<tvec << std::endl;
                }

                if (cv::norm(tvec) > 0.00001)
                {
                    if (debug_display_) {
                        cv::aruco::drawAxis(imageCopy, camMatrix_, distCoeffs_, rvec, tvec, 0.4);
                    }
                    status_tracker_ = 1;
                }
                else
                {
                    std::cout << "Cannot estimate the pose" << std::endl;
                    status_tracker_ = 0;
                }
            }
            else
                status_tracker_ = 0;

            //if(rejected.size() > 0)
            //    cv::aruco::drawDetectedMarkers(imageCopy, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

            // Draw markers on the image
            if (ids.size() > 0)
                cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if (status_tracker_) {
                // Publish pose
                ros::Time now = ros::Time::now();
                msg_pose.header.stamp = now;
                msg_pose.header.frame_id = image_header_.frame_id;
                msg_pose.pose.position.x = tvec[0];
                msg_pose.pose.position.y = tvec[1];
                msg_pose.pose.position.z = tvec[2];

                cv::Mat rot_mat(3, 3, cv::DataType<float>::type);
                cv::Rodrigues(rvec, rot_mat);

                cv::Vec3d eulerAngles = rotationMatrixToEulerAngles(rot_mat);

                //cv::Vec4d cv_quat = rotationMatrixToQuaternion(rot_mat);

                geometry_msgs::Quaternion p_quat = tf::createQuaternionMsgFromRollPitchYaw(eulerAngles[0], eulerAngles[1], eulerAngles[2]);

                //geometry_msgs::Quaternion p_quat;
                //p_quat.x = cv_quat[0];
                //p_quat.y = cv_quat[1];
                //p_quat.z = cv_quat[2];
                //p_quat.w = cv_quat[3];

                msg_pose.pose.orientation = p_quat;

                object_pose_publisher.publish(msg_pose);

                //Publish status
                status.data = 1;
                status_publisher.publish(status);

                // transform and message to publish camera pose
                static tf::TransformBroadcaster br;

                tf::Transform transform;
                transform = tf::Transform(tf::Quaternion(p_quat.x, p_quat.y, p_quat.z, p_quat.w),
                    tf::Vector3(tvec[0] - camera_offset_x_, tvec[1] - camera_offset_y_, tvec[2] - camera_offset_z_));
                //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", camera_frame_name_));

                tf::Transform inv_transform;
                inv_transform = transform.inverse();
                const tf::Vector3 camera_origin = inv_transform.getOrigin();
                const tf::Quaternion camera_quaternion = inv_transform.getRotation();

                // what follows is from trial and error till I get the right orientation
                // for sure it could be better
                tf::Quaternion q_orig, q_rot, q_new;
                double r=M_PI, p=0, y=M_PI*1.5;  // Rotate the previous pose
                q_rot = tf::createQuaternionFromRPY(r, p, y);
                q_orig = camera_quaternion;
                q_new = q_rot*q_orig;  // Calculate the new orientation
                q_new.normalize();

                //msg_camera_pose.header.stamp = now;
                msg_camera_pose.header.stamp = image_header_.stamp;
                msg_camera_pose.header.frame_id = "world";
                msg_camera_pose.pose.position.x = camera_origin.getX();
                msg_camera_pose.pose.position.y = camera_origin.getY();
                msg_camera_pose.pose.position.z = camera_origin.getZ();
                msg_camera_pose.pose.orientation.x = -q_new.getY();
                msg_camera_pose.pose.orientation.y = -q_new.getX();
                msg_camera_pose.pose.orientation.z = -q_new.getZ();
                msg_camera_pose.pose.orientation.w = q_new.getW();

                camera_pose_publisher.publish(msg_camera_pose);

                msg_camera_pose_0.pose = msg_camera_pose.pose;

            }
            else
            {
                msg_camera_pose_0.header.stamp = image_header_.stamp;
                msg_camera_pose_0.header.frame_id = "world";

                camera_pose_publisher.publish(msg_camera_pose_0);

                status.data = 0;
                status_publisher.publish(status);
                ROS_DEBUG_THROTTLE(2, "No target detected");
            }

            if (debug_display_) {
                cv::imshow(OPENCV_WINDOW, imageCopy);
                cv::waitKey(2);
            }

            cont ++;

            lastHeaderSeq_ = image_header_.seq;
        }

        ros::spinOnce();
        rate.sleep();

    } //end while

    if (debug_display_) {
        cv::destroyWindow(OPENCV_WINDOW);
    }
} // end spin



}
