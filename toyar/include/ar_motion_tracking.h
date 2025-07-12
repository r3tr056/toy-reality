#ifndef AR_MOTION_TRACKING_H_
#define AR_MOTION_TRACKING_H_

#include "ar_la.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <memory>
#include <deque>

using namespace toyar;

// Feature point structure for tracking
struct TrackedFeature {
    cv::Point2f imagePoint;
    Vec3 worldPoint;
    int id;
    int age;
    bool isValid;
    float confidence;
};

// Pose structure with confidence
struct Pose6DOF {
    Vec3 position;
    Vec3 rotation; // Rodrigues vector
    Mat4 transformMatrix;
    float confidence;
    double timestamp;
    
    Pose6DOF() : position(0,0,0), rotation(0,0,0), confidence(0.0f), timestamp(0.0) {
        transformMatrix = Mat4::identity();
    }
};

// Motion tracking state
enum class TrackingState {
    NOT_TRACKING,
    INITIALIZING,
    TRACKING,
    LOST
};

class ARMotionTracker {
public:
    ARMotionTracker();
    ~ARMotionTracker();
    
    bool initialize(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, 
                   int imageWidth, int imageHeight);
    
    // Main tracking function - call this every frame
    bool processFrame(const cv::Mat& frame, double timestamp);
    
    // Get current pose
    Pose6DOF getCurrentPose() const { return currentPose; }
    TrackingState getTrackingState() const { return trackingState; }
    
    // Get tracked features for visualization
    std::vector<TrackedFeature> getTrackedFeatures() const { return trackedFeatures; }
    
    // Reset tracking
    void reset();
    
    // Configuration
    void setMaxFeatures(int maxFeatures) { this->maxFeatures = maxFeatures; }
    void setMinFeatures(int minFeatures) { this->minFeatures = minFeatures; }
    
private:
    // Core tracking methods
    bool initializeTracking(const cv::Mat& frame);
    bool trackFeatures(const cv::Mat& prevFrame, const cv::Mat& currentFrame);
    bool estimatePose(const cv::Mat& frame);
    void updateMapPoints();
    void detectNewFeatures(const cv::Mat& frame);
    
    // Feature management
    void cullBadFeatures();
    void addNewFeatures(const std::vector<cv::KeyPoint>& keypoints, const cv::Mat& frame);
    
    // Pose estimation helpers
    bool solvePnPRansac(const std::vector<cv::Point3f>& worldPoints,
                       const std::vector<cv::Point2f>& imagePoints,
                       cv::Mat& rvec, cv::Mat& tvec);
    
    // Triangulation
    cv::Point3f triangulatePoint(const cv::Point2f& pt1, const cv::Point2f& pt2,
                                const cv::Mat& P1, const cv::Mat& P2);
    
    // Bundle adjustment (simplified)
    void refineMapPoints();
    
    // Camera parameters
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int imageWidth, imageHeight;
    
    // Feature detection and tracking
    cv::Ptr<cv::ORB> orbDetector;
    cv::Ptr<cv::BFMatcher> matcher;
    
    // Optical flow tracker
    cv::TermCriteria flowCriteria;
    
    // Current state
    TrackingState trackingState;
    Pose6DOF currentPose;
    std::vector<TrackedFeature> trackedFeatures;
    
    // Frame history
    cv::Mat previousFrame;
    cv::Mat currentFrame;
    std::deque<Pose6DOF> poseHistory;
    
    // Feature tracking parameters
    int maxFeatures;
    int minFeatures;
    int nextFeatureId;
    
    // RANSAC parameters
    int ransacIterations;
    float ransacThreshold;
    float minInlierRatio;
    
    // Map management
    static const int MAX_POSE_HISTORY = 30;
    static const int MAX_FEATURE_AGE = 50;
    static const float MIN_PARALLAX;
    static const float MAX_REPROJECTION_ERROR;
    
    // Initialization
    bool isInitialized;
    int initializationFrameCount;
    std::vector<std::vector<cv::Point2f>> initializationFeatures;
    
    // Performance monitoring
    float averageTrackingTime;
    int frameCount;
};

// Utility functions for motion tracking
namespace MotionTrackingUtils {
    // Convert between OpenCV and toyar types
    Mat4 cvMatToMat4(const cv::Mat& rvec, const cv::Mat& tvec);
    void mat4ToCvMat(const Mat4& transform, cv::Mat& rvec, cv::Mat& tvec);
    
    // Geometric utilities
    float computeParallax(const cv::Point2f& pt1, const cv::Point2f& pt2);
    bool isPointInFront(const cv::Point3f& point, const cv::Mat& R, const cv::Mat& t);
    
    // Filtering
    void filterOutliers(std::vector<cv::Point2f>& points1, 
                       std::vector<cv::Point2f>& points2,
                       const cv::Mat& fundamentalMatrix);
}

#endif // AR_MOTION_TRACKING_H_
