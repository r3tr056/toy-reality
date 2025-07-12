#include "ar_motion_tracking.h"
#include <iostream>
#include <algorithm>
#include <chrono>

// Static constants
const float ARMotionTracker::MIN_PARALLAX = 1.0f;
const float ARMotionTracker::MAX_REPROJECTION_ERROR = 2.0f;

ARMotionTracker::ARMotionTracker() 
    : trackingState(TrackingState::NOT_TRACKING)
    , maxFeatures(500)
    , minFeatures(50)
    , nextFeatureId(0)
    , ransacIterations(1000)
    , ransacThreshold(1.0f)
    , minInlierRatio(0.7f)
    , isInitialized(false)
    , initializationFrameCount(0)
    , averageTrackingTime(0.0f)
    , frameCount(0)
{
    // Initialize ORB detector
    orbDetector = cv::ORB::create(maxFeatures);
    matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    
    // Optical flow criteria
    flowCriteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
}

ARMotionTracker::~ARMotionTracker() {
}

bool ARMotionTracker::initialize(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, 
                                int imageWidth, int imageHeight) {
    this->cameraMatrix = cameraMatrix.clone();
    this->distCoeffs = distCoeffs.clone();
    this->imageWidth = imageWidth;
    this->imageHeight = imageHeight;
    
    reset();
    
    std::cout << "Motion tracker initialized with image size: " 
              << imageWidth << "x" << imageHeight << std::endl;
    
    return true;
}

bool ARMotionTracker::processFrame(const cv::Mat& frame, double timestamp) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    if (frame.empty()) return false;
    
    // Convert to grayscale if needed
    cv::Mat grayFrame;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = frame.clone();
    }
    
    bool success = false;
    
    switch (trackingState) {
        case TrackingState::NOT_TRACKING:
            success = initializeTracking(grayFrame);
            if (success) {
                trackingState = TrackingState::INITIALIZING;
            }
            break;
            
        case TrackingState::INITIALIZING:
            success = trackFeatures(previousFrame, grayFrame);
            if (success && initializationFrameCount >= 10) {
                success = estimatePose(grayFrame);
                if (success) {
                    trackingState = TrackingState::TRACKING;
                    std::cout << "Motion tracking initialized successfully!" << std::endl;
                }
            }
            initializationFrameCount++;
            break;
            
        case TrackingState::TRACKING:
            success = trackFeatures(previousFrame, grayFrame);
            if (success) {
                success = estimatePose(grayFrame);
                if (!success || static_cast<int>(trackedFeatures.size()) < minFeatures) {
                    trackingState = TrackingState::LOST;
                    std::cout << "Tracking lost - insufficient features" << std::endl;
                }
            }
            break;
            
        case TrackingState::LOST:
            // Try to relocalize
            success = initializeTracking(grayFrame);
            if (success) {
                trackingState = TrackingState::INITIALIZING;
                initializationFrameCount = 0;
                std::cout << "Attempting to relocalize..." << std::endl;
            }
            break;
    }
    
    if (success) {
        currentPose.timestamp = timestamp;
        poseHistory.push_back(currentPose);
        if (poseHistory.size() > MAX_POSE_HISTORY) {
            poseHistory.pop_front();
        }
        
        // Detect new features if needed
        if (trackedFeatures.size() < maxFeatures * 0.7f) {
            detectNewFeatures(grayFrame);
        }
        
        // Update map points
        updateMapPoints();
        
        // Cull bad features
        cullBadFeatures();
    }
    
    previousFrame = grayFrame.clone();
    currentFrame = grayFrame.clone();
    
    // Update performance metrics
    auto endTime = std::chrono::high_resolution_clock::now();
    float frameTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();
    averageTrackingTime = (averageTrackingTime * frameCount + frameTime) / (frameCount + 1);
    frameCount++;
    
    return success;
}

bool ARMotionTracker::initializeTracking(const cv::Mat& frame) {
    trackedFeatures.clear();
    
    // Detect features
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orbDetector->detectAndCompute(frame, cv::noArray(), keypoints, descriptors);
    
    if (static_cast<int>(keypoints.size()) < minFeatures) {
        return false;
    }
    
    // Convert keypoints to tracked features
    for (size_t i = 0; i < keypoints.size() && static_cast<int>(i) < maxFeatures; ++i) {
        TrackedFeature feature;
        feature.imagePoint = keypoints[i].pt;
        feature.worldPoint = Vec3(0, 0, 0); // Will be triangulated later
        feature.id = nextFeatureId++;
        feature.age = 0;
        feature.isValid = true;
        feature.confidence = 1.0f;
        
        trackedFeatures.push_back(feature);
    }
    
    // Reset pose to identity
    currentPose = Pose6DOF();
    
    std::cout << "Initialized tracking with " << trackedFeatures.size() << " features" << std::endl;
    
    return true;
}

bool ARMotionTracker::trackFeatures(const cv::Mat& prevFrame, const cv::Mat& currentFrame) {
    if (trackedFeatures.empty() || prevFrame.empty()) {
        return false;
    }
    
    // Extract previous points
    std::vector<cv::Point2f> prevPoints, nextPoints;
    std::vector<int> validIndices;
    
    for (size_t i = 0; i < trackedFeatures.size(); ++i) {
        if (trackedFeatures[i].isValid) {
            prevPoints.push_back(trackedFeatures[i].imagePoint);
            validIndices.push_back(i);
        }
    }
    
    if (prevPoints.empty()) {
        return false;
    }
    
    // Track features using optical flow
    std::vector<uchar> status;
    std::vector<float> errors;
    
    cv::calcOpticalFlowPyrLK(prevFrame, currentFrame, prevPoints, nextPoints, 
                            status, errors, cv::Size(21, 21), 3, flowCriteria);
    
    // Update tracked features
    int validFeatureCount = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        int featureIdx = validIndices[i];
        
        if (status[i] && errors[i] < 30.0f) {
            // Check if point is within image bounds
            cv::Point2f& pt = nextPoints[i];
            if (pt.x >= 0 && pt.x < imageWidth && pt.y >= 0 && pt.y < imageHeight) {
                trackedFeatures[featureIdx].imagePoint = pt;
                trackedFeatures[featureIdx].age++;
                trackedFeatures[featureIdx].confidence = std::max(0.1f, 
                    trackedFeatures[featureIdx].confidence - errors[i] * 0.01f);
                validFeatureCount++;
            } else {
                trackedFeatures[featureIdx].isValid = false;
            }
        } else {
            trackedFeatures[featureIdx].isValid = false;
        }
    }
    
    std::cout << "Tracked " << validFeatureCount << " features" << std::endl;
    
    return validFeatureCount >= minFeatures;
}

bool ARMotionTracker::estimatePose(const cv::Mat& /*frame*/) {
    // Collect 3D-2D correspondences
    std::vector<cv::Point3f> worldPoints;
    std::vector<cv::Point2f> imagePoints;
    
    for (const auto& feature : trackedFeatures) {
        if (feature.isValid && feature.age > 2) {
            worldPoints.push_back(cv::Point3f(feature.worldPoint.x, 
                                            feature.worldPoint.y, 
                                            feature.worldPoint.z));
            imagePoints.push_back(feature.imagePoint);
        }
    }
    
    if (worldPoints.size() < 6) {
        // Not enough points for pose estimation, try to triangulate new points
        if (trackingState == TrackingState::INITIALIZING && poseHistory.size() >= 2) {
            // Use previous poses to triangulate points
            const Pose6DOF& prevPose = poseHistory[poseHistory.size() - 2];
            
            // Create projection matrices
            cv::Mat R1, t1, R2, t2;
            MotionTrackingUtils::mat4ToCvMat(prevPose.transformMatrix, R1, t1);
            cv::Rodrigues(R1, R1);
            
            cv::Mat P1, P2;
            cv::hconcat(R1, t1, P1);
            P1 = cameraMatrix * P1;
            
            // Current pose estimation using fundamental matrix
            std::vector<cv::Point2f> prevImagePoints, currImagePoints;
            for (const auto& feature : trackedFeatures) {
                if (feature.isValid && feature.age >= 1) {
                    // This is simplified - in practice you'd store previous positions
                    currImagePoints.push_back(feature.imagePoint);
                }
            }
            
            if (currImagePoints.size() >= 8) {
                // Estimate relative pose using 5-point algorithm
                cv::Mat E, R, t, mask;
                E = cv::findEssentialMat(prevImagePoints, currImagePoints, cameraMatrix, 
                                       cv::RANSAC, 0.999, 1.0, mask);
                
                if (!E.empty()) {
                    cv::recoverPose(E, prevImagePoints, currImagePoints, cameraMatrix, R, t, mask);
                    
                    // Update current pose
                    cv::Mat rvec;
                    cv::Rodrigues(R, rvec);
                    currentPose.transformMatrix = MotionTrackingUtils::cvMatToMat4(rvec, t);
                    
                    // Extract position and rotation
                    currentPose.position = Vec3(t.at<double>(0), t.at<double>(1), t.at<double>(2));
                    currentPose.rotation = Vec3(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
                    currentPose.confidence = 0.8f;
                    
                    return true;
                }
            }
        }
        return false;
    }
    
    // Solve PnP
    cv::Mat rvec, tvec;
    bool success = solvePnPRansac(worldPoints, imagePoints, rvec, tvec);
    
    if (success) {
        currentPose.transformMatrix = MotionTrackingUtils::cvMatToMat4(rvec, tvec);
        currentPose.position = Vec3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        currentPose.rotation = Vec3(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        currentPose.confidence = 0.9f;
        
        std::cout << "Pose estimated: pos(" << currentPose.position.x << ", " 
                  << currentPose.position.y << ", " << currentPose.position.z << ")" << std::endl;
    }
    
    return success;
}

bool ARMotionTracker::solvePnPRansac(const std::vector<cv::Point3f>& worldPoints,
                                    const std::vector<cv::Point2f>& imagePoints,
                                    cv::Mat& rvec, cv::Mat& tvec) {
    if (worldPoints.size() != imagePoints.size() || worldPoints.size() < 6) {
        return false;
    }
    
    std::vector<int> inliers;
    bool success = cv::solvePnPRansac(worldPoints, imagePoints, cameraMatrix, distCoeffs,
                                     rvec, tvec, false, ransacIterations, 
                                     ransacThreshold, 0.99, inliers);
    
    if (success) {
        float inlierRatio = float(inliers.size()) / worldPoints.size();
        if (inlierRatio < minInlierRatio) {
            return false;
        }
        
        // Refine with inliers only
        std::vector<cv::Point3f> inlierWorldPoints;
        std::vector<cv::Point2f> inlierImagePoints;
        
        for (int idx : inliers) {
            inlierWorldPoints.push_back(worldPoints[idx]);
            inlierImagePoints.push_back(imagePoints[idx]);
        }
        
        cv::solvePnP(inlierWorldPoints, inlierImagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    }
    
    return success;
}

void ARMotionTracker::updateMapPoints() {
    // Simplified map point update
    for (auto& feature : trackedFeatures) {
        if (feature.isValid && feature.age > 1) {
            // In a full SLAM system, this would involve bundle adjustment
            // For now, we just update confidence based on reprojection error
            
            if (trackingState == TrackingState::TRACKING) {
                // Project 3D point to image
                std::vector<cv::Point3f> objectPoints = {
                    cv::Point3f(feature.worldPoint.x, feature.worldPoint.y, feature.worldPoint.z)
                };
                std::vector<cv::Point2f> projectedPoints;
                
                cv::Mat rvec, tvec;
                MotionTrackingUtils::mat4ToCvMat(currentPose.transformMatrix, rvec, tvec);
                
                cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
                
                float error = cv::norm(projectedPoints[0] - feature.imagePoint);
                if (error > MAX_REPROJECTION_ERROR) {
                    feature.confidence *= 0.8f;
                    if (feature.confidence < 0.3f) {
                        feature.isValid = false;
                    }
                } else {
                    feature.confidence = std::min(1.0f, feature.confidence + 0.1f);
                }
            }
        }
    }
}

void ARMotionTracker::detectNewFeatures(const cv::Mat& frame) {
    // Create mask to avoid detecting features near existing ones
    cv::Mat mask = cv::Mat::ones(frame.size(), CV_8UC1) * 255;
    
    for (const auto& feature : trackedFeatures) {
        if (feature.isValid) {
            cv::circle(mask, feature.imagePoint, 20, cv::Scalar(0), -1);
        }
    }
    
    // Detect new features
    std::vector<cv::KeyPoint> keypoints;
    orbDetector->detect(frame, keypoints, mask);
    
    // Add new features
    int featuresAdded = 0;
    for (const auto& kp : keypoints) {
        if (static_cast<int>(trackedFeatures.size()) >= maxFeatures) break;
        
        TrackedFeature feature;
        feature.imagePoint = kp.pt;
        feature.worldPoint = Vec3(0, 0, 0); // Will be triangulated
        feature.id = nextFeatureId++;
        feature.age = 0;
        feature.isValid = true;
        feature.confidence = 1.0f;
        
        trackedFeatures.push_back(feature);
        featuresAdded++;
    }
    
    if (featuresAdded > 0) {
        std::cout << "Added " << featuresAdded << " new features" << std::endl;
    }
}

void ARMotionTracker::cullBadFeatures() {
    auto it = std::remove_if(trackedFeatures.begin(), trackedFeatures.end(),
        [](const TrackedFeature& f) {
            return !f.isValid || f.age > MAX_FEATURE_AGE || f.confidence < 0.2f;
        });
    
    size_t removedCount = std::distance(it, trackedFeatures.end());
    trackedFeatures.erase(it, trackedFeatures.end());
    
    if (removedCount > 0) {
        std::cout << "Removed " << removedCount << " bad features" << std::endl;
    }
}

void ARMotionTracker::reset() {
    trackingState = TrackingState::NOT_TRACKING;
    trackedFeatures.clear();
    poseHistory.clear();
    currentPose = Pose6DOF();
    isInitialized = false;
    initializationFrameCount = 0;
    nextFeatureId = 0;
    frameCount = 0;
    averageTrackingTime = 0.0f;
}

// Utility functions implementation
namespace MotionTrackingUtils {

Mat4 cvMatToMat4(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    
    Mat4 transform;
    
    // Rotation part
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            transform.m[j * 4 + i] = R.at<double>(i, j);
        }
    }
    
    // Translation part
    transform.m[12] = tvec.at<double>(0);
    transform.m[13] = tvec.at<double>(1);
    transform.m[14] = tvec.at<double>(2);
    
    // Bottom row
    transform.m[3] = transform.m[7] = transform.m[11] = 0.0f;
    transform.m[15] = 1.0f;
    
    return transform;
}

void mat4ToCvMat(const Mat4& transform, cv::Mat& rvec, cv::Mat& tvec) {
    // Extract rotation matrix
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R.at<double>(i, j) = transform.m[j * 4 + i];
        }
    }
    
    // Convert to rotation vector
    cv::Rodrigues(R, rvec);
    
    // Extract translation
    tvec = cv::Mat::zeros(3, 1, CV_64F);
    tvec.at<double>(0) = transform.m[12];
    tvec.at<double>(1) = transform.m[13];
    tvec.at<double>(2) = transform.m[14];
}

float computeParallax(const cv::Point2f& pt1, const cv::Point2f& pt2) {
    return cv::norm(pt1 - pt2);
}

bool isPointInFront(const cv::Point3f& point, const cv::Mat& R, const cv::Mat& t) {
    cv::Mat worldPoint = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
    cv::Mat cameraPoint = R * worldPoint + t;
    return cameraPoint.at<double>(2) > 0;
}

void filterOutliers(std::vector<cv::Point2f>& points1, 
                   std::vector<cv::Point2f>& points2,
                   const cv::Mat& fundamentalMatrix) {
    std::vector<cv::Point2f> filteredPoints1, filteredPoints2;
    
    for (size_t i = 0; i < points1.size(); ++i) {
        cv::Mat pt1 = (cv::Mat_<double>(3, 1) << points1[i].x, points1[i].y, 1);
        cv::Mat pt2 = (cv::Mat_<double>(3, 1) << points2[i].x, points2[i].y, 1);
        
        cv::Mat errorMat = pt2.t() * fundamentalMatrix * pt1;
        double error = std::abs(errorMat.at<double>(0));
        
        if (error < 1.0) { // Threshold for inlier
            filteredPoints1.push_back(points1[i]);
            filteredPoints2.push_back(points2[i]);
        }
    }
    
    points1 = filteredPoints1;
    points2 = filteredPoints2;
}

} // namespace MotionTrackingUtils
