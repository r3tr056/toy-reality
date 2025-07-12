#ifndef AR_PLANE_DETECTION_H_
#define AR_PLANE_DETECTION_H_

#include "ar_la.h"
#include "ar_motion_tracking.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

using namespace toyar;

struct DetectedPlane {
    Vec3 normal;
    Vec3 center;
    float size;
    std::vector<Vec3> boundaryPoints;
    float confidence;
    int id;
    bool isValid;
    
    DetectedPlane() : normal(0, 1, 0), center(0, 0, 0), size(1.0f), 
                     confidence(0.0f), id(-1), isValid(false) {}
};

class ARPlaneDetector {
public:
    ARPlaneDetector();
    ~ARPlaneDetector();
    
    bool initialize(int imageWidth, int imageHeight);
    
    // Main plane detection function
    std::vector<DetectedPlane> detectPlanes(const cv::Mat& frame, 
                                           const std::vector<TrackedFeature>& features,
                                           const Pose6DOF& currentPose);
    
    // Get all detected planes
    std::vector<DetectedPlane> getDetectedPlanes() const { return detectedPlanes; }
    
    // Configuration
    void setMinPlaneSize(float size) { minPlaneSize = size; }
    void setMaxDistance(float distance) { maxPlaneDistance = distance; }
    void setNormalTolerance(float tolerance) { normalTolerance = tolerance; }
    
private:
    // Core detection methods
    std::vector<Vec3> extractPointCloud(const std::vector<TrackedFeature>& features);
    std::vector<DetectedPlane> findPlanesRANSAC(const std::vector<Vec3>& points);
    DetectedPlane fitPlane(const std::vector<Vec3>& inlierPoints);
    bool mergePlanes(DetectedPlane& plane1, const DetectedPlane& plane2);
    void updatePlaneConfidence(DetectedPlane& plane, const std::vector<Vec3>& supportingPoints);
    
    // Geometric utilities
    float distanceToPlane(const Vec3& point, const DetectedPlane& plane);
    bool isHorizontalPlane(const Vec3& normal);
    bool isVerticalPlane(const Vec3& normal);
    std::vector<Vec3> computePlaneBoundary(const std::vector<Vec3>& points, const DetectedPlane& plane);
    
    // Parameters
    int imageWidth, imageHeight;
    float minPlaneSize;
    float maxPlaneDistance;
    float normalTolerance;
    int ransacIterations;
    float ransacThreshold;
    int minInliers;
    
    // State
    std::vector<DetectedPlane> detectedPlanes;
    int nextPlaneId;
    
    // Constants
    static const float MERGE_DISTANCE_THRESHOLD;
    static const float MERGE_NORMAL_THRESHOLD;
    static const int MAX_PLANES;
};

// Utility functions for plane detection
namespace PlaneDetectionUtils {
    // Compute plane equation from 3 points
    Vec3 computePlaneNormal(const Vec3& p1, const Vec3& p2, const Vec3& p3);
    float computePlaneDistance(const Vec3& point, const Vec3& normal, const Vec3& planePoint);
    
    // Point cloud processing
    std::vector<Vec3> filterPointsByDistance(const std::vector<Vec3>& points, float maxDistance);
    std::vector<Vec3> downsamplePoints(const std::vector<Vec3>& points, float gridSize);
    
    // Plane validation
    bool isValidPlane(const DetectedPlane& plane, int minPoints);
    float computePlaneRoughness(const std::vector<Vec3>& points, const DetectedPlane& plane);
}

#endif // AR_PLANE_DETECTION_H_
