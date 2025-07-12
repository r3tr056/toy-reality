#include "ar_plane_detection.h"
#include <algorithm>
#include <random>
#include <iostream>
#include <map>

// Static constants
const float ARPlaneDetector::MERGE_DISTANCE_THRESHOLD = 0.1f;
const float ARPlaneDetector::MERGE_NORMAL_THRESHOLD = 0.95f; // cos(18 degrees)
const int ARPlaneDetector::MAX_PLANES = 10;

ARPlaneDetector::ARPlaneDetector()
    : imageWidth(640), imageHeight(480)
    , minPlaneSize(0.2f)
    , maxPlaneDistance(5.0f)
    , normalTolerance(0.1f)
    , ransacIterations(1000)
    , ransacThreshold(0.05f)
    , minInliers(20)
    , nextPlaneId(0)
{
}

ARPlaneDetector::~ARPlaneDetector() {
}

bool ARPlaneDetector::initialize(int width, int height) {
    imageWidth = width;
    imageHeight = height;
    
    detectedPlanes.clear();
    nextPlaneId = 0;
    
    std::cout << "Plane detector initialized for " << width << "x" << height << std::endl;
    return true;
}

std::vector<DetectedPlane> ARPlaneDetector::detectPlanes(const cv::Mat& /*frame*/, 
                                                        const std::vector<TrackedFeature>& features,
                                                        const Pose6DOF& /*currentPose*/) {
    // Extract 3D point cloud from tracked features
    std::vector<Vec3> pointCloud = extractPointCloud(features);
    
    if (static_cast<int>(pointCloud.size()) < minInliers) {
        return detectedPlanes;
    }
    
    // Filter points by distance
    pointCloud = PlaneDetectionUtils::filterPointsByDistance(pointCloud, maxPlaneDistance);
    
    // Downsample for performance
    pointCloud = PlaneDetectionUtils::downsamplePoints(pointCloud, 0.05f);
    
    // Find planes using RANSAC
    std::vector<DetectedPlane> newPlanes = findPlanesRANSAC(pointCloud);
    
    // Merge with existing planes
    for (auto& newPlane : newPlanes) {
        bool merged = false;
        for (auto& existingPlane : detectedPlanes) {
            if (existingPlane.isValid && mergePlanes(existingPlane, newPlane)) {
                merged = true;
                break;
            }
        }
        
        if (!merged && detectedPlanes.size() < MAX_PLANES) {
            newPlane.id = nextPlaneId++;
            detectedPlanes.push_back(newPlane);
            std::cout << "Detected new plane " << newPlane.id 
                      << " at (" << newPlane.center.x << ", " 
                      << newPlane.center.y << ", " << newPlane.center.z << ")" << std::endl;
        }
    }
    
    // Remove invalid planes
    auto it = std::remove_if(detectedPlanes.begin(), detectedPlanes.end(),
        [](const DetectedPlane& plane) { return !plane.isValid || plane.confidence < 0.3f; });
    detectedPlanes.erase(it, detectedPlanes.end());
    
    return detectedPlanes;
}

std::vector<Vec3> ARPlaneDetector::extractPointCloud(const std::vector<TrackedFeature>& features) {
    std::vector<Vec3> pointCloud;
    
    for (const auto& feature : features) {
        if (feature.isValid && feature.age > 5 && feature.confidence > 0.5f) {
            // Only use features that have been triangulated (have valid 3D position)
            if (feature.worldPoint.x != 0 || feature.worldPoint.y != 0 || feature.worldPoint.z != 0) {
                pointCloud.push_back(feature.worldPoint);
            }
        }
    }
    
    return pointCloud;
}

std::vector<DetectedPlane> ARPlaneDetector::findPlanesRANSAC(const std::vector<Vec3>& points) {
    std::vector<DetectedPlane> planes;
    std::vector<Vec3> remainingPoints = points;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Find up to 3 planes
    for (int planeCount = 0; planeCount < 3 && static_cast<int>(remainingPoints.size()) >= minInliers; ++planeCount) {
        DetectedPlane bestPlane;
        std::vector<Vec3> bestInliers;
        int bestInlierCount = 0;
        
        // RANSAC iterations
        for (int iter = 0; iter < ransacIterations && remainingPoints.size() >= 3; ++iter) {
            // Sample 3 random points
            std::uniform_int_distribution<> dis(0, remainingPoints.size() - 1);
            int idx1 = dis(gen);
            int idx2 = dis(gen);
            int idx3 = dis(gen);
            
            if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) continue;
            
            Vec3 p1 = remainingPoints[idx1];
            Vec3 p2 = remainingPoints[idx2];
            Vec3 p3 = remainingPoints[idx3];
            
            // Compute plane normal and validate
            Vec3 normal = PlaneDetectionUtils::computePlaneNormal(p1, p2, p3);
            if (normal.norm() < 0.1f) continue; // Degenerate case
            
            normal = normal.normalized();
            
            // Count inliers
            std::vector<Vec3> inliers;
            for (const auto& point : remainingPoints) {
                float distance = std::abs(PlaneDetectionUtils::computePlaneDistance(point, normal, p1));
                if (distance < ransacThreshold) {
                    inliers.push_back(point);
                }
            }
            
            // Check if this is the best plane so far
            if (static_cast<int>(inliers.size()) > bestInlierCount && static_cast<int>(inliers.size()) >= minInliers) {
                bestInlierCount = inliers.size();
                bestInliers = inliers;
                
                // Fit plane to all inliers
                bestPlane = fitPlane(inliers);
                bestPlane.normal = normal;
            }
        }
        
        // Add plane if valid
        if (bestInlierCount >= minInliers && PlaneDetectionUtils::isValidPlane(bestPlane, bestInlierCount)) {
            updatePlaneConfidence(bestPlane, bestInliers);
            bestPlane.boundaryPoints = computePlaneBoundary(bestInliers, bestPlane);
            bestPlane.isValid = true;
            
            planes.push_back(bestPlane);
            
            // Remove inliers from remaining points
            std::vector<Vec3> newRemainingPoints;
            for (const auto& point : remainingPoints) {
                float distance = std::abs(PlaneDetectionUtils::computePlaneDistance(
                    point, bestPlane.normal, bestPlane.center));
                if (distance >= ransacThreshold) {
                    newRemainingPoints.push_back(point);
                }
            }
            remainingPoints = newRemainingPoints;
            
            std::cout << "RANSAC found plane with " << bestInlierCount << " inliers" << std::endl;
        } else {
            break; // No more valid planes
        }
    }
    
    return planes;
}

DetectedPlane ARPlaneDetector::fitPlane(const std::vector<Vec3>& inlierPoints) {
    DetectedPlane plane;
    
    if (inlierPoints.empty()) return plane;
    
    // Compute centroid
    Vec3 centroid(0, 0, 0);
    for (const auto& point : inlierPoints) {
        centroid = centroid + point;
    }
    centroid = centroid * (1.0f / inlierPoints.size());
    plane.center = centroid;
    
    // Compute size as maximum distance from center
    float maxDist = 0.0f;
    for (const auto& point : inlierPoints) {
        float dist = (point - centroid).norm();
        if (dist > maxDist) {
            maxDist = dist;
        }
    }
    plane.size = maxDist * 2.0f; // Diameter
    
    return plane;
}

bool ARPlaneDetector::mergePlanes(DetectedPlane& plane1, const DetectedPlane& plane2) {
    // Check if planes are close enough to merge
    float centerDistance = (plane1.center - plane2.center).norm();
    float normalSimilarity = plane1.normal.dot(plane2.normal);
    
    if (centerDistance < MERGE_DISTANCE_THRESHOLD && normalSimilarity > MERGE_NORMAL_THRESHOLD) {
        // Merge planes by averaging properties
        float weight1 = plane1.confidence;
        float weight2 = plane2.confidence;
        float totalWeight = weight1 + weight2;
        
        if (totalWeight > 0) {
            plane1.center = (plane1.center * weight1 + plane2.center * weight2) * (1.0f / totalWeight);
            plane1.normal = (plane1.normal * weight1 + plane2.normal * weight2).normalized();
            plane1.size = std::max(plane1.size, plane2.size);
            plane1.confidence = std::min(1.0f, plane1.confidence + plane2.confidence * 0.1f);
        }
        
        return true;
    }
    
    return false;
}

void ARPlaneDetector::updatePlaneConfidence(DetectedPlane& plane, const std::vector<Vec3>& supportingPoints) {
    // Base confidence on number of supporting points and their distribution
    float pointDensity = supportingPoints.size() / std::max(1.0f, plane.size * plane.size);
    float roughness = PlaneDetectionUtils::computePlaneRoughness(supportingPoints, plane);
    
    plane.confidence = std::min(1.0f, pointDensity * 0.1f) * (1.0f - roughness);
    
    // Boost confidence for horizontal planes (likely floors/tables)
    if (isHorizontalPlane(plane.normal)) {
        plane.confidence *= 1.2f;
    }
}

float ARPlaneDetector::distanceToPlane(const Vec3& point, const DetectedPlane& plane) {
    return PlaneDetectionUtils::computePlaneDistance(point, plane.normal, plane.center);
}

bool ARPlaneDetector::isHorizontalPlane(const Vec3& normal) {
    return std::abs(normal.dot(Vec3(0, 1, 0))) > 0.8f; // Within ~37 degrees of vertical
}

bool ARPlaneDetector::isVerticalPlane(const Vec3& normal) {
    return std::abs(normal.dot(Vec3(0, 1, 0))) < 0.3f; // Within ~17 degrees of horizontal
}

std::vector<Vec3> ARPlaneDetector::computePlaneBoundary(const std::vector<Vec3>& points, const DetectedPlane& plane) {
    // Simplified boundary computation - just return corner points
    std::vector<Vec3> boundary;
    
    if (points.size() < 4) return boundary;
    
    // Project points onto plane and find convex hull (simplified)
    Vec3 minPoint = points[0];
    Vec3 maxPoint = points[0];
    
    for (const auto& point : points) {
        if (point.x < minPoint.x) minPoint.x = point.x;
        if (point.y < minPoint.y) minPoint.y = point.y;
        if (point.z < minPoint.z) minPoint.z = point.z;
        if (point.x > maxPoint.x) maxPoint.x = point.x;
        if (point.y > maxPoint.y) maxPoint.y = point.y;
        if (point.z > maxPoint.z) maxPoint.z = point.z;
    }
    
    // Create 4 corner points (simplified rectangular boundary)
    boundary.push_back(Vec3(minPoint.x, plane.center.y, minPoint.z));
    boundary.push_back(Vec3(maxPoint.x, plane.center.y, minPoint.z));
    boundary.push_back(Vec3(maxPoint.x, plane.center.y, maxPoint.z));
    boundary.push_back(Vec3(minPoint.x, plane.center.y, maxPoint.z));
    
    return boundary;
}

// Utility functions implementation
namespace PlaneDetectionUtils {

Vec3 computePlaneNormal(const Vec3& p1, const Vec3& p2, const Vec3& p3) {
    Vec3 v1 = p2 - p1;
    Vec3 v2 = p3 - p1;
    return v1.cross(v2);
}

float computePlaneDistance(const Vec3& point, const Vec3& normal, const Vec3& planePoint) {
    return normal.dot(point - planePoint);
}

std::vector<Vec3> filterPointsByDistance(const std::vector<Vec3>& points, float maxDistance) {
    std::vector<Vec3> filtered;
    
    for (const auto& point : points) {
        float distance = point.norm(); // Distance from origin
        if (distance <= maxDistance) {
            filtered.push_back(point);
        }
    }
    
    return filtered;
}

std::vector<Vec3> downsamplePoints(const std::vector<Vec3>& points, float gridSize) {
    if (points.empty()) return points;
    
    // Simple grid-based downsampling
    std::map<std::tuple<int, int, int>, Vec3> grid;
    
    for (const auto& point : points) {
        int gx = static_cast<int>(point.x / gridSize);
        int gy = static_cast<int>(point.y / gridSize);
        int gz = static_cast<int>(point.z / gridSize);
        
        auto key = std::make_tuple(gx, gy, gz);
        if (grid.find(key) == grid.end()) {
            grid[key] = point;
        }
    }
    
    std::vector<Vec3> downsampled;
    for (const auto& pair : grid) {
        downsampled.push_back(pair.second);
    }
    
    return downsampled;
}

bool isValidPlane(const DetectedPlane& plane, int minPoints) {
    return plane.size > 0.1f && minPoints >= 10 && plane.confidence > 0.1f;
}

float computePlaneRoughness(const std::vector<Vec3>& points, const DetectedPlane& plane) {
    if (points.empty()) return 1.0f;
    
    float totalDeviation = 0.0f;
    for (const auto& point : points) {
        float distance = std::abs(computePlaneDistance(point, plane.normal, plane.center));
        totalDeviation += distance;
    }
    
    return totalDeviation / points.size();
}

} // namespace PlaneDetectionUtils
