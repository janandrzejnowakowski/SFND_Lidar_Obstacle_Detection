#include "ransac.h"

inline std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;

    int max_inliers = 0;
    double a, b, c, d, distance, denominator;
    double total_dist;

    std::random_device random_device;
    std::mt19937 engine{random_device()};
    std::uniform_int_distribution<int> dist(0, cloud->points.size() - 1);

    while (maxIterations--) {
        std::vector<pcl::PointXYZ> p {};
        std::unordered_set<int> currentInliers;

        while (p.size() < 3) {
            int i = dist(engine);
            p.push_back(cloud->points[i]);
            currentInliers.insert(i);
        }

        a = (p[1].y - p[0].y)*(p[2].z - p[0].z) - (p[1].z - p[0].z)*(p[2].y - p[0].y);
        b = (p[1].z - p[0].z)*(p[2].x - p[0].x) - (p[1].x - p[0].x)*(p[2].z - p[0].z);
        c = (p[1].x - p[0].x)*(p[2].y - p[0].y) - (p[1].y - p[0].y)*(p[2].x - p[0].x);
        d = -(a*p[0].x + b*p[0].y + c*p[0].z);
        denominator = std::sqrt(std::pow(a, 2) + std::pow(b, 2) + std::pow(c, 2));

        for (int j = 0; j < cloud->points.size(); ++j) {
            pcl::PointXYZ curr_point = cloud->points[j];
            distance = std::abs(a * curr_point.x + b * curr_point.y + c * curr_point.z + d) / denominator;
            total_dist += distance;
            if (distance < distanceTol) {
                currentInliers.insert(j);
            }
        }
        if (currentInliers.size() > max_inliers) {
            max_inliers = currentInliers.size();
            inliersResult = currentInliers;
        }
    }
    return inliersResult;
}