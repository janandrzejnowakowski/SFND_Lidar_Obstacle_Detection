#ifndef RANSAC_H
#define RANSAC_H

#include <unordered_set>
#include <random>
#include "ransac.cpp"

inline std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr, int, float);

#endif //RANSAC_H
