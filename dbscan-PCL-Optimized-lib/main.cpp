/*
DBSCAN, or Density-Based Spatial Clustering of Applications with Noise,
is an unsupervised machine learning algorithm. Unsupervised machine learning
algorithms are used to classify unlabeled data.

DBSCAN is particularly well suited for problems which require:
1. Minimal domain knowledge to determine the input parameters (i.e. K in k-means and Dmin in hierarchical clustering)
2. Discovery of clusters with arbitrary shapes
3. Good efficiency on large databases

As is the case in most machine learning algorithms, the model’s behaviour is dictated by several parameters.

1. eps: Two points are considered neighbors if the distance between the two points is below the threshold epsilon.
2. min_samples: The minimum number of neighbors a given point should have in order to be classified as a core point.
                It’s important to note that the point itself is included in the minimum number of samples.

The algorithm works by computing the distance between every point and all other points. We then place the points into
one of three categories.
1. Core point: A point with at least min_samples points whose distance with respect to the point is below the threshold
defined by epsilon.
2. Border point: A point that isn’t in close proximity to at least min_samples points but is close enough to one or more
core point. Border points are included in the cluster of the closest core point.
3. Noise point: Points that aren’t close enough to core points to be considered border points. Noise points are ignored.
That is to say, they aren’t part of any cluster.
*/

#define _CRT_SECURE_NO_WARNINGS
#include <iostream>

#include "include/OctreeGenerator.h"
#include "include/dbScan.h"
#include <math.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

//tryf ro filter
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/common/common.h>
#include <cxxopts.hpp>


#include "keypointcluster.h"
#include <queue>
#include <cxxopts.hpp>



int main(int argc, char *argv[]) {
    ///The smallest scale to use in the DoN filter.
    int octree_resolution, minPtsAux, minPts;
    float eps;
    // Inputs
    std::string path_str;
    int filter_flag=0;
    int show;
    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
            ("help", "Print help")
            ("f,filter", "set to 1 for filtering", cxxopts::value<int>(filter_flag)->default_value("0"))
            ("a,octree_resolution", "Param foo", cxxopts::value<int>(octree_resolution)->default_value("100"))
            ("b,eps", "Param foo", cxxopts::value<float>(eps)->default_value("5.0"))
            ("c,minPtsAux", "Param foo", cxxopts::value<int>(minPtsAux)->default_value("6"))
            ("d,minPts", "Param foo", cxxopts::value<int>(minPts)->default_value("2"))
            ("s,show", "show visualization", cxxopts::value<int>(show)->default_value("0"))
            ("input_file", "Input pcd file", cxxopts::value(path_str));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }

    std::cout << "\n*************************************" << std::endl;
    std::cout << "*** DBSCAN Cluster Segmentation *** " << std::endl;
    std::cout << "*************************************" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_str, *cloud) == -1 )    {
        PCL_ERROR("Couldn't read src or tgt file");
        return -1;
    }

    //do some filtering on the cloud to remove outliers
    if(filter_flag){
        // Create the filtering object for RadiusOutlierRemoval
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        //std::cerr << "setRadiusSearch: " <<test_double1<< std::endl;
        //std::cerr << "setMinNeighborsInRadius: " <<test_double2<< std::endl;
        outrem.setRadiusSearch(5.);//good 5 and r = 3//0.8
        outrem.setMinNeighborsInRadius (4);//2
        std::cerr << "Cloud after StatisticalOutlierRemoval: " <<cloud->size()<< std::endl;
        outrem.setInputCloud(cloud);
        outrem.filter (*cloud);
        std::cerr << "Cloud after RadiusOutlierRemoval: " <<cloud->size()<< std::endl;
    }

    queue<KeypointCluster> Keypoint_Cluster_Queue;
    Keypoint_Cluster_Queue = dbscan_classification(octree_resolution, eps, minPtsAux, minPts, cloud, show);

    std::cout << "-----back in main-----" << std::endl;
    std::cout << "Size of queue = " << Keypoint_Cluster_Queue.size() << endl;
    if(show){
        visualize_clusters(Keypoint_Cluster_Queue, cloud);
    }
    return 0;
}
