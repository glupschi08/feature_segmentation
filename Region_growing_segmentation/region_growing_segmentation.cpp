#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <random>
#include <cxxopts.hpp>
#include <pcl/filters/radius_outlier_removal.h>



int
main (int argc, char** argv)
{
   // Inputs
    std::string path_str;
    bool filter_flag=false;
    double test_double1=0, test_double2=0, test_double3=0, test_double4=0;
    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
        ("help", "Print help")
        ("input_file", "Input MBES pings", cxxopts::value(path_str))
        ("t_d1", "An test double", cxxopts::value<double>(test_double1))
        ("t_d2", "An test double", cxxopts::value<double>(test_double2))
        ("t_d3", "An test double", cxxopts::value<double>(test_double3))
        ("t_d4", "An test double", cxxopts::value<double>(test_double4))
        ("f,filter", "set to false -> no filtering", cxxopts::value<bool>(filter_flag));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
if ( pcl::io::loadPCDFile <pcl::PointXYZ> (path_str, *cloud) == -1) 
//if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("../../data/keypoints__visualize_temp_4.pcd", *cloud) == -1)
  //if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("../../data/region_growing_tutorial.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  if(filter_flag==true){
    std::cout << "Start Filtering." << std::endl; 
    std::cerr << "Cloud before RadiusOutlierRemoval: " <<cloud->size()<< std::endl;
    //PointCloudT::Ptr cloud_ptr (new PointCloudT);
    //*cloud_ptr = submap_i.submap_pcl_;
   // Create the filtering object for RadiusOutlierRemoval
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    std::cerr << "setRadiusSearch: " <<test_double1<< std::endl;
    std::cerr << "setMinNeighborsInRadius: " <<test_double2<< std::endl;
    outrem.setRadiusSearch(test_double1);//0.8
    outrem.setMinNeighborsInRadius (test_double2);//2
    outrem.setInputCloud(cloud);
    outrem.filter (*cloud);
    std::cerr << "Cloud after RadiusOutlierRemoval: " <<cloud->size()<< std::endl;
  }


  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

/*
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

*/
    std::cerr << "Filter: setMinClusterSize: " <<test_double3<< std::endl;
    std::cerr << "Filter: setNumberOfNeighbours: " <<test_double4<< std::endl;
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (test_double3);//50
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (test_double4);//30
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  for(int counter=0;counter<=clusters.size ();counter++){
	std::cout << "cluster "<< counter<<" has " << clusters[counter].indices.size () << " points." << std::endl;
  }
  
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  //while (counter < clusters[0].indices.size ())
  //{
    //std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    //if (counter % 10 == 0)
      //std::cout << std::endl;
  //}
  std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}

