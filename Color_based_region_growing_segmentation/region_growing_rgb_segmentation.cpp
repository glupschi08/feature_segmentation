#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <chrono>
#include <random>
#include <cxxopts.hpp>




std::tuple<uint8_t, uint8_t, uint8_t> jet(double x){
    const double rone = 0.8;
    const double gone = 1.0;
    const double bone = 1.0;
    double r, g, b;

    x = (x < 0 ? 0 : (x > 1 ? 1 : x));

    if (x < 1. / 8.) {
        r = 0;
        g = 0;
        b = bone * (0.5 + (x) / (1. / 8.) * 0.5);
    } else if (x < 3. / 8.) {
        r = 0;
        g = gone * (x - 1. / 8.) / (3. / 8. - 1. / 8.);
        b = bone;
    } else if (x < 5. / 8.) {
        r = rone * (x - 3. / 8.) / (5. / 8. - 3. / 8.);
        g = gone;
        b = (bone - (x - 3. / 8.) / (5. / 8. - 3. / 8.));
    } else if (x < 7. / 8.) {
        r = rone;
        g = (gone - (x - 5. / 8.) / (7. / 8. - 5. / 8.));
        b = 0;
    } else {
        r = (rone - (x - 7. / 8.) / (1. - 7. / 8.) * 0.5);
        g = 0;
        b = 0;
    }
    return std::make_tuple(uint8_t(255.*r), uint8_t(255.*g), uint8_t(255.*b));
}


//does the prescalling for jet -> maps z to [0-1]:[1-0] in the area between 0 and threshold
//e.g. points along a linear line in z direction would get be: blue, green, yellow, red, yellow, green, blue, green,...
std::tuple<uint8_t, uint8_t, uint8_t> stacked_jet(double z, double threshold){
    pcl::PointXYZRGB pointrgb;
    std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
    double r, g, b, val;
    if(z<=0){
        while(z<0){
            z+=threshold;
        }
    }else{
        while(z>threshold){
            z-=threshold;
        }
    }
    if(z>threshold/2){
        z-=(threshold/2);
        val=-((z/(threshold/2))-1);
    }else{
        val=z/(threshold/2);
    }
    //std::cout << "new z: " << z  << "   val: " << val <<std::endl;
    //std::cout << "val: " << val << std::endl;
    //colors_rgb = jet(z/(threshold);
    return jet(val);
    //return std::make_tuple(uint8_t(255.*0), uint8_t(255.*0), uint8_t(255.*0));
}

//using namespace std::chrono_literals;
//using namespace std::literals::chrono_literals
int
main (int argc, char** argv)
{

   // Inputs
    std::string path_str;
    double jet_stacking_threshold=30.;
    int filter_flag=0;
    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
        ("help", "Print help")
            ("i,staked_height", "the height for the stacked color", cxxopts::value<double>(jet_stacking_threshold)->default_value("30.0"))
            ("f,filter", "set to 1 for filtering", cxxopts::value<int>(filter_flag)->default_value("0"))

            ("input_file", "Input MBES pings", cxxopts::value(path_str));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (path_str, *cloud) == -1 )  
//if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("../../data/region_growing_rgb_tutorial.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
    if (result.count("i") ){
        std::cout << "Stacked height threshold: " << jet_stacking_threshold << std::endl;
    }


    std::cout << "start jet function." << std::endl;
    for (std::size_t i = 0; i < cloud->points.size (); ++i){
        pcl::PointXYZRGB pointrgb;
        std::tuple<uint8_t, uint8_t, uint8_t> colors_rgb;
        colors_rgb = stacked_jet( cloud->points[i].z, jet_stacking_threshold);

        std::uint32_t rgb = (static_cast<std::uint32_t>(std::get<0>(colors_rgb)) << 16 |
                             static_cast<std::uint32_t>(std::get<1>(colors_rgb)) << 8 |
                             static_cast<std::uint32_t>(std::get<2>(colors_rgb)));
        pointrgb.rgb = *reinterpret_cast<float*>(&rgb);
        cloud->points[i].r = pointrgb.r;
        cloud->points[i].g = pointrgb.g;
        cloud->points[i].b = pointrgb.b;
    }

    /*
    std::cout << "start Filtering." << std::endl;
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 100.0); //0.0 1.0
  pass.filter (*indices);
*/


//do some filtering on the cloud to remove outliers
    if(filter_flag){
        // Create the filtering object for RadiusOutlierRemoval
        //PointCloudT::Ptr cloud_ptr (new PointCloudT);
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;

        //pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        //std::cerr << "setRadiusSearch: " <<test_double1<< std::endl;
        //std::cerr << "setMinNeighborsInRadius: " <<test_double2<< std::endl;
        outrem.setRadiusSearch(5.);//good 5 and r = 3//0.8
        outrem.setMinNeighborsInRadius (4);//2
        std::cerr << "Cloud after StatisticalOutlierRemoval: " <<cloud->size()<< std::endl;
        outrem.setInputCloud(cloud);
        outrem.filter (*cloud);
        std::cerr << "Cloud after RadiusOutlierRemoval: " <<cloud->size()<< std::endl;
        //cloud = *cloud_ptr;
    }


    //std::cout << "cloud size after filtering: " <<indices->size()<< std::endl;
    std::cout << "start RegionGrowingRGB function." << std::endl;

  double MinClusterSize=100;//600
  float RegionColorThreshold=5;//5
  float PointColorThreshold=6;//6
  float DistanceThreshold=2;//10

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  //reg.setIndices (cloud);
  //reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (DistanceThreshold);
  reg.setPointColorThreshold (PointColorThreshold);
  reg.setRegionColorThreshold (RegionColorThreshold);
  reg.setMinClusterSize (MinClusterSize);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

    std::cout << "start visu." << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud (colored_cloud);
  while (!viewer.wasStopped ())
  {
    //sstd::this_thread::sleep_for(100us);
  }

  return (0);
}
