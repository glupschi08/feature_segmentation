/**
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 */
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <random>
#include <pcl/features/don.h>
#include <cxxopts.hpp>



using namespace pcl;
using namespace std;



// Colors to display the generated clusters
float colors[] = {
        255, 0,   0,   // red 		1
        0,   255, 0,   // green		2
        0,   0,   255, // blue		3
        255, 255, 0,   // yellow		4
        0,   255, 255, // light blue	5
        255, 0,   255, // magenta     6
        255, 255, 255, // white		7
        255, 128, 0,   // orange		8
        255, 153, 255, // pink		9
        51,  153, 255, //			10
        153, 102, 51,  //			11
        128, 51,  153, //			12
        153, 153, 51,  //			13
        163, 38,  51,  //			14
        204, 153, 102, //		15
        204, 224, 255, //		16
        128, 179, 255, //		17
        206, 255, 0,   //			18
        255, 204, 204, //			19
        204, 255, 153, //			20

}; // 20x3=60 color elements




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
    return jet(val);
}



boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals){
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    return (viewer);
}





int main (int argc, char *argv[])
{
  ///The smallest scale to use in the DoN filter.
  double scale1;
  ///The largest scale to use in the DoN filter.
  double scale2;
  ///The minimum DoN magnitude to threshold by
  double threshold;
  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius;

  double jet_stacking_threshold=30;


    // Inputs
    std::string path_str;
    int filter_flag=0;

    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
            ("help", "Print help")
            ("i,staked_height", "the height for the stacked color", cxxopts::value<double>(jet_stacking_threshold)->default_value("30.0"))
            ("f,filter", "set to 1 for filtering", cxxopts::value<int>(filter_flag)->default_value("0"))

            ("a,scale1", "Param foo", cxxopts::value<double>(scale1)->default_value("100.0"))
            ("b,scale2", "Param foo", cxxopts::value<double>(scale2)->default_value("5.0"))
            ("c,threshold", "Param foo", cxxopts::value<double>(threshold)->default_value("6.0"))
            ("d,segradius", "Param foo", cxxopts::value<double>(segradius)->default_value("2.0"))

            ("input_file", "Input MBES pings", cxxopts::value(path_str));

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help({ "", "Group" }) << endl;
        exit(0);
    }

  /*
  if (argc < 6)
  {
    std::cerr << "usage: " << argv[0] << " inputfile smallscale largescale threshold segradius" << std::endl;
    exit (EXIT_FAILURE);
  }

  /// the file to read from.
  string infile = argv[1];
  /// small scale
  istringstream (argv[2]) >> scale1;
  /// large scale
  istringstream (argv[3]) >> scale2;
  istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
  istringstream (argv[5]) >> segradius;   // threshold for radius segmentation
*/







  // Load cloud in blob format
  pcl::PCLPointCloud2 blob;
  //pcl::io::loadPCDFile (infile.c_str (), blob);
    pcl::io::loadPCDFile (path_str, blob);

  pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  pcl::fromPCLPointCloud2 (blob, *cloud);

    std::cout << "Starting size of Pointcloud: " << cloud->points.size () << " data points." << std::endl;

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




  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1 >= scale2)
  {
    std::cerr << "Error: Large scale must be > small scale!" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  std::cout << "Calculating normals for scale..." << scale1 << std::endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  std::cout << "Calculating normals for scale..." << scale2 << std::endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud (*cloud, *doncloud);

  std::cout << "Calculating DoN... " << std::endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                               new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;

  pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CLUSTERS"));
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::visualization::PCLVisualizer corresp_viewer("Correspondences Viewer");
    corresp_viewer.setBackgroundColor(0, 0, 0);
    //viewer->setBackgroundColor(0, 0, 0);
    pcl::PointCloud<PointXYZ> cluster;
  int j = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
      double counter=0;
    pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster_don->points.push_back (doncloud->points[*pit]);

      //cluster[counter].x=(doncloud->points[*pit].x);
      //cluster[counter].y=(doncloud->points[*pit].y);
      //cluster[counter].z=(doncloud->points[*pit].z);
      counter++;
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;
    std::random_device seeder;
    std::ranlux48 gen(seeder());
    std::uniform_int_distribution<int> uniform_0_255(0, 255);

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    std::string nameId = "cluster_";
    nameId += std::to_string(j);
      std::cout << nameId << std::endl;
      //viewer=normalsVis(cloud, );
      //viewer->addPointCloudNormals<pcl::PointXYZNormal,  pcl::PointXYZNormal>  (cloud_cluster_don,  cloud_cluster_don,  10,  0.5,  "normals");
      //viewer->addPointCloud<pcl::PointNormal>(cloud_cluster_don->points, "foo", 1);
      uint8_t r;
      uint8_t g;
      uint8_t b;

      if (j < 60) {

          r = (uint8_t)colors[j];
          g = (uint8_t)colors[j + 1];
          b = (uint8_t)colors[j + 2];

      } else {

          r = (uint8_t)uniform_0_255(gen);
          g = (uint8_t)uniform_0_255(gen);
          b = (uint8_t)uniform_0_255(gen);
      }

      corresp_viewer.addPointCloudNormals<PointNormal>(cloud_cluster_don,1,0.5, nameId);
      corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, nameId);
      //corresp_viewer.addPointCloud(cluster, nameId, 0);
      //corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Src cloud");


      //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, nameId);

    /*
    //Save cluster
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    stringstream ss;
    ss << "don_cluster_" << j << ".pcd";
    writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);

     */
  }

    while (!corresp_viewer.wasStopped()) {
        corresp_viewer.spin();
    }
}

