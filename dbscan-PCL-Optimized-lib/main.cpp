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

#include "include/HTRBasicDataStructures.h"
#include "include/OctreeGenerator.h"
#include "include/dbScan.h"
#include <boost/algorithm/algorithm.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <math.h> /* log */
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/centroid.h>

//tryf ro filter
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/common/common.h>

#include <cxxopts.hpp>

//for PCA of class
#include <pcl/common/pca.h>
#include "keypointcluster.h"
#include <queue>
#include <cxxopts.hpp>


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

bool is_number(const std::string &s) {

  std::string::const_iterator it = s.begin();
  while (it != s.end() && std::isdigit(*it))
    ++it;
  return !s.empty() && it == s.end();
  /*
    return !s.empty() && std::find_if(s.begin(),
        s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
        */
}

void readCloudFromFile(int argc, char **argv, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {

  pcl::PolygonMesh cl;
  std::vector<int> filenames;
  bool file_is_pcd = false;
  bool file_is_ply = false;
  bool file_is_txt = false;
  bool file_is_xyz = false;

  pcl::console::TicToc tt;
  pcl::console::print_highlight("Loading ");

  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
  if (filenames.size() <= 0) {
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (filenames.size() <= 0) {
      filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
      if (filenames.size() <= 0) {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".xyz");
        if (filenames.size() <= 0) {
          std::cout << "Usage: ./dbscan <input pointcloud> <octree resolution> <eps> <minPtsAux> <minPts> "
                       "<output dir> output extension = pcd(default)"
                       "or ./dbscan <input pointcloud>     <-- fast test"
                    << std::endl;
          std::cerr << "Support: ply - pcd - txt - xyz" << std::endl;
          return std::exit(-1);
        } else if (filenames.size() == 1) {
          file_is_xyz = true;
        }
      } else if (filenames.size() == 1) {
        file_is_txt = true;
      }
    } else if (filenames.size() == 1) {
      file_is_pcd = true;
    }
  } else if (filenames.size() == 1) {
    file_is_ply = true;
  } else {
  std::cout << "Usage: ./dbscan <input pointcloud> <octree resolution> <eps> <minPtsAux> <minPts> "
                       "<output dir> output extension = pcd(default)"
                       "or ./dbscan <input pointcloud>     <-- fast test"
                    << std::endl;
          std::cerr << "Support: ply - pcd - txt - xyz" << std::endl;
    return std::exit(-1);
  }

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0) {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl;
      std::cout << "Usage: ./dbscan <input pointcloud> <octree resolution> <eps> <minPtsAux> <minPts> "
                       "<output dir> output extension = pcd(default)"
                       "or ./dbscan <input pointcloud>     <-- fast test"
                    << std::endl;
          std::cerr << "Support: ply - pcd - txt - xyz" << std::endl;
      return std::exit(-1);
    }
    pcl::console::print_info("\nFound pcd file.\n");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->size());
    pcl::console::print_info(" points]\n");
  } else if (file_is_ply) {
    pcl::io::loadPLYFile(argv[filenames[0]], *cloud);
    if (cloud->points.size() <= 0 or cloud->points[0].x <= 0 and cloud->points[0].y <= 0 and cloud->points[0].z <= 0) {
      pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");
      pcl::io::loadPolygonFile(argv[filenames[0]], cl);
      pcl::fromPCLPointCloud2(cl.cloud, *cloud);
      if (cloud->points.size() <= 0 or
          cloud->points[0].x <= 0 and cloud->points[0].y <= 0 and cloud->points[0].z <= 0) {
        pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");
        pcl::PLYReader plyRead;
        plyRead.read(argv[filenames[0]], *cloud);
        if (cloud->points.size() <= 0 or
            cloud->points[0].x <= 0 and cloud->points[0].y <= 0 and cloud->points[0].z <= 0) {
          pcl::console::print_error("\nError. ply file is not compatible.\n");
          return std::exit(-1);
        }
      }
    }

    pcl::console::print_info("\nFound ply file.\n");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->size());
    pcl::console::print_info(" points]\n");

  } else if (file_is_txt) {

    std::ifstream file(argv[filenames[0]], std::ifstream::in);
    if (!file.is_open()) {
      std::cout << "Error: Could not find " << argv[filenames[0]] << std::endl;
      return std::exit(-1);
    }

    std::cout << "file opened." << std::endl;
    double x_, y_, z_;
    unsigned int r, g, b;

    while (file >> x_ >> y_ >> z_ >> r >> g >> b) {

      pcl::PointXYZRGB pt;
      pt.x = x_;
      pt.y = y_;
      pt.z = z_;

      uint8_t r_, g_, b_;
      r_ = uint8_t(r);
      g_ = uint8_t(g);
      b_ = uint8_t(b);

      uint32_t rgb_ = ((uint32_t)r_ << 16 | (uint32_t)g_ << 8 | (uint32_t)b_);
      pt.rgb = *reinterpret_cast<float *>(&rgb_);

      cloud->points.push_back(pt);
      // std::cout << "pointXYZRGB:" <<  pt << std::endl;
    }

    pcl::console::print_info("\nFound txt file.\n");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->points.size());
    pcl::console::print_info(" points]\n");

    file.close();

  } else if (file_is_xyz) {

    std::ifstream file(argv[filenames[0]]);
    if (!file.is_open()) {
      std::cout << "Error: Could not find " << argv[filenames[0]] << std::endl;
      return std::exit(-1);
    }

    std::cout << "file opened." << std::endl;
    double x_, y_, z_;

    while (file >> x_ >> y_ >> z_) {

      pcl::PointXYZRGB pt;
      pt.x = x_;
      pt.y = y_;
      pt.z = z_;

      cloud->points.push_back(pt);
      // std::cout << "pointXYZRGB:" <<  pt << std::endl;
    }

    pcl::console::print_info("\nFound xyz file.\n");
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", tt.toc());
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", cloud->points.size());
    pcl::console::print_info(" points]\n");
    file.close();
  }

  cloud->width = (int)cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  if ((int)cloud->points.size() <= 0) {
    pcl::console::print_error("\nCouldn't read file.");
    pcl::console::print_info("[");
    pcl::console::print_value("%d", cloud->points.size());
    pcl::console::print_info(" points]\n");
    return std::exit(-1);
  }
}

queue<KeypointCluster> dbscan_classification(int octreeResolution, float eps, int minPtsAux, int minPts, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int show){
//void init(int filter_flag, int octreeResolution, float eps, int minPtsAux, int minPts, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int show){

  queue<KeypointCluster> Keypoint_Cluster_Queue;
  std::vector<htr::Point3D> groupA;
  dbScanSpace::dbscan dbscan;


  /*************************************************************************************************/
  // K nearest neighbor search
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *cloud_xyz);
  pcl::PointXYZ searchPoint;
  // ... populate the cloud and the search point
  // create a kd-tree instance
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  // assign a point cloud - this builds the tree
  kdtree.setInputCloud(cloud_xyz);
  // pre-allocate the neighbor index and
  // distance vectors
  int K = 10;
  std::vector<int> pointsIdx(K);
  std::vector<float> pointsSquaredDist(K);
  // K nearest neighbor search
  kdtree.nearestKSearch(searchPoint, K, pointsIdx, pointsSquaredDist);

  std::cout << "K nearest neighbor search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if (kdtree.nearestKSearch(searchPoint, K, pointsIdx, pointsSquaredDist) > 0) {
    for (std::size_t i = 0; i < pointsIdx.size(); ++i)
      std::cout << "    " << cloud->points[pointsIdx[i]].x << " " << cloud->points[pointsIdx[i]].y << " "
                << cloud->points[pointsIdx[i]].z << " (squared distance: " << pointsSquaredDist[i] << ")" << std::endl;
  }

  std::vector<double> doubleVec(pointsSquaredDist.begin(), pointsSquaredDist.end());
  double min_val = *std::min_element(doubleVec.begin(), doubleVec.end());
  double max_val = *std::max_element(doubleVec.begin(), doubleVec.end());
  std::vector<double> doubleVec_normalized;

  for (double x : doubleVec) {
    double norm = ((x - min_val) / (max_val - min_val));
    doubleVec_normalized.push_back(norm);
  }

  std::partial_sort(doubleVec_normalized.begin(), doubleVec_normalized.begin() + 2, doubleVec_normalized.end());
  std::cout << "Sorted squared distances (normalized): \n";

  for (auto x : doubleVec_normalized)
    std::cout << x << std::endl;

  std::vector<double> doubleVec_X;
  double cont_x = 0;

  for (int x = 0; x < 300; x++) {
    doubleVec_X.push_back(cont_x);
    cont_x += 500;
  }

  // defining a plotter
  pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter;
  // adding the polynomial func1 to the plotter with [-10, 10] as the range in X axis and "y = x^2" as title
  plotter->addPlotData(doubleVec_normalized, doubleVec_X, "k square distance", vtkChart::LINE, std::vector<char>());
  plotter->spinOnce(300);

  /************************************************************************************************/
	std::cerr << "octreeResolution: " <<octreeResolution<< std::endl;
	std::cerr << "eps: " <<eps<< std::endl;
	std::cerr << "minPtsAux_: " <<minPtsAux<< std::endl;
	std::cerr << "minPts: " <<minPts<< std::endl;
    //----------------------------------------------------------------
    dbscan.init(groupA, cloud, octreeResolution, eps, minPtsAux, minPts); /*RUN DBSCAN*/
    //----------------------------------------------------------------

  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
  // dbscan.generateClusters();
  dbscan.generateClusters_fast();


  ofstream fout;
  int cont = 0;
  if (dbscan.getClusters().size() <= 0) {

    pcl::console::print_error("\nCould not generated clusters, bad parameters\n");
    std::exit(-1);
  }
    std::cout << "num of clusters:"<<  dbscan.getClusters().size()  <<"\n";

    int cluster_cnt =1;
    for (auto &cluster : dbscan.getClusters()) {
      cluster_cnt++;
      std::string str1 = "../";
      str1 += "/cloud_cluster_";
      str1 += std::to_string(cont);
      str1 += ".pcd";

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());

      // std::cout << "\nPrinting clusters..." << std::endl;
      std::cout << "cluster " << cluster_cnt << ":" << cluster.clusterPoints.size() << std::endl;
      std::cout << "cluster clustersCentroids" << cluster_cnt << ":" << cluster.centroid << std::endl;
      //std::cout << "cluster clustersCentroids" << cluster_cnt << "x :" << cluster.centroid.x << std::endl;
      pcl::PointXYZ centroit_point;
        centroit_point.x=cluster.centroid.x;
        centroit_point.y=cluster.centroid.y;
        centroit_point.z=cluster.centroid.z;
        //std::cout << "cluster clustersCentroids" << cluster_cnt << "point.x :" << point.x << std::endl;
        for (auto &point : cluster.clusterPoints) {
            pcl::PointXYZRGB pt;
            pt.x = point.x;
            pt.y = point.y;
            pt.z = point.z;
            pt.r = point.r;
            pt.g = point.g;
            pt.b = point.b;
            cloud_cluster_pcd->points.push_back(pt);
        }

      pcl::PointXYZRGB minPt, maxPt;
      pcl::getMinMax3D (*cloud_cluster_pcd, minPt, maxPt);
      std::cout << "Max x: " << maxPt.x << std::endl;
      std::cout << "Max y: " << maxPt.y << std::endl;
      std::cout << "Max z: " << maxPt.z << std::endl;
      std::cout << "Min x: " << minPt.x << std::endl;
      std::cout << "Min y: " << minPt.y << std::endl;
      std::cout << "Min z: " << minPt.z << std::endl;

      //Source: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
      // Compute principal directions
      Eigen::Vector4f pcaCentroid;
      pcl::compute3DCentroid(*cloud_cluster_pcd, pcaCentroid);
      Eigen::Matrix3f covariance;
      computeCovarianceMatrixNormalized(*cloud_cluster_pcd, pcaCentroid, covariance);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
      Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
      eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
        // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PCA<pcl::PointXYZRGB> pca;
      pca.setInputCloud(cloud_cluster_pcd);
      pca.project(*cloud_cluster_pcd, *cloudPCAprojection);
      std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
      std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;

      //save the information in the ClusterDescriptor
      KeypointCluster Cluster1;


      Cluster1.set_values(cloud_cluster_pcd->size(),centroit_point,pca.getEigenVectors(),pca.getEigenValues(),minPt.x, minPt.y, minPt.z,maxPt.x, maxPt.y, maxPt.z);
      //std::cout << "Cluster1 Eigenvalues: " << Cluster1.Eigenvalues << std::endl;
      //Cluster1.set_values(1,cloud_cluster_pcd->size(),minPt.x, minPt.y, minPt.z,maxPt.x, maxPt.y, maxPt.z);
      std::cout << "Cluster1 Eigenvalues: " << Cluster1.Eigenvalues << std::endl;
      Cluster1.set_cloud(cluster_cnt, *cloud_cluster_pcd);
      Keypoint_Cluster_Queue.push( Cluster1 );
      pcl::io::savePCDFileBinary(str1.c_str(), *cloud_cluster_pcd);
      cont += 1;
    }
    cout << "Size of queue = " << Keypoint_Cluster_Queue.size() << endl;

  //-------------------------------------------------------------------//

  end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  pcl::console::print_info("\n- elapsed time: ");
  pcl::console::print_value("%d", elapsed_seconds.count());

  //-----------------Visualize clusters pcl-visualizer-----------------//

  if (show) {

    vtkObject::GlobalWarningDisplayOff(); // Disable vtk render warning

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("DBSCAN CLUSTERS"));

    int PORT1 = 0;
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
    viewer->setBackgroundColor(0, 0, 0, PORT1);
    viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

    int PORT2 = 0;
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
    viewer->setBackgroundColor(0, 0, 0, PORT2);
    viewer->addText("CLUSTERS", 10, 10, "PORT2", PORT2);

    viewer->setPosition(0, 0);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // Setting background to a dark
    int tmp_thresh=Keypoint_Cluster_Queue.size();
    for (int counter=0;counter < tmp_thresh;counter++){
        string lineID = to_string(counter);
        pcl::PointXYZ src_idx, tgt_idx;
        src_idx= Keypoint_Cluster_Queue.front().Centroid;

        std::cout <<"eigenvect:"<<Keypoint_Cluster_Queue.front().Eigenvectors(0,0)<< std::endl;
        std::cout <<"eigenvect:"<<Keypoint_Cluster_Queue.front().Eigenvectors(1,0)<< std::endl;
        std::cout <<"eigenvect:"<<Keypoint_Cluster_Queue.front().Eigenvectors(2,0)<< std::endl;
        //std::cout <<"eigenvect:"<<Keypoint_Cluster_Queue.front().Eigenvectors(1,1)<< std::endl;

        std::cout <<"eigenvalue:"<<Keypoint_Cluster_Queue.front().Eigenvalues[0]<< std::endl;
        std::cout <<"eigenvalue:"<<Keypoint_Cluster_Queue.front().Eigenvalues[1]<< std::endl;
        std::cout <<"eigenvalue:"<<Keypoint_Cluster_Queue.front().Eigenvalues[2]<< std::endl;
        //std::cout <<"eigenvalue:"<<Keypoint_Cluster_Queue.front().Eigenvalues[3]<< std::endl;
        float scaller, offset;
        int visu_scale=50, vec_cnt=0;
        scaller =(Keypoint_Cluster_Queue.front().Eigenvalues[0]+Keypoint_Cluster_Queue.front().Eigenvalues[1]+Keypoint_Cluster_Queue.front().Eigenvalues[2]);
        tgt_idx=src_idx;
        offset=visu_scale*(Keypoint_Cluster_Queue.front().Eigenvalues[vec_cnt]/scaller);
        /*
        tgt_idx.x=src_idx.x+offset*Keypoint_Cluster_Queue.front().Eigenvectors(0,vec_cnt);
        tgt_idx.y=src_idx.y+offset*Keypoint_Cluster_Queue.front().Eigenvectors(1,vec_cnt);
        tgt_idx.z=src_idx.z+offset*Keypoint_Cluster_Queue.front().Eigenvectors(2,vec_cnt);
        */

        tgt_idx.x=src_idx.x+offset*Keypoint_Cluster_Queue.front().Eigenvectors(vec_cnt,0);
        tgt_idx.y=src_idx.y+offset*Keypoint_Cluster_Queue.front().Eigenvectors(vec_cnt,1);
        tgt_idx.z=src_idx.z+offset*Keypoint_Cluster_Queue.front().Eigenvectors(vec_cnt,2);

        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(src_idx, tgt_idx, 125, 125, 125, "x"+lineID);

        vec_cnt++;
        tgt_idx=src_idx;
        offset=visu_scale*(Keypoint_Cluster_Queue.front().Eigenvalues[vec_cnt]/scaller);

        tgt_idx.x=src_idx.x+offset*Keypoint_Cluster_Queue.front().Eigenvectors(0,vec_cnt);
        tgt_idx.y=src_idx.y+offset*Keypoint_Cluster_Queue.front().Eigenvectors(1,vec_cnt);
        tgt_idx.z=src_idx.z+offset*Keypoint_Cluster_Queue.front().Eigenvectors(2,vec_cnt);
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(src_idx, tgt_idx, 125, 125, 125, "y"+lineID);

        vec_cnt++;
        tgt_idx=src_idx;
        offset=visu_scale*(Keypoint_Cluster_Queue.front().Eigenvalues[vec_cnt]/scaller);

        tgt_idx.x=src_idx.x+offset*Keypoint_Cluster_Queue.front().Eigenvectors(0,vec_cnt);
        tgt_idx.y=src_idx.y+offset*Keypoint_Cluster_Queue.front().Eigenvectors(1,vec_cnt);
        tgt_idx.z=src_idx.z+offset*Keypoint_Cluster_Queue.front().Eigenvectors(2,vec_cnt);
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(src_idx, tgt_idx, 125, 125, 125, "z"+lineID);

        Keypoint_Cluster_Queue.pop();
    }

    int numClust = 0;
    std::random_device seeder;
    std::ranlux48 gen(seeder());
    std::uniform_int_distribution<int> uniform_0_255(0, 255);

    int j = 0;

    for (auto &cluster : dbscan.getClusters()) {

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
      // uint8_t r(255), g(15), b(15);

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

      // Adding different color to each cluster

      for (auto &pointCluster : cluster.clusterPoints) {

        pcl::PointXYZRGB point;
        point.x = pointCluster.x;
        point.y = pointCluster.y;
        point.z = pointCluster.z;

        uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float *>(&rgb);

        cluster_rgb->points.push_back(point);
      }

      j += 3;

      std::string nameId = "cluster_";
      nameId += std::to_string(numClust);

      // std::cout << "Adding: " << nameId << " to pcl visualizer" << std::endl;
      viewer->addPointCloud(cluster_rgb, nameId.c_str(), PORT2);
      numClust += 1;
    }

    double scale = 1;

    viewer->addCoordinateSystem(scale);
    pcl::PointXYZ p11, p22, p33;
    p11.getArray3fMap() << 1, 0, 0;
    p22.getArray3fMap() << 0, 1, 0;
    p33.getArray3fMap() << 0, 0.1, 1;

    viewer->addText3D("x", p11, 0.2, 1, 0, 0, "x_");
    viewer->addText3D("y", p22, 0.2, 0, 1, 0, "y_");
    viewer->addText3D("z", p33, 0.2, 0, 0, 1, "z_");

    if (cloud->points[0].r <= 0 and cloud->points[0].g <= 0 and cloud->points[0].b <= 0) {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(cloud, 255, 255, 0);
      viewer->addPointCloud(cloud, color_handler, "Original_Cloud", PORT1);
    } else {
      viewer->addPointCloud(cloud, "Original_Cloud", PORT1);
    }

    viewer->initCameraParameters();
    viewer->resetCamera();

    // std::cout << "\nGenerated: " << numClust << " clusters" << std::endl;
    pcl::console::print_info("\n- clusters: ");
    pcl::console::print_value("%d", numClust);

    pcl::console::print_info("\npress [q] to exit!\n");

    while (!viewer->wasStopped()) {
      viewer->spin();
    }

  }
  return Keypoint_Cluster_Queue;
}

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

  return 0;
}
