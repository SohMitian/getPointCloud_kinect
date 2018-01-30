#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include<string>
#include<fstream>
#include<iostream>

typedef pcl::PointXYZRGBA PointType;

int main(int argc, char* argv[])
{
	pcl::PCDWriter writer;
	int cnt = 0;
	std::ostringstream oss;
	std::string str = "cloud";
	std::string file;
	
	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);

	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);

		/* Point Cloud Processing */

		cloud = ptr->makeShared();
	};

	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	grabber->start();


	while (!viewer->wasStopped()) {
		// Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(mutex);
		//Sキーを押すと点群を保存
		if (GetAsyncKeyState('S')) {
			file = str + std::to_string(cnt);
			// ファイルが存在しない場合の処理
			//pcl::io::savePCDFileASCII("cloud.pcd", *cloud);
			writer.write<pcl::PointXYZRGBA>(file+".pcd", *cloud, false);
			std::cout << file << ".pcd出力完了" << endl;
			cnt++;
		}
		if (lock.owns_lock() && cloud) {
			
			// Update Point Cloud
			if (!viewer->updatePointCloud(cloud, "cloud")) {
				viewer->addPointCloud(cloud, "cloud");
			
				
			}
		}
	}

	// Stop Grabber
	grabber->stop();
	//writer.write("cloud.pcd", *cloud, false);
	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	return 0;
}