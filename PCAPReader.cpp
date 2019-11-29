#include "PCAPReader.h"

namespace rv {

PCAPReader::PCAPReader(Grabber& grabber, PointCloudColorHandler<PointType> *handler,  const string& scan_file_path, uint32_t buffer_size)
    : PCDReader(scan_file_path)
    , cloudViewer(new PCLVisualizer ("PCL VLP Cloud"))
    , pcapGrabber(grabber)
    , colorHandler(handler)
    , filePath(scan_file_path)
{
    transPCAP();
    
    currentScan = 0;
    bufferedScans = buffer_size;
    firstBufferedScan = 0;
    
    initScanFilenames(filePath);
}
void PCAPReader::cloud_callback(const CloudConstPtr& cloud)
{
    FPS_CALC("cloud callback");
    boost::mutex::scoped_lock lock (cloudMutex);
    laserScan = cloud;
}

void PCAPReader::keyboard_callback(const KeyboardEvent& event, void*)
{
    if (event.keyUp ())
    {
	switch (event.getKeyCode ())
	{
	    case '0':
	    delete colorHandler;
	    colorHandler = new PointCloudColorHandlerCustom<PointXYZI> (255, 255, 255);
	    break;
	    case '1':
	    delete colorHandler;
	    colorHandler = new PointCloudColorHandlerGenericField<PointXYZI> ("x");
	    break;
	    case '2':
	    delete colorHandler;
	    colorHandler = new PointCloudColorHandlerGenericField<PointXYZI> ("y");
	    break;
	    case '3':
	    delete colorHandler;
	    colorHandler = new PointCloudColorHandlerGenericField<PointXYZI> ("z");
	    break;
	    case '4':
	    delete colorHandler;
	    colorHandler = new PointCloudColorHandlerGenericField<PointXYZI> ("intensity");
	    break;
	    case 'a':
	    cloudViewer->removeAllCoordinateSystems ();
	    cloudViewer->addCoordinateSystem (1.0, "global");
	    break;
	    case 'A':
	    cloudViewer->removeAllCoordinateSystems ();
	    break;
	}
    }
}


void PCAPReader::transPCAP()
{
    cloudViewer->addCoordinateSystem (1.0, "global");
    cloudViewer->setBackgroundColor (0, 0, 0);
    cloudViewer->initCameraParameters ();
    cloudViewer->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
    cloudViewer->setCameraClipDistances (0.0, 50.0);
    cloudViewer->registerKeyboardCallback (&PCAPReader::keyboard_callback, *this);

    boost::function<void
    (const CloudConstPtr&)> cloud_cb = boost::bind (&PCAPReader::cloud_callback, this, _1);
    boost::signals2::connection cloud_connection = pcapGrabber.registerCallback (cloud_cb);

    pcapGrabber.start ();
  
    ofstream out_timestamps( "timestamps.txt", ios::out );
    int count = 0;
    
    string file_path = filePath.substr(0, filePath.rfind("/")) + "/velodyne/";
    std::cout << "scan file path is: " << file_path << std::endl;
    
    while (!cloudViewer->wasStopped ())
    {
        CloudConstPtr tmp, cloud;

        if (cloudMutex.try_lock ())
        {
          laserScan.swap(cloud);
          cloudMutex.unlock ();
        }

        if (cloud)
        {
	  // std::cout << "size of current scan = " << cloud->size() << std::endl;
          
          char tmp[256];
          // std::sprintf(tmp, "%010d.bin", count);
	  std::sprintf(tmp, "%010d.pcd", count);
          string out_pts_file_tmp(tmp);

	  filePath.clear();
	  filePath = file_path + out_pts_file_tmp;
	  
          // ofstream out_pts( filePath, ios::out | ios::binary );
      
          //-- get time stamp and write timestamps.txt
          unsigned int timestamp_upper = cloud->header.stamp >> 32; // time()
          unsigned int timestamp_lower = cloud->header.stamp & 0xffffffff;  // microseconds from the top of the hour
          out_timestamps << std::to_string( timestamp_lower ) << '\n';

          //-- get pointcloud and write pts.bin
//           for(int i = 0; i < cloud->size(); i++)
//           {
//             pcl::PointXYZI point = cloud->points[i];
//             float px = point.x;
//             float py = point.y;
//             float pz = point.z;
//             float pr = point.intensity;
//             out_pts.write((char *)(&px), sizeof(px));
//             out_pts.write((char *)(&py), sizeof(py));
//             out_pts.write((char *)(&pz), sizeof(pz));
//             // out_pts.write((char *)(&pr), sizeof(pr));
// 
//           }
// 
//           out_pts.close();
          pcl::io::savePCDFileBinary<pcl::PointXYZI>(filePath, *cloud);

          FPS_CALC("drawing cloud");
          colorHandler->setInputCloud(cloud);
          if (!cloudViewer->updatePointCloud (cloud, *colorHandler, "VLP"))
            cloudViewer->addPointCloud (cloud, *colorHandler, "VLP");

          cloudViewer->spinOnce();
          count++;
        }

        if (!pcapGrabber.isRunning ())
          cloudViewer->spin ();

        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }

    pcapGrabber.stop ();

    cloud_connection.disconnect ();
    out_timestamps.close();
    
    std::printf("[INFO] Transformed pcap as .bin file done.\n");
    // std::cout << "KITTIReader input file path is: " << filePath << std::endl;
}


}