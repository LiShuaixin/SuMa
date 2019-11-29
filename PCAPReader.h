#ifndef PCAPLASERSCANREADER_H_
#define PCAPLASERSCANREADER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <typeinfo>

#include <boost/algorithm/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/pcd_io.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>

#include "LaserscanReader.h"
#include "PCDReader.h"

#include <rv/ParameterList.h>
#include <rv/RingBuffer.h>
#include <rv/IOError.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

#define SHOW_FPS 0
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = getTime ();\
    double now = getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

namespace rv {

/** \brief a reader for the PCAP datasets provided by the KIT.
 *
 * \author Lee
 */
class PCAPReader : public PCDReader {
public:
     
  typedef pcl::PointXYZI PointType;
  typedef PointCloud<PointType> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef typename Cloud::Ptr CloudPtr;
  
  PCAPReader(Grabber& grabber, PointCloudColorHandler<PointType> *handler, const string& scan_file_path, uint32_t buffer_size = 50);

  void cloud_callback (const CloudConstPtr& cloud);
  void keyboard_callback (const KeyboardEvent& event,
                       void* /*cookie*/);  

 protected:
  void transPCAP();

  std::string filePath;
  
  boost::shared_ptr<PCLVisualizer> cloudViewer;
  boost::shared_ptr<ImageViewer> imageViewer;

  Grabber& pcapGrabber;
  boost::mutex cloudMutex;
  boost::mutex imageMutex;

  CloudConstPtr laserScan;
  PointCloudColorHandler<PointType> *colorHandler;

};
}

#endif /* PCAPLASERSCANREADER_H_ */