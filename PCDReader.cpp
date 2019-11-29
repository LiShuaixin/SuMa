#include "PCDReader.h"

#include <rv/FileUtil.h>
#include <algorithm>
#include <cmath>
#include <fstream>

#include <glow/glutil.h>
#include <rv/XmlDocument.h>
#include <rv/string_utils.h>
#include <boost/lexical_cast.hpp>

// TODO: read calibration from provided calibration files.

namespace rv {

PCDReader::PCDReader(const std::string& scan_filename, uint32_t buffer_size)
    : currentScan(0), bufferedScans(buffer_size), firstBufferedScan(0) {
  initScanFilenames(scan_filename);
}

void PCDReader::initScanFilenames(const std::string& scan_filename) {
  scan_filenames.clear();

  std::vector<std::string> files = FileUtil::getDirectoryListing(FileUtil::dirName(scan_filename));
  /** filter irrelevant files. **/
  for (uint32_t i = 0; i < files.size(); ++i) {
    if (FileUtil::extension(files[i]) == ".pcd") {
      scan_filenames.push_back(files[i]);
    }
  }

  std::sort(scan_filenames.begin(), scan_filenames.end());
}

void PCDReader::reset() {
  currentScan = 0;
  bufferedScans.clear();
  firstBufferedScan = 0;
}

bool PCDReader::read(Laserscan& scan) {
  bool result = false;
  scan.clear();

  if (currentScan >= (int32_t)scan_filenames.size()) {
    return false;
  }

  if (currentScan - firstBufferedScan < bufferedScans.size()) /** scan already in buffer, no need to read scan. **/
  {
    scan = bufferedScans[currentScan - firstBufferedScan];

    result = true;
  } else {
    result = read(currentScan, scan);
    if (result) {
      if (bufferedScans.capacity() == bufferedScans.size()) ++firstBufferedScan;
      bufferedScans.push_back(scan);
    }
  }

  ++currentScan;

  return result;
}

bool PCDReader::isSeekable() const {
  return true;
}

void PCDReader::seek(uint32_t scannr) {
  assert(scannr < scan_filenames.size());

  /** scan already in buffer, nothing to read just set current scan **/
  if (scannr - firstBufferedScan < bufferedScans.size()) {
    currentScan = scannr;
  } else if (currentScan < (int32_t)scannr) {
    /** if we don't have to read everything again than read missing scans. **/
    if ((scannr - 1) - currentScan < bufferedScans.capacity()) {
      currentScan =
          firstBufferedScan + bufferedScans.size(); /** advance to last scan in buffer to read no scans twice. **/
      while (currentScan < (int32_t)scannr) {
        Laserscan scan;

        if (bufferedScans.capacity() == bufferedScans.size()) ++firstBufferedScan;
        read(currentScan, scan);
        bufferedScans.push_back(scan);

        ++currentScan;
      }
    } else /** otherwise we just reset the buffer and start buffering again. **/
    {
      currentScan = scannr;
      firstBufferedScan = scannr;
      bufferedScans.clear();
    }
  } else if (currentScan > (int32_t)scannr) /** we have to add scans at the beginning **/
  {
    /** if we don't have to read every thing new, than read missing scans. **/
    if (currentScan - scannr < bufferedScans.capacity()) {
      currentScan = firstBufferedScan;

      while (currentScan > (int32_t)scannr) {
        --currentScan;

        Laserscan scan;

        read(currentScan, scan);
        bufferedScans.push_front(scan);
      }

      firstBufferedScan = currentScan;
    } else /** otherwise we just reset the buffer and start buffering again. **/
    {
      currentScan = scannr;
      firstBufferedScan = scannr;
      bufferedScans.clear();
    }
  }
}

uint32_t PCDReader::count() const {
  return scan_filenames.size();
}

bool PCDReader::read(uint32_t scan_idx, Laserscan& scan) {
  if (scan_idx > scan_filenames.size()) return false;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(scan_filenames[scan_idx], *cloud) == -1)
  {
    PCL_ERROR("Cloudn't read file!");
    return false;
  }

  scan.clear();
  std::vector<Point3f>& points = scan.points();
  std::vector<float>& remissions = scan.remissions();

  int num_points = cloud->size();
  points.resize(num_points);
  remissions.resize(num_points);

  float max_remission = 0;

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x() = cloud->points[i].x;
    points[i].y() = cloud->points[i].y;
    points[i].z() = cloud->points[i].z;
    remissions[i] = cloud->points[i].intensity;
    max_remission = std::max(remissions[i], max_remission);
  }

  for (uint32_t i = 0; i < num_points; ++i) {
    remissions[i] /= max_remission;
  }
  
  // std::cout << "load scan size = " << scan.size() << std::endl;

  return true;
}
}
