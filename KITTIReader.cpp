#include "KITTIReader.h"

#include <rv/FileUtil.h>
#include <util/kitti_utils.h>
#include <algorithm>
#include <cmath>
#include <fstream>

#include <glow/glutil.h>
#include <rv/XmlDocument.h>
#include <rv/string_utils.h>
#include <boost/lexical_cast.hpp>

// TODO: read calibration from provided calibration files.

namespace rv {

KITTIReader::KITTIReader(const std::string& scan_filename, uint32_t buffer_size)
    : currentScan(0), bufferedScans(buffer_size), firstBufferedScan(0) {
  initScanFilenames(scan_filename);
}

void KITTIReader::initScanFilenames(const std::string& scan_filename) {
  scan_filenames.clear();
  scan_timestamps.clear();

  std::vector<std::string> files = FileUtil::getDirectoryListing(FileUtil::dirName(scan_filename));
  /** filter irrelevant files. **/
  for (uint32_t i = 0; i < files.size(); ++i) {
    if (FileUtil::extension(files[i]) == ".bin") {
      scan_filenames.push_back(files[i]);
    }
  }

  std::sort(scan_filenames.begin(), scan_filenames.end());
  
  // read kitti GPS/INS timestamp
  std::ifstream ftime, ftime_start, ftime_end;
  
  std::string timestamps(FileUtil::dirName(scan_filename) + "/../timestamps.txt");
  std::string timestamps_start(FileUtil::dirName(scan_filename) + "/../timestamps_start.txt");
  std::string timestamps_end(FileUtil::dirName(scan_filename) + "/../timestamps_end.txt");
  std::cout <<"laser scan timestamp filename: " << timestamps << std::endl;
    
  ftime.open(timestamps.c_str());
  ftime_start.open(timestamps_start.c_str());
  ftime_end.open(timestamps_end.c_str());
  
    
  while(!ftime.eof()){
    std::vector<double> time_vec;
    std::string s1, s2, s3;
    
    std::getline(ftime_start, s1);
    if(s1.empty())
    {
	std::cout << "Load laserscan timestamp failed!" << std::endl;
    }
    
    double stamp_start;
    stamp_start = KITTI::parseTimeStamp(s1);
    // std::cout <<"oxst timestamp string: " << s << ", which equals to: " << std::fixed << std::setprecision(10) << stamp << std::endl;
    time_vec.push_back(stamp_start);
    
    std::getline(ftime, s2);
    if(s2.empty())
    {
	std::cout << "Load laserscan timestamp failed!" << std::endl;
    }
    
    double stamp;
    stamp = KITTI::parseTimeStamp(s2);
    // std::cout <<"oxst timestamp string: " << s << ", which equals to: " << std::fixed << std::setprecision(10) << stamp << std::endl;
    time_vec.push_back(stamp);
    
    std::getline(ftime_end, s3);
    if(s3.empty())
    {
	std::cout << "Load laserscan timestamp failed!" << std::endl;
    }
    
    double stamp_end;
    stamp_end = KITTI::parseTimeStamp(s3);
    // std::cout <<"oxst timestamp string: " << s << ", which equals to: " << std::fixed << std::setprecision(10) << stamp << std::endl;
    time_vec.push_back(stamp_end);
    
    scan_timestamps.push_back(time_vec);
  }
}

void KITTIReader::reset() {
  currentScan = 0;
  bufferedScans.clear();
  firstBufferedScan = 0;
}

bool KITTIReader::read(Laserscan& scan) {
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

bool KITTIReader::isSeekable() const {
  return true;
}

void KITTIReader::seek(uint32_t scannr) {
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

uint32_t KITTIReader::count() const {
  return scan_filenames.size();
}

bool KITTIReader::read(uint32_t scan_idx, Laserscan& scan) {
  if (scan_idx > scan_filenames.size()) return false;
  
  std::ifstream in(scan_filenames[scan_idx].c_str(), std::ios::binary);
  if (!in.is_open()) return false;

  scan.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (4 * sizeof(float));
  in.seekg(0, std::ios::beg);

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();
  std::vector<Point3f>& points = scan.points_;
  std::vector<float>& remissions = scan.remissions_;
  
  points.resize(num_points);
  remissions.resize(num_points);

  float max_remission = 0;

  for (uint32_t i = 0; i < num_points; ++i) {
    points[i].x() = values[4 * i];
    points[i].y() = values[4 * i + 1];
    points[i].z() = values[4 * i + 2];
    remissions[i] = values[4 * i + 3];
    max_remission = std::max(remissions[i], max_remission);
  }

  for (uint32_t i = 0; i < num_points; ++i) {
    remissions[i] /= max_remission;
  }
  
  /// \note calculate real time of each laser point
  double stamp_start, stamp, stamp_end;
  stamp_start = scan_timestamps[scan_idx][0];
  stamp = scan_timestamps[scan_idx][1];
  stamp_end = scan_timestamps[scan_idx][2];
  double scan_period = stamp_end - stamp_start;
  scan.scan_period_ = scan_period;
  scan.start_timestamp = stamp_start;
  
  std::vector<double>& timestamps = scan.timestamps_;
  timestamps.resize(num_points); 
  // std::cout << "start time = " << stamp_start << ", end time = " << stamp_end << std::endl;
  
  // x轴到起始扫描线的夹角，顺时针为正
  double start_ori = -atan2(points[0].y(), points[0].x()); 
  double end_ori = -atan2(points[num_points - 1].y(), points[num_points - 1].x()) + 2 * M_PI; 
    
  // 确定end、ori的水平面位置关系->输入的点云坐标系为前x左y上z
  if(end_ori - start_ori > 3 * M_PI){
    end_ori -= 2 * M_PI;
  }else if(end_ori - start_ori < M_PI){	    
    end_ori += 2 * M_PI;
  }
    
  bool halfPassed = false;
  int count = num_points;
    
  for(uint32_t i = 0; i < num_points; i++){	
    // skip zero valued points
    if ( points[i].x() * points[i].x() 
       + points[i].y() * points[i].y() 
       + points[i].z() * points[i].z() < 0.0001) continue;
	    
    double ori = -atan2(points[i].y(), points[i].x());
    
    if(!halfPassed)
    {
      if(ori < start_ori-M_PI/2)
	ori += 2 * M_PI;
      else if(ori > start_ori + M_PI*3/2)
	ori -= 2 * M_PI;
      
      if(ori - start_ori > M_PI)
	halfPassed = true;
    }
    else
    {
      ori += 2* M_PI;
	
      if(ori < end_ori - M_PI*3/2)
	ori += 2 * M_PI;
      else if(ori > end_ori + M_PI/2)
	ori -= 2 * M_PI;
    }
	
    // 每帧点云中各点扫描时间
    double real_time = ((ori - start_ori) / (end_ori - start_ori)) * scan_period;
    timestamps.push_back( real_time );	    
  }    

  return true;
}
}
