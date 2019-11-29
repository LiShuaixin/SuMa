#include "POSReader.h"

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

POSReader::POSReader( const std::string& pos_filepath, uint32_t buffer_size )
                    : currentPos(0), bufferedPos(buffer_size), firstBufferedPos(0) {
  initPosFilenames(pos_filepath);
}

void POSReader::initPosFilenames(const std::string& pos_filepath) {
  posFilenames.clear();
  posTimestamps.clear();

  std::vector<std::string> files = FileUtil::getDirectoryListing(FileUtil::dirName(pos_filepath));
  
  /** filter irrelevant files. **/
  for (uint32_t i = 0; i < files.size(); ++i) {
    if (FileUtil::extension(files[i]) == ".txt") {
      posFilenames.push_back(files[i]);
    }
  }

  std::sort(posFilenames.begin(), posFilenames.end());
  // std::cout <<"oxst filename size: " << posFilenames.size() << std::endl;

  Ro = Eigen::Matrix3f::Identity();
  to = Eigen::Vector3f::Zero();
    
  // read kitti GPS/INS timestamp
  std::ifstream ftime;
  
  std::string timestamps(FileUtil::dirName(pos_filepath) + "/../timestamps.txt");
  std::cout <<"oxst timestamp filename: " << timestamps << std::endl;
    
  ftime.open(timestamps.c_str());
    
  while(!ftime.eof()){
    std::string s;
    std::getline(ftime, s);
    if(s.empty())
    {
	std::cout << "Load POS timestamp failed!" << std::endl;
    }
    
    double stamp;
    stamp = KITTI::parseTimeStamp(s);
    // std::cout <<"oxst timestamp string: " << s << ", which equals to: " << std::fixed << std::setprecision(10) << stamp << std::endl;
    
    posTimestamps.push_back(stamp);
  }
}

void POSReader::reset() {
  currentPos = 0;
  bufferedPos.clear();
  firstBufferedPos = 0;
  
  scale = -1;
}

bool POSReader::read(Pos& pos) {
  bool result = false;

  if(scale == -1 )
      return false;
  
  if (currentPos >= (int32_t)posFilenames.size()) {
    return false;
  }

  if (currentPos - firstBufferedPos < bufferedPos.size()) /** scan already in buffer, no need to read scan. **/
  {
    pos = bufferedPos[currentPos - firstBufferedPos];

    result = true;
  } else {
    result = read(currentPos, pos);
    if (result) {
      if (bufferedPos.capacity() == bufferedPos.size()) ++firstBufferedPos;
      bufferedPos.push_back(pos);
    }
  }

  ++currentPos;

  return result;
}

bool POSReader::isSeekable() const {
  return !(scale == -1);
}

void POSReader::seek(uint32_t posnr) {
  assert(posnr < posFilenames.size());
  
  if( scale == -1 )
      return;

  /** pos already in buffer, nothing to read just set current pos **/
  if (posnr - firstBufferedPos < bufferedPos.size()) {
    currentPos = posnr;
  } else if (currentPos < (int32_t)posnr) {
    /** if we don't have to read everything again than read missing pos. **/
    if ((posnr - 1) - currentPos < bufferedPos.capacity()) {
      currentPos =
          firstBufferedPos + bufferedPos.size(); /** advance to last scan in buffer to read no scans twice. **/
      while (currentPos < (int32_t)posnr) {
        Pos pos;

        if (bufferedPos.capacity() == bufferedPos.size()) ++firstBufferedPos;
        read(currentPos, pos);
        bufferedPos.push_back(pos);

        ++currentPos;
      }
    } else /** otherwise we just reset the buffer and start buffering again. **/
    {
      currentPos = posnr;
      firstBufferedPos = posnr;
      bufferedPos.clear();
    }
  } else if (currentPos > (int32_t)posnr) /** we have to add scans at the beginning **/
  {
    /** if we don't have to read every thing new, than read missing scans. **/
    if (currentPos - posnr < bufferedPos.capacity()) {
      currentPos = firstBufferedPos;

      while (currentPos > (int32_t)posnr) {
        --currentPos;

        Pos pos;

        read(currentPos, pos);
        bufferedPos.push_front(pos);
      }

      firstBufferedPos = currentPos;
    } else /** otherwise we just reset the buffer and start buffering again. **/
    {
      currentPos = posnr;
      firstBufferedPos = posnr;
      bufferedPos.clear();
    }
  }
}

uint32_t POSReader::count() const {
  return posFilenames.size();
}

bool POSReader::read(uint32_t pos_idx, Pos& pos) {
  if (pos_idx > posFilenames.size()) return false;

  float stamp = posTimestamps[pos_idx];
  std::ifstream in(posFilenames[pos_idx].c_str());
  if (!in.is_open()) return false;
    
  std::string s;
  std::getline(in, s);
    
  if(s.empty()){
    std::cout << "Load POS failed!" << std::endl;
  }
    
  std::string lat, lon, alt, ro, pi, ya, vn, ve, vf, vl, vu, 
              ax, ay, az, af, al, au, wx, wy, wz, wf, wl, wu, 
	      pos_accuracy, vel_accuracy, navstat, numsats, posmode, velmode, orimode;

  std::istringstream is(s);
  is >> lat >> lon >> alt >> ro >> pi >> ya >> vn >> ve >> vf >> vl >> vu >> 
	ax >> ay >> az >> af >> al >> au >> wx >> wy >> wz >> wf >> wl >> wu >> 
	pos_accuracy >> vel_accuracy >> navstat >> numsats >> posmode >> velmode >> orimode;
    
  Eigen::Vector3f BLH(atof(lat.c_str())*M_PI/180, atof(lon.c_str())*M_PI/180, atof(alt.c_str()));
    
  if(scale == -1)
  {
    scale = cos(BLH(0));
    to = KITTI::transBLH2XYZ(BLH, scale);
    ro_ = atof(ro.c_str()); po_ = atof(pi.c_str()); ho_ = atof(ya.c_str());
    Ro = KITTI::transEuler2Rotation(ro_, po_, ho_);
    
    if(scale == -1)
	std::cout << "ERROR: scale still equal to -1!!!" << std::endl;
  }
    
  Eigen::Vector3f twc = KITTI::transBLH2XYZ(BLH, scale); 
    
  // 旋转角: 由ENU到当前时刻各轴系的旋转角度
  float roll, pitch, heading;
  roll = atof(ro.c_str());
  pitch = atof(pi.c_str());
  heading = atof(ya.c_str());
  Eigen::Matrix3f Rwc = KITTI::transEuler2Rotation(roll, pitch, heading); 
    
  Eigen::Matrix4f Tmp = Eigen::Matrix4f::Identity(), Twc, T0;
  T0.setIdentity();
  T0.topLeftCorner<3, 3>() = Ro;
  T0.topRightCorner<3, 1>() = to;
    
  Tmp.topLeftCorner<3, 3>() = Rwc;
  Tmp.topRightCorner<3, 1>() = twc;
  // std::cout << "[DEBUG POSE READER]: transformation read Tmp =\n " << Tmp << std::endl;
    
  Twc = T0.inverse() * Tmp;
  // std::cout << "[DEBUG POSE READER]: transformation read =\n " << Twc << std::endl;
 
  // relative pose
  pos.roll() = roll;
  pos.pitch() = pitch;
  pos.yaw() = heading;
  pos.timestamp() = stamp;
  // std::cout << "TEST pos timestamp = " << std::fixed << std::setprecision(10) << pos.timestamp() << std::endl;
  
  Transform Ti(Twc.topLeftCorner<3, 3>(), Vector3f(Twc.topRightCorner<3, 1>()));
  pos.pose() = Ti;
  // std::cout << "TEST pos transformation = \n" << std::fixed << std::setprecision(10) << pos.pose() << std::endl;
    
  // imu and gps measurements
  pos.ax() = atof(ax.c_str());
  pos.ay() = atof(ay.c_str());
  pos.az() = atof(az.c_str());
  pos.wx() = atof(wx.c_str());
  pos.wy() = atof(wy.c_str());
  pos.wz() = atof(wz.c_str());
  pos.vf() = atof(vf.c_str());
  pos.vl() = atof(vl.c_str());
  pos.vu() = atof(vu.c_str());
    
  pos.lat() = BLH(0);
  pos.lon() = BLH(1);
  pos.alt() = BLH(2);   
  // std::cout << "TEST original BLH = " << std::fixed << std::setprecision(4) << lat << ", " << lon << ", " << alt << std::endl;
  // std::cout << "TEST BLH = " << std::fixed << std::setprecision(4) << pos.lat() << ", " << pos.lon() << ", " << pos.alt() << std::endl;

  /// \note can be used to control the availability of pos estimation by setting if condition 
  if(1)
    pos.setValid(true);
  else
    pos.setValid(false);
  in.close();

  return true;
}
}
