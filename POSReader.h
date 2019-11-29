#ifndef POSREADER_H_
#define POSREADER_H_

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <time.h>
#include <string>
#include <stdio.h>  
#include <stdlib.h>  
#include <vector>

#include <rv/Pos.h>
#include <rv/IOError.h>
#include <rv/ParameterList.h>
#include <rv/RingBuffer.h>

namespace rv {
    
class POSReader
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  POSReader(const std::string& pos_filepath, uint32_t buffer_size = 50);
  ~POSReader() {};
  
  void reset();  
  bool read( Pos& pos );
  void seek(uint32_t posnr);
  bool isSeekable() const;
  uint32_t count() const;
  
  
    // long long unixtime_offset( const std::string& y, const std::string& m, const std::string& d );
    
    // inline Pos get_data() { return mPos; }

protected:
  void initPosFilenames(const std::string& pos_filepath);

  bool read(uint32_t pos_idx, Pos& pos);

  int32_t currentPos;
  std::vector<std::string> posFilenames;
  std::vector<double> posTimestamps;
  RingBuffer<Pos> bufferedPos;
  uint32_t firstBufferedPos;
    
  // reference coordinate param
  float ro_, po_, ho_;
  Eigen::Vector3f to;
  Eigen::Matrix3f Ro;
  // Eigen::Matrix4f To_;

  double scale = -1;
};
}
#endif /* LASERSCANREADER_H_ */