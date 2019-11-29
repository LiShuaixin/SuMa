
#ifndef PCDLASERSCANREADER_H_
#define PCDLASERSCANREADER_H_

#include "LaserscanReader.h"

#include <rv/ParameterList.h>
#include <rv/RingBuffer.h>
#include <rv/IOError.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace rv {

/** \brief a reader for the PCD file.
 *
 * \author Lee
 */
class PCDReader : public LaserscanReader {
 public:
  PCDReader(const std::string& scan_filename, uint32_t buffer_size = 50);

  void reset() override;
  bool read(Laserscan& scan) override;
  void seek(uint32_t scan) override;
  bool isSeekable() const override;
  uint32_t count() const override;

 protected:
  void initScanFilenames(const std::string& scan_filename);

  bool read(uint32_t scan_idx, Laserscan& scan);

  int32_t currentScan;
  std::vector<std::string> scan_filenames;
  RingBuffer<Laserscan> bufferedScans;
  uint32_t firstBufferedScan;

};
}
#endif /* PCDLASERSCANREADER_H_ */
