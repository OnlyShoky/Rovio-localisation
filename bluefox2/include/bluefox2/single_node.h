#ifndef BLUEFOX2_SINGLE_NODE_H_
#define BLUEFOX2_SINGLE_NODE_H_

#include "bluefox2/Bluefox2DynConfig.h"
#include <camera_base/camera_node_base.h>
#include <sensor_msgs/TimeReference.h>

namespace bluefox2 {

// must be x^2-1
#define FIFO_SIZE 1023

struct TriggerPacket {
  uint32_t triggerCounter;
  ros::Time triggerTime;
};

typedef struct TriggerPacket TriggerPacket_t;


class Bluefox2Ros;

class SingleNode : public camera_base::CameraNodeBase<Bluefox2DynConfig> {
 public:
  explicit SingleNode(ros::NodeHandle &pnh);
  
  virtual void Acquire() override;
  virtual void Setup(Bluefox2DynConfig &config) override;

  void AcquireOnce();

  void callback(const sensor_msgs::TimeReference::ConstPtr &time_ref);
 
 private:
  boost::shared_ptr<Bluefox2Ros> bluefox2_ros_;
  bool boost_{false};

  int ctm;
  ros::Subscriber subTimeRef;
  int outOfSyncCounter;

  TriggerPacket_t fifo[FIFO_SIZE];
  uint32_t nextTriggerCounter;
  int fifoReadPos;
  int fifoWritePos;
  void fifoWrite(TriggerPacket_t pkt);
  bool fifoRead(TriggerPacket_t &pkt);
  bool fifoLook(TriggerPacket_t &pkt);
};

}  // namespace bluefox2

#endif  // BLUEFOX2_SINGLE_NODE_H_
