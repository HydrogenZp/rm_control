//
// Created for target projection UI
//

#pragma once

#include "rm_referee/ui/time_change_ui.h"
#include <rm_msgs/TrackData.h>
#include <geometry_msgs/PointStamped.h>

namespace rm_referee
{
class TargetProjectionUi : public TimeChangeUi
{
public:
  explicit TargetProjectionUi(XmlRpc::XmlRpcValue& rpc_value, Base& base,
                              std::deque<Graph>* graph_queue, std::deque<Graph>* character_queue)
    : TimeChangeUi(rpc_value, base, "target_projection", graph_queue, character_queue)
  {
    if (rpc_value.hasMember("data"))
    {
      XmlRpc::XmlRpcValue& data = rpc_value["data"];
      fx_ = static_cast<double>(data["fx"]);
      fy_ = static_cast<double>(data["fy"]);
      cx_ = static_cast<double>(data["cx"]);
      cy_ = static_cast<double>(data["cy"]);
    }
    else
      ROS_WARN("TargetProjectionUi config 'data' not defined.");

    radius_ = static_cast<int>(rpc_value["config"]["radius"]);
  }

  void updateTrackData(const rm_msgs::TrackData::ConstPtr& data);
  void addForQueue(int add_times = 1) override;

private:
  void updateConfig() override;

  double fx_{}, fy_{}, cx_{}, cy_{};
  int radius_{};
  int center_u_{}, center_v_{};
  bool tracking_{};
};

}  // namespace rm_referee