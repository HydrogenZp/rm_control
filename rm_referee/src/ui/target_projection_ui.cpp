//
// Created for target projection UI
//

#include "rm_referee/ui/target_projection_ui.h"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_referee
{
void TargetProjectionUi::updateTrackData(const rm_msgs::TrackData::ConstPtr& data)
{
  if (!data->tracking)
  {
    tracking_ = false;
    updateForQueue();
    return;
  }

  geometry_msgs::PointStamped p_odom, p_vt;
  p_odom.header = data->header;
  p_odom.point = data->position;

  try
  {
    geometry_msgs::TransformStamped transform_stamped =
        tf_buffer_.lookupTransform("vt_optical_frame", data->header.frame_id, ros::Time(0));
    tf2::doTransform(p_odom, p_vt, transform_stamped);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "TargetProjectionUi TF failed: %s", ex.what());
    return;
  }

  double z = p_vt.point.z;
  if (z < 0.1)
    return;

  center_u_ = static_cast<int>(fx_ * p_vt.point.x / z + cx_);
  center_v_ = static_cast<int>(fy_ * p_vt.point.y / z + cy_);

  center_u_ = std::max(0, std::min(center_u_, 2047));
  center_v_ = std::max(0, std::min(center_v_, 2047));

  tracking_ = true;
  updateForQueue();
}

void TargetProjectionUi::updateConfig()
{
  if (tracking_)
  {
    graph_->setStartX(center_u_);
    graph_->setStartY(center_v_);
    graph_->setRadius(radius_);
  }
  else
  {
    graph_->setStartX(-1);
    graph_->setStartY(-1);
    graph_->setRadius(0);
  }
}

}  // namespace rm_referee