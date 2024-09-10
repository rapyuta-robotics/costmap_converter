#include <costmap_converter/costmap_to_bounding_box.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToBoundingBox, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

CostmapToBoundingBox::CostmapToBoundingBox() : CostmapToPolygonsDBSMCCH()
{
  dynamic_recfg_ = NULL;
}

CostmapToBoundingBox::~CostmapToBoundingBox()
{
  if (dynamic_recfg_ != NULL)
    delete dynamic_recfg_;
}

void CostmapToBoundingBox::initialize(ros::NodeHandle nh)
{
  // DB SCAN
  nh.param("cluster_max_distance", parameter_.max_distance_, 0.4);
  nh.param("cluster_min_pts", parameter_.min_pts_, 2);
  nh.param("cluster_max_pts", parameter_.max_pts_, 30);
  // convex hull (only necessary if outlier filtering is enabled)
  nh.param("convex_hull_min_pt_separation", parameter_.min_keypoint_separation_, 0.1);
  parameter_buffered_ = parameter_;

  // setup dynamic reconfigure
  dynamic_recfg_ = new dynamic_reconfigure::Server<CostmapToBoundingBoxConfig>(nh);
  dynamic_reconfigure::Server<CostmapToBoundingBoxConfig>::CallbackType cb =
      boost::bind(&CostmapToBoundingBox::reconfigureCB, this, _1, _2);
  dynamic_recfg_->setCallback(cb);
}

void CostmapToBoundingBox::compute()
{
  std::vector<std::vector<KeyPoint> > clusters;
  dbScan(clusters);

  // Create new polygon container
  PolygonContainerPtr polygons(new std::vector<geometry_msgs::Polygon>());

  // add bbox to polygon container
  for (std::size_t i = 1; i < clusters.size(); ++i)  // skip first cluster, since it is just noise
  {
    polygons->push_back(geometry_msgs::Polygon());
    bbox(clusters[i], polygons->back());
  }

  // add our non-cluster points to the polygon container (as single points)
  if (!clusters.empty())
  {
    for (std::size_t i = 0; i < clusters.front().size(); ++i)
    {
      polygons->push_back(geometry_msgs::Polygon());
      convertPointToPolygon(clusters.front()[i], polygons->back());
    }
  }

  // replace shared polygon container
  updatePolygonContainer(polygons);
}

void CostmapToBoundingBox::bbox(std::vector<KeyPoint>& cluster, geometry_msgs::Polygon& polygon)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();
  for (const auto& keypoint : cluster)
  {
    min_x = std::min(min_x, keypoint.x);
    min_y = std::min(min_y, keypoint.y);
    max_x = std::max(max_x, keypoint.x);
    max_y = std::max(max_y, keypoint.y);
  }

  polygon.points.resize(4);
  polygon.points[0].x = min_x - costmap_resolution_ / 2;
  polygon.points[0].y = min_y - costmap_resolution_ / 2;

  polygon.points[1].x = min_x - costmap_resolution_ / 2;
  polygon.points[1].y = max_y + costmap_resolution_ / 2;

  polygon.points[2].x = max_x + costmap_resolution_ / 2;
  polygon.points[2].y = max_y + costmap_resolution_ / 2;

  polygon.points[3].x = max_x + costmap_resolution_ / 2;
  polygon.points[3].y = min_y - costmap_resolution_ / 2;
}

void CostmapToBoundingBox::reconfigureCB(CostmapToBoundingBoxConfig& config, uint32_t level)
{
  boost::mutex::scoped_lock lock(parameter_mutex_);
  parameter_buffered_.max_distance_ = config.cluster_max_distance;
  parameter_buffered_.min_pts_ = config.cluster_min_pts;
  parameter_buffered_.max_pts_ = config.cluster_max_pts;
  parameter_buffered_.min_keypoint_separation_ = config.cluster_min_pts;
}

}  // end namespace costmap_converter
