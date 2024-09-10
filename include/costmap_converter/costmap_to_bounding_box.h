#ifndef COSTMAP_TO_BOUNDING_BOX_H_
#define COSTMAP_TO_BOUNDING_BOX_H_

#include <costmap_converter/costmap_converter_interface.h>
#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/misc.h>
#include <costmap_converter/CostmapToBoundingBoxConfig.h>

namespace costmap_converter
{

/**
 * @class CostmapToBoundingBox
 * @brief This class converts the costmap_2d into a set bounding boxes
 *
 * The conversion is performed in two stages:
 * 1. Clusters in the costmap are collected using the DBSCAN Algorithm (https://en.wikipedia.org/wiki/DBSCAN)
 *    Reference: Ester, Martin; Kriegel, Hans-Peter; Sander, Jörg; Xu, Xiaowei,
 *               A density-based algorithm for discovering clusters in large spatial databases with noise.
 *               Proceedings of the Second International Conference on Knowledge Discovery and Data Mining. AAAI Press.
 * 1996. pp. 226–231. ISBN 1-57735-004-9.
 *
 * 2. Instead of computing the convex hull of each cluster, a bounding box is computed.
 *
 * The output is a container of polygons (RECTANGLES bounding boxes)
 */
class CostmapToBoundingBox : public CostmapToPolygonsDBSMCCH
{
private:
  static constexpr auto LOGNAME = "CostmapToBoundingBox";

public:
  /**
   * @brief Constructor
   */
  CostmapToBoundingBox();

  /**
   * @brief Destructor
   */
  virtual ~CostmapToBoundingBox();

  /**
   * @brief Initialize the plugin
   * @param nh Nodehandle that defines the namespace for parameters
   */
  virtual void initialize(ros::NodeHandle nh);

  /**
   * @brief This method performs the actual work (conversion of the costmap to bounding box)
   */
  virtual void compute();

private:
  /**
   * @brief Callback for the dynamic_reconfigure node.
   *
   * This callback allows to modify parameters dynamically at runtime without restarting the node
   * @param config Reference to the dynamic reconfigure config
   * @param level Dynamic reconfigure level
   */
  void reconfigureCB(CostmapToBoundingBoxConfig& config, uint32_t level);

  /**
   * @brief Computes the bounding box of a cluster of points (keypoints) and stores it in a polygon message
   * @param cluster List of keypoints that should be converted into a polygon
   * @param[out] polygon The resulting bounding box
   */
  void bbox(std::vector<KeyPoint>& cluster, geometry_msgs::Polygon& polygon);

  dynamic_reconfigure::Server<CostmapToBoundingBoxConfig>* dynamic_recfg_;  //!< Dynamic reconfigure server to allow
                                                                            //!< config modifications at runtime
};

}  // namespace costmap_converter

#endif /* COSTMAP_TO_BOUNDING_BOX_H_ */
