#pragma once
#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace costmap_2d {

struct MapLocation
{
  unsigned int x;
  unsigned int y;
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
class Costmap2D
{
public:
  Costmap2D() {}
  Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
            double origin_x, double origin_y, unsigned char default_value = 0) {}
  Costmap2D(const Costmap2D& map) {}
  Costmap2D& operator=(const Costmap2D& map) { return *this; }
  virtual ~Costmap2D() {}

  // @brief  Turn this costmap into a copy of a window of a costmap passed in
  bool copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                         double win_size_y) { return false; }



  // @brief  Get the cost of a cell in the costmap
  unsigned char getCost(unsigned int mx, unsigned int my) const { return 0; }

  // @brief  Set the cost of a cell in the costmap
  void setCost(unsigned int mx, unsigned int my, unsigned char cost) {}

  // @brief  Convert from map coordinates to world coordinates
  void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {}

  // @brief  Convert from world coordinates to map coordinates
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const { return false; }

  // @brief  Convert from world coordinates to map coordinates without checking for legal bounds
  void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const {}

  // @brief  Convert from world coordinates to map coordinates, constraining results to legal bounds.
  void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const {}

  // @brief  Given two map coordinates... compute the associated index
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return 0;
  }

  // @brief  Given an index... compute the associated map coordinates
  inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const {}

  // @brief  Will return a pointer to the underlying unsigned char array used as the costmap
  unsigned char* getCharMap() const { return NULL; }

  // @brief  Accessor for the x size of the costmap in cells
  unsigned int getSizeInCellsX() const { return 0; }

  // @brief  Accessor for the y size of the costmap in cells
  unsigned int getSizeInCellsY() const { return 0; }

  // @brief  Accessor for the x size of the costmap in meters
  double getSizeInMetersX() const { return 0; }

  // @brief  Accessor for the y size of the costmap in meters
  double getSizeInMetersY() const { return 0; }

  // @brief  Accessor for the x origin of the costmap
  double getOriginX() const { return 0; }

  // @brief  Accessor for the y origin of the costmap
  double getOriginY() const { return 0; }

  // @brief  Accessor for the resolution of the costmap
  double getResolution() const { return 0; }

  void setDefaultValue(unsigned char c) {}

  unsigned char getDefaultValue()
  {
    return 0;
  }

  // @brief  Sets the cost of a convex polygon to a desired value
  bool setConvexPolygonCost(const std::vector<geometry_msgs::msg::Point>& polygon, unsigned char cost_value) { return false; }

  // @brief  Get the map cells that make up the outline of a polygon
  void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells) {}

  // @brief  Get the map cells that fill a convex polygon
  void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells) {}

  // @brief  Move the origin of the costmap to a new location.... keeping data when it can
  virtual void updateOrigin(double new_origin_x, double new_origin_y) {}

  // @brief  Save the costmap out to a pgm file
  bool saveMap(std::string file_name) { return false; }

  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                 double origin_y) {}

  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn) {}

  // @brief  Given distance in the world... convert it to cells
  unsigned int cellDistance(double world_dist) { return 0; }
protected:
  unsigned int size_x_;
  unsigned int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  unsigned char* costmap_;
  unsigned char default_value_;

};
#pragma GCC diagnostic pop

}
