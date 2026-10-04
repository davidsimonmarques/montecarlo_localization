#ifndef MONTECARLO_LOCALIZATION_MAP_MODEL_HPP_
#define MONTECARLO_LOCALIZATION_MAP_MODEL_HPP_

#include <vector>
#include <random>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace montecarlo_localization
{

class MapModel
{
public:
  MapModel();
  ~MapModel() = default;

  /**
   * @brief Load and process a new occupancy grid map
   * @param map ROS OccupancyGrid message
   * @param occupied_thresh Threshold above which cell is considered an obstacle [0..100]
   * @param free_thresh Threshold below which cell is considered free [0..100]
   */
  void set_map(const nav_msgs::msg::OccupancyGrid & map, int occupied_thresh = 50, int free_thresh = 20);

  /**
   * @brief Check if a valid map is loaded
   */
  bool is_loaded() const { return is_loaded_; }

  /**
   * @brief Convert world coordinates (meters) to map grid coordinates (indices)
   */
  bool world_to_map(double wx, double wy, int & mx, int & my) const;

  /**
   * @brief Convert map grid coordinates (indices) to world coordinates (meters)
   */
  void map_to_world(int mx, int my, double & wx, double & wy) const;

  /**
   * @brief Check if grid indices are within map boundaries
   */
  bool is_valid_cell(int mx, int my) const;

  /**
   * @brief Check if cell is free space
   */
  bool is_free(int mx, int my) const;

  /**
   * @brief Check if cell is an obstacle
   */
  bool is_occupied(int mx, int my) const;

  /**
   * @brief Query precomputed distance (in meters) to nearest obstacle
   */
  double get_distance_to_obstacle(double wx, double wy) const;

  /**
   * @brief Sample a random free position (x, y in world meters)
   */
  bool sample_free_space(double & wx, double & wy, std::mt19937 & rng) const;

  // Getters
  double resolution() const { return resolution_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }
  double max_distance() const { return max_distance_; }

private:
  /**
   * @brief Computes 2D Euclidean Distance Transform using Felzenszwalb-Huttenlocher algorithm
   */
  void compute_distance_transform();

  bool is_loaded_{false};
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double max_distance_{10.0};
  int occupied_thresh_{50};
  int free_thresh_{20};

  std::vector<int8_t> grid_data_;
  std::vector<float> distance_field_;  // Distance in meters to nearest obstacle for each cell
  std::vector<std::pair<int, int>> free_cells_; // List of free (mx, my) cells for fast random sampling
};

}  // namespace montecarlo_localization

#endif  // MONTECARLO_LOCALIZATION_MAP_MODEL_HPP_
