#include "montecarlo_localization/map_model.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

namespace montecarlo_localization
{

static constexpr float INF_DIST = 1e9f;

MapModel::MapModel()
{
}

void MapModel::set_map(const nav_msgs::msg::OccupancyGrid & map, int occupied_thresh, int free_thresh)
{
  width_ = static_cast<int>(map.info.width);
  height_ = static_cast<int>(map.info.height);
  resolution_ = map.info.resolution;
  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;
  occupied_thresh_ = occupied_thresh;
  free_thresh_ = free_thresh;

  grid_data_ = map.data;
  distance_field_.assign(width_ * height_, INF_DIST);
  free_cells_.clear();

  if (width_ <= 0 || height_ <= 0) {
    is_loaded_ = false;
    return;
  }

  // Identify free cells
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int idx = y * width_ + x;
      int8_t val = grid_data_[idx];
      if (val >= 0 && val <= free_thresh_) {
        free_cells_.emplace_back(x, y);
      }
    }
  }

  compute_distance_transform();
  is_loaded_ = true;
}

bool MapModel::world_to_map(double wx, double wy, int & mx, int & my) const
{
  if (!is_loaded_) {
    return false;
  }
  mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return is_valid_cell(mx, my);
}

void MapModel::map_to_world(int mx, int my, double & wx, double & wy) const
{
  wx = origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_;
  wy = origin_y_ + (static_cast<double>(my) + 0.5) * resolution_;
}

bool MapModel::is_valid_cell(int mx, int my) const
{
  return (mx >= 0 && mx < width_ && my >= 0 && my < height_);
}

bool MapModel::is_free(int mx, int my) const
{
  if (!is_valid_cell(mx, my)) {
    return false;
  }
  int8_t val = grid_data_[my * width_ + mx];
  return (val >= 0 && val <= free_thresh_);
}

bool MapModel::is_occupied(int mx, int my) const
{
  if (!is_valid_cell(mx, my)) {
    return false;
  }
  int8_t val = grid_data_[my * width_ + mx];
  return (val >= occupied_thresh_);
}

double MapModel::get_distance_to_obstacle(double wx, double wy) const
{
  if (!is_loaded_) {
    return max_distance_;
  }

  int mx, my;
  if (!world_to_map(wx, wy, mx, my)) {
    // Outside map boundary
    return max_distance_;
  }

  float dist = distance_field_[my * width_ + mx];
  return static_cast<double>(dist);
}

bool MapModel::sample_free_space(double & wx, double & wy, std::mt19937 & rng) const
{
  if (free_cells_.empty()) {
    return false;
  }

  std::uniform_int_distribution<size_t> dist(0, free_cells_.size() - 1);
  const auto & cell = free_cells_[dist(rng)];

  std::uniform_real_distribution<double> jitter(-0.5, 0.5);
  wx = origin_x_ + (static_cast<double>(cell.first) + 0.5 + jitter(rng)) * resolution_;
  wy = origin_y_ + (static_cast<double>(cell.second) + 0.5 + jitter(rng)) * resolution_;
  return true;
}

// 1D Felzenszwalb-Huttenlocher Distance Transform
static void dt_1d(const std::vector<float> & f, std::vector<float> & d, int n)
{
  std::vector<int> v(n);
  std::vector<float> z(n + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -INF_DIST;
  z[1] = INF_DIST;

  for (int q = 1; q < n; ++q) {
    float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = INF_DIST;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    float diff = static_cast<float>(q - v[k]);
    d[q] = diff * diff + f[v[k]];
  }
}

void MapModel::compute_distance_transform()
{
  // Initialize squared distance matrix
  std::vector<float> d_sq(width_ * height_, INF_DIST);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      int idx = y * width_ + x;
      if (grid_data_[idx] >= occupied_thresh_) {
        d_sq[idx] = 0.0f;
      }
    }
  }

  // Transform along columns
  std::vector<float> col_in(height_);
  std::vector<float> col_out(height_);
  for (int x = 0; x < width_; ++x) {
    for (int y = 0; y < height_; ++y) {
      col_in[y] = d_sq[y * width_ + x];
    }
    dt_1d(col_in, col_out, height_);
    for (int y = 0; y < height_; ++y) {
      d_sq[y * width_ + x] = col_out[y];
    }
  }

  // Transform along rows
  std::vector<float> row_in(width_);
  std::vector<float> row_out(width_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      row_in[x] = d_sq[y * width_ + x];
    }
    dt_1d(row_in, row_out, width_);
    for (int x = 0; x < width_; ++x) {
      float dist_metric = std::sqrt(row_out[x]) * static_cast<float>(resolution_);
      distance_field_[y * width_ + x] = std::min(dist_metric, static_cast<float>(max_distance_));
    }
  }
}

}  // namespace montecarlo_localization
