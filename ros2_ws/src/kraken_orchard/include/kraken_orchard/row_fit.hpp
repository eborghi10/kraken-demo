// SPDX-License-Identifier: BSD-3-Clause
#ifndef KRAKEN_ORCHARD__ROW_FIT_HPP_
#define KRAKEN_ORCHARD__ROW_FIT_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "kraken_orchard/cloud_filter.hpp"

namespace kraken_orchard
{

struct RowFitOptions
{
  double max_lateral{3.0};
  std::size_t min_trunks{3};
  double min_span{4.0};
  double max_residual{0.35};
  double max_heading_difference{0.15};
};

/// What the trunks either side say about the corridor between them.
struct RowFit
{
  bool valid{false};
  std::string reason;
  double width{0.0};
  double lateral_offset{0.0};
  double row_heading{0.0};
  int left_trunks{0};
  int right_trunks{0};
};

/// Measure the aisle from the trunk centres standing either side of it.
///
/// Positions are in the sensor's frame. The Kraken's lidar is bolted on
/// without rotation, so the offset and heading read directly as the robot's
/// error against the row.
RowFit fitRow(const std::vector<Trunk> & trunks, const RowFitOptions & options = {});

}  // namespace kraken_orchard

#endif  // KRAKEN_ORCHARD__ROW_FIT_HPP_
