//
// The MIT License (MIT)
//
// Copyright (c) 2026 robot.com. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "comm/scan_merger.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace livox_ros {

namespace {

// A lidar that sent nothing for 2 * tolerance + this much is no longer waited for.
constexpr double kExtraWaitS = 0.2;

uint64_t ToNs(double seconds) {
  return static_cast<uint64_t>(std::max(seconds, 0.0) * static_cast<double>(kNsPerSecond));
}

uint64_t AbsDiff(uint64_t a, uint64_t b) {
  return a > b ? a - b : b - a;
}

bool ByTime(const PointXyzlt &a, const PointXyzlt &b) {
  return a.offset_time < b.offset_time;
}

}  // namespace

ScanMerger::ScanMerger(double pair_tolerance_s, double min_range_m)
    : tolerance_ns_(ToNs(pair_tolerance_s)),
      max_wait_ns_(ToNs(2.0 * pair_tolerance_s + kExtraWaitS)),
      min_range_sq_(min_range_m > 0.0 ? min_range_m * min_range_m : 0.0) {}

std::vector<StoragePacket> ScanMerger::Add(uint8_t lidar, const Origin &origin, StoragePacket &&scan) {
  Lidar &entry = lidars_[lidar];
  entry.newest = std::max(entry.newest, scan.base_time);
  entry.held.push_back(Held{std::move(scan), origin});

  std::vector<StoragePacket> ready;
  for (;;) {
    // The oldest scan still held leads the next group; the other lidars are matched against it.
    Lidar *anchor_lidar = nullptr;
    uint64_t newest = 0;
    for (auto &item : lidars_) {
      Lidar &candidate = item.second;
      newest = std::max(newest, candidate.newest);
      if (candidate.held.empty()) {
        continue;
      }
      if (anchor_lidar == nullptr ||
          candidate.held.front().scan.base_time < anchor_lidar->held.front().scan.base_time) {
        anchor_lidar = &candidate;
      }
    }
    if (anchor_lidar == nullptr) {
      break;
    }
    const uint64_t anchor_stamp = anchor_lidar->held.front().scan.base_time;

    // One scan per lidar: the anchor itself, and for each other lidar its first held scan within tolerance.
    // A lidar without one is waited for only while its scan may still come: it has not yet sent anything
    // newer than the window of the anchor (the scans of a lidar arrive in order) and it has not gone silent.
    std::vector<std::pair<Lidar *, size_t>> picks;
    bool waiting = false;
    for (auto &item : lidars_) {
      Lidar &candidate = item.second;
      if (&candidate == anchor_lidar) {
        picks.emplace_back(&candidate, 0);
        continue;
      }
      bool matched = false;
      for (size_t i = 0; i < candidate.held.size(); ++i) {
        if (AbsDiff(candidate.held[i].scan.base_time, anchor_stamp) <= tolerance_ns_) {
          picks.emplace_back(&candidate, i);
          matched = true;
          break;
        }
      }
      const bool silent = candidate.newest + max_wait_ns_ < newest;
      const bool may_still_come = candidate.newest <= anchor_stamp + tolerance_ns_;
      if (!matched && may_still_come && !silent) {
        waiting = true;
        break;
      }
    }
    if (waiting) {
      break;
    }

    std::vector<const Held *> group;
    group.reserve(picks.size());
    for (const auto &pick : picks) {
      group.push_back(&pick.first->held[pick.second]);
    }
    StoragePacket merged = Merge(group);
    (picks.size() == lidars_.size() ? complete_ : partial_) += 1;
    for (const auto &pick : picks) {
      pick.first->held.erase(pick.first->held.begin() + static_cast<std::ptrdiff_t>(pick.second));
    }
    if (!merged.points.empty()) {
      ready.push_back(std::move(merged));
    }
  }
  return ready;
}

StoragePacket ScanMerger::Merge(const std::vector<const Held *> &group) const {
  StoragePacket merged{};
  merged.lidar_type = group.front()->scan.lidar_type;
  merged.handle = group.front()->scan.handle;
  merged.base_time = group.front()->scan.base_time;
  size_t total = 0;
  for (const Held *held : group) {
    merged.base_time = std::min(merged.base_time, held->scan.base_time);
    total += held->scan.points.size();
  }
  merged.points.reserve(total);

  // The points of a scan are in the order they were received, so each block is already ordered by time
  // and one linear merge keeps the whole cloud ordered (the deskew of a consumer takes the first and
  // last point as the start and end of the scan). The sort is only a guard for a block that is not.
  for (const Held *held : group) {
    const auto block = static_cast<std::ptrdiff_t>(merged.points.size());
    for (const PointXyzlt &point : held->scan.points) {
      if (Keep(point, held->origin)) {
        merged.points.push_back(point);
      }
    }
    if (!std::is_sorted(merged.points.begin() + block, merged.points.end(), ByTime)) {
      std::stable_sort(merged.points.begin() + block, merged.points.end(), ByTime);
    }
    std::inplace_merge(merged.points.begin(), merged.points.begin() + block, merged.points.end(), ByTime);
  }
  merged.points_num = static_cast<uint32_t>(merged.points.size());
  return merged;
}

bool ScanMerger::Keep(const PointXyzlt &point, const Origin &origin) const {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }
  const double dx = static_cast<double>(point.x) - static_cast<double>(origin[0]);
  const double dy = static_cast<double>(point.y) - static_cast<double>(origin[1]);
  const double dz = static_cast<double>(point.z) - static_cast<double>(origin[2]);
  return dx * dx + dy * dy + dz * dz >= min_range_sq_;
}

}  // namespace livox_ros
