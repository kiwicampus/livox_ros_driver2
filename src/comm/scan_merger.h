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

#ifndef LIVOX_ROS_DRIVER2_SCAN_MERGER_H_
#define LIVOX_ROS_DRIVER2_SCAN_MERGER_H_

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include "comm/comm.h"

namespace livox_ros {

/**
 * Pairs the scans of several lidars by stamp and concatenates each group into one scan.
 *
 * The scans come from the per-lidar queues of the driver, so their points already are in the common
 * frame of the `extrinsic_parameter` entries of the config file. The oldest scan held leads the next
 * group, and the scans of the other lidars within `pair_tolerance_s` of it join it. It goes out as soon
 * as every other lidar has either joined, or sent a scan newer than its window (so none can join any
 * more: the scans of a lidar arrive in order), or been silent for more than `2 * pair_tolerance_s + 0.2`
 * s (a stopped lidar delays the others by that long once, not for good). Groups therefore leave in
 * stamp order, and a scan waits for its partners at most about one scan period. The one exception is
 * start-up: a lidar is only known from its first scan, so the scans that went out before it was heard of
 * can be newer than that first scan, which then goes out alone.
 *
 * Per group the points closer than `min_range_m` to their own sensor (or not finite) are dropped, the
 * rest are concatenated and kept ordered by their time, and the stamp is the earliest of the group.
 *
 * Not thread safe: call it from the one thread that drains the lidar queues.
 */
class ScanMerger {
 public:
  /** Position of a lidar in the common frame, in metres (the translation of its extrinsic). */
  using Origin = std::array<float, 3>;

  ScanMerger(double pair_tolerance_s, double min_range_m);

  /**
   * Adds the scan of one lidar.
   *
   * @param lidar Index of the lidar in the driver (any stable id works).
   * @param origin Position of that lidar in the common frame; the min range is measured from it.
   * @param scan The scan; its base_time is the stamp used for pairing.
   * @return The merged scans that are ready, oldest first. Possibly empty.
   */
  std::vector<StoragePacket> Add(uint8_t lidar, const Origin &origin, StoragePacket &&scan);

  /** Number of merged scans that had a scan of every lidar seen so far. */
  uint64_t CompleteCount() const { return complete_; }
  /** Number of merged scans that lacked a scan of at least one lidar seen so far. */
  uint64_t PartialCount() const { return partial_; }

 private:
  struct Held {
    StoragePacket scan;
    Origin origin;
  };
  struct Lidar {
    std::deque<Held> held;  // oldest first
    uint64_t newest = 0;    // newest stamp seen from this lidar, ns
  };

  StoragePacket Merge(const std::vector<const Held *> &group) const;
  bool Keep(const PointXyzlt &point, const Origin &origin) const;

  uint64_t tolerance_ns_;
  uint64_t max_wait_ns_;
  double min_range_sq_;
  std::map<uint8_t, Lidar> lidars_;
  uint64_t complete_ = 0;
  uint64_t partial_ = 0;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_SCAN_MERGER_H_
