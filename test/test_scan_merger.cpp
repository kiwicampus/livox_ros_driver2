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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "comm/scan_merger.h"

namespace livox_ros {
namespace {

constexpr uint64_t kMs = 1000000;  // ns
constexpr uint64_t kT0 = 1700000000ULL * 1000 * kMs;
const ScanMerger::Origin kAtZero = {0.0f, 0.0f, 0.0f};

PointXyzlt Point(float x, float y, float z, uint64_t offset_time) {
  PointXyzlt point = {};
  point.x = x;
  point.y = y;
  point.z = z;
  point.offset_time = offset_time;
  return point;
}

StoragePacket Scan(uint64_t base_time, std::vector<PointXyzlt> points) {
  StoragePacket scan = {};
  scan.base_time = base_time;
  scan.points_num = static_cast<uint32_t>(points.size());
  scan.points = std::move(points);
  return scan;
}

// A scan of `count` points about 1.4 m from the origin, 1 ms apart from `base_time`.
StoragePacket Sweep(uint64_t base_time, int count, float x) {
  std::vector<PointXyzlt> points;
  for (int i = 0; i < count; ++i) {
    points.push_back(Point(x, 1.0f, 0.0f, base_time + static_cast<uint64_t>(i) * kMs));
  }
  return Scan(base_time, std::move(points));
}

// Brings two lidars to their steady state: lidar 0's scan at kT0 + 100 ms is held, waiting for lidar 1's.
// Lidar 0's first scan went out alone (lidar 1 was not known yet), and so did lidar 1's (lidar 0's next scan
// showed that no partner for it was coming), which is the only start-up artefact.
void Prime(ScanMerger &merger) {
  merger.Add(0, kAtZero, Sweep(kT0, 1, 1.0f));
  merger.Add(1, kAtZero, Sweep(kT0, 1, -1.0f));
  merger.Add(0, kAtZero, Sweep(kT0 + 100 * kMs, 2, 1.0f));
}

TEST(ScanMerger, SingleLidarGoesOutAtOnce) {
  ScanMerger merger(0.05, 0.0);
  auto out = merger.Add(0, kAtZero, Sweep(kT0, 3, 1.0f));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].points.size(), 3u);
  EXPECT_EQ(out[0].points_num, 3u);
  EXPECT_EQ(out[0].base_time, kT0);
  EXPECT_EQ(merger.PartialCount(), 0u);
}

TEST(ScanMerger, JoinsTheScansOfTwoLidars) {
  ScanMerger merger(0.05, 0.0);
  Prime(merger);

  auto out = merger.Add(1, kAtZero, Sweep(kT0 + 103 * kMs, 2, -1.0f));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].points.size(), 4u);
  EXPECT_EQ(out[0].points_num, 4u);
  EXPECT_EQ(out[0].base_time, kT0 + 100 * kMs);  // the earlier stamp of the pair
  EXPECT_EQ(merger.CompleteCount(), 2u);         // lidar 0's first scan, alone while it was the only one, and this
  EXPECT_EQ(merger.PartialCount(), 1u);          // lidar 1's first scan
}

TEST(ScanMerger, HoldsAScanOnlyUntilItsPartnerArrives) {
  ScanMerger merger(0.05, 0.0);
  Prime(merger);

  EXPECT_TRUE(merger.Add(0, kAtZero, Sweep(kT0 + 200 * kMs, 2, 1.0f)).empty());  // lidar 1 may still come
  EXPECT_EQ(merger.Add(1, kAtZero, Sweep(kT0 + 102 * kMs, 2, -1.0f)).size(), 1u);  // joins the 100 ms scan
  EXPECT_EQ(merger.Add(1, kAtZero, Sweep(kT0 + 201 * kMs, 2, -1.0f)).size(), 1u);  // and the 200 ms one
}

TEST(ScanMerger, JoinedPointsStayOrderedByTime) {
  ScanMerger merger(0.05, 0.0);
  merger.Add(0, kAtZero, Sweep(kT0, 1, 1.0f));
  merger.Add(1, kAtZero, Sweep(kT0, 1, -1.0f));
  // Interleaved in time: lidar 0 at even ms, lidar 1 at odd ms.
  std::vector<PointXyzlt> a, b;
  for (uint64_t i = 0; i < 4; ++i) {
    a.push_back(Point(1.0f, 0.0f, 0.0f, kT0 + 100 * kMs + 2 * i * kMs));
    b.push_back(Point(-1.0f, 0.0f, 0.0f, kT0 + 101 * kMs + 2 * i * kMs));
  }
  merger.Add(0, kAtZero, Scan(kT0 + 100 * kMs, a));
  auto out = merger.Add(1, kAtZero, Scan(kT0 + 101 * kMs, b));
  ASSERT_EQ(out.size(), 1u);
  ASSERT_EQ(out[0].points.size(), 8u);
  for (size_t i = 1; i < out[0].points.size(); ++i) {
    EXPECT_LE(out[0].points[i - 1].offset_time, out[0].points[i].offset_time) << "at " << i;
  }
  EXPECT_GT(out[0].points[0].x, 0.0f);  // they alternate between the lidars
  EXPECT_LT(out[0].points[1].x, 0.0f);
  EXPECT_EQ(out[0].base_time, kT0 + 100 * kMs);
}

TEST(ScanMerger, ScansFurtherApartThanTheToleranceAreNotJoined) {
  // Lidar 1 lags lidar 0 by 40 ms with a 20 ms tolerance: the clocks are not in sync, so nothing pairs and
  // every scan goes out alone, as soon as the next scan of the other lidar shows that.
  ScanMerger merger(0.02, 0.0);
  size_t out_count = 0;
  for (uint64_t frame = 0; frame < 10; ++frame) {
    for (uint8_t lidar : {0, 1}) {
      const uint64_t stamp = kT0 + frame * 100 * kMs + (lidar == 1 ? 40 * kMs : 0);
      for (const auto &scan : merger.Add(lidar, kAtZero, Sweep(stamp, 1, 1.0f))) {
        EXPECT_EQ(scan.points.size(), 1u);
        ++out_count;
      }
    }
  }
  EXPECT_GE(out_count, 18u);
  EXPECT_EQ(merger.CompleteCount(), 1u);  // only lidar 0's first scan, while it was alone
  EXPECT_GE(merger.PartialCount(), 17u);
}

TEST(ScanMerger, ALidarThatWentSilentIsWaitedForOnlyOnce) {
  ScanMerger merger(0.05, 0.0);
  Prime(merger);

  // Lidar 1 stops while lidar 0 keeps going at 10 Hz. Its scans are held while lidar 1 may still send
  // (up to 2 * 0.05 + 0.2 = 0.3 s after its last), then all go out, and later ones leave as they arrive.
  for (uint64_t step = 1; step <= 20; ++step) {
    auto out = merger.Add(0, kAtZero, Sweep(kT0 + 100 * kMs + step * 100 * kMs, 1, 1.0f));
    const size_t expected = step < 3 ? 0u : (step == 3 ? 4u : 1u);
    EXPECT_EQ(out.size(), expected) << "step " << step;
  }
}

TEST(ScanMerger, DropsPointsCloserThanTheMinRangeToTheirOwnSensor) {
  // Lidar 1 sits 2 m along x, so its point at x = 2.3 is 0.3 m from it (dropped) although 2.3 m from the
  // origin, while lidar 0's point at x = 2.3 is far from it (kept).
  ScanMerger merger(0.05, 0.6);
  const ScanMerger::Origin second = {2.0f, 0.0f, 0.0f};
  merger.Add(1, second, Scan(kT0, {Point(2.0f, 1.0f, 0.0f, kT0)}));
  merger.Add(0, kAtZero, Scan(kT0, {Point(1.0f, 0.0f, 0.0f, kT0)}));
  merger.Add(1, second,
             Scan(kT0 + 100 * kMs, {Point(2.3f, 0.0f, 0.0f, kT0 + 100 * kMs),    // 0.3 m from lidar 1: out
                                    Point(2.0f, 1.0f, 0.0f, kT0 + 101 * kMs)}));  // 1.0 m: in
  auto out = merger.Add(0, kAtZero,
                        Scan(kT0 + 100 * kMs, {Point(2.3f, 0.0f, 0.0f, kT0 + 102 * kMs),    // 2.3 m from lidar 0: in
                                               Point(0.2f, 0.0f, 0.0f, kT0 + 103 * kMs)}));  // 0.2 m: out
  ASSERT_EQ(out.size(), 1u);
  ASSERT_EQ(out[0].points.size(), 2u);
  EXPECT_FLOAT_EQ(out[0].points[0].y, 1.0f);
  EXPECT_FLOAT_EQ(out[0].points[1].x, 2.3f);
  EXPECT_EQ(out[0].points_num, 2u);
}

TEST(ScanMerger, DropsPointsThatAreNotFinite) {
  ScanMerger merger(0.05, 0.0);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  auto out = merger.Add(0, kAtZero,
                        Scan(kT0, {Point(1.0f, 0.0f, 0.0f, kT0), Point(nan, 0.0f, 0.0f, kT0 + kMs),
                                   Point(0.0f, inf, 0.0f, kT0 + 2 * kMs), Point(0.0f, 0.0f, 2.0f, kT0 + 3 * kMs)}));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].points.size(), 2u);
}

TEST(ScanMerger, ABlockOutOfOrderIsStillSortedInTheOutput) {
  ScanMerger merger(0.05, 0.0);
  auto out = merger.Add(0, kAtZero,
                        Scan(kT0, {Point(1.0f, 0.0f, 0.0f, kT0 + 3 * kMs), Point(1.0f, 0.0f, 0.0f, kT0 + kMs),
                                   Point(1.0f, 0.0f, 0.0f, kT0 + 2 * kMs)}));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].points[0].offset_time, kT0 + kMs);
  EXPECT_EQ(out[0].points[1].offset_time, kT0 + 2 * kMs);
  EXPECT_EQ(out[0].points[2].offset_time, kT0 + 3 * kMs);
}

TEST(ScanMerger, ThreeLidarsAreJoinedIntoOne) {
  ScanMerger merger(0.05, 0.0);
  std::vector<StoragePacket> last;
  for (uint64_t frame = 0; frame < 10; ++frame) {
    for (uint8_t lidar : {0, 1, 2}) {
      auto out = merger.Add(lidar, kAtZero, Sweep(kT0 + frame * 100 * kMs + lidar * 2 * kMs, 2, 1.0f));
      if (!out.empty()) {
        last = std::move(out);
      }
    }
  }
  ASSERT_EQ(last.size(), 1u);
  EXPECT_EQ(last[0].points.size(), 6u);
  EXPECT_EQ(last[0].base_time, kT0 + 900 * kMs);
  EXPECT_GE(merger.CompleteCount(), 8u);
}

TEST(ScanMerger, OutputStampsNeverGoBackwardsOnceStarted) {
  // Only the first scan of a lidar the merger had not heard of can be older than scans already sent (lidar
  // 0's first scan goes out alone, and lidar 1's, stamped a bit earlier, follows), so start-up is skipped.
  ScanMerger merger(0.05, 0.0);
  uint64_t previous = 0;
  size_t seen = 0;
  size_t joined = 0;
  for (uint64_t frame = 0; frame < 30; ++frame) {
    // Lidar 1 jitters around lidar 0 by a few ms, and sometimes arrives first.
    const uint64_t jitter = (frame % 2 == 0) ? 3 * kMs : 0;
    const uint64_t t0 = kT0 + frame * 100 * kMs + 5 * kMs;
    const uint64_t t1 = kT0 + frame * 100 * kMs + 5 * kMs - jitter + (frame % 3) * kMs;
    std::vector<std::pair<uint8_t, uint64_t>> arrivals = {{0, t0}, {1, t1}};
    if (frame % 4 == 1) {
      std::swap(arrivals[0], arrivals[1]);
    }
    for (const auto &arrival : arrivals) {
      for (const auto &scan : merger.Add(arrival.first, kAtZero, Sweep(arrival.second, 2, 1.0f))) {
        if (++seen > 2) {
          EXPECT_GE(scan.base_time, previous);
        }
        previous = scan.base_time;
        joined += scan.points.size() == 4 ? 1 : 0;
      }
    }
  }
  EXPECT_GE(joined, 27u);
}

TEST(ScanMerger, AScanWithNoPointsLeftIsNotPublished) {
  ScanMerger merger(0.05, 1.0);
  auto out = merger.Add(0, kAtZero, Scan(kT0, {Point(0.1f, 0.0f, 0.0f, kT0)}));
  EXPECT_TRUE(out.empty());
}

}  // namespace
}  // namespace livox_ros
