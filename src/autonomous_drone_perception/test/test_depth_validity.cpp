#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "gtest/gtest.h"
#include "autonomous_drone_perception/depth_validity.hpp"

namespace adp = autonomous_drone_perception;

class DepthValidity : public ::testing::Test
{
protected:
  void SetUp() override
  {
    depth.header.stamp.sec = 42;
    depth.header.frame_id = "stereo_left_optical_frame";
    depth.width = 160;
    depth.height = 32;
    depth.encoding = "32FC1";
    depth.step = depth.width * sizeof(float);
    depth.data.resize(depth.step * depth.height);
    left.header = depth.header;
    left.width = depth.width;
    left.height = depth.height;
    left.encoding = "mono8";
    left.step = left.width;
    left.data.resize(left.width * left.height);
    for (size_t y = 0; y < depth.height; ++y) {
      for (size_t x = 0; x < depth.width; ++x) {
        put(x, y, 2);
        left.data[y * left.width + x] = (x % 2) * 255;
      }
    }
    right = left;
    right.header.frame_id = "stereo_right_optical_frame";
  }
  void put(size_t x, size_t y, float value)
  {
    std::memcpy(depth.data.data() + y * depth.step + x * sizeof(float), &value, sizeof(float));
  }
  float get(size_t x, size_t y)
  {
    float value;
    std::memcpy(&value, depth.data.data() + y * depth.step + x * sizeof(float), sizeof(float));
    return value;
  }
  void apply() {adp::reject_unsupported_depth(depth, left, right, 48, 2);}
  sensor_msgs::msg::Image depth, left, right;
};

TEST_F(DepthValidity, PreservesSupportedScaleStampAndNaNs)
{
  put(80, 16, NAN);
  const auto header = depth.header;
  apply();
  EXPECT_FLOAT_EQ(get(90, 16), 2);
  EXPECT_TRUE(std::isnan(get(80, 16)));
  EXPECT_TRUE(std::isnan(get(30, 16)));
  EXPECT_TRUE(std::isnan(get(60, 16)));  // Estimated match fits, full search does not.
  EXPECT_TRUE(std::isnan(get(90, 0)));
  EXPECT_EQ(depth.header, header);
}

TEST_F(DepthValidity, RejectsUniformAndHorizontalStripeAmbiguityInEitherView)
{
  for (size_t y = 0; y < right.height; ++y) {
    for (size_t x = 0; x < right.width; ++x) {
      right.data[y * right.width + x] = (y % 2) * 255;
    }
  }
  apply();
  EXPECT_TRUE(std::isnan(get(90, 16)));
  SetUp();
  std::fill(left.data.begin(), left.data.end(), 100);
  apply();
  EXPECT_TRUE(std::isnan(get(90, 16)));
}

TEST_F(DepthValidity, KeepsDisparityDiscontinuityUnknownWithOcclusionMargin)
{
  for (size_t y = 0; y < depth.height; ++y) {
    for (size_t x = 0; x < 100; ++x) {put(x, y, 1.6f);}
  }
  apply();
  EXPECT_TRUE(std::isnan(get(100, 16)));
  EXPECT_TRUE(std::isnan(get(105, 16)));
  EXPECT_FLOAT_EQ(get(130, 16), 2);
  EXPECT_FLOAT_EQ(get(80, 16), 1.6f);
}

TEST_F(DepthValidity, RejectsMismatchedSourceStampFrameAndBuffer)
{
  right.header.stamp.nanosec = 1;
  EXPECT_THROW(apply(), std::invalid_argument);
  right.header.stamp = depth.header.stamp;
  right.header.frame_id = left.header.frame_id;
  EXPECT_THROW(apply(), std::invalid_argument);
  right.header.frame_id = "stereo_right_optical_frame";
  right.data.pop_back();
  EXPECT_THROW(apply(), std::invalid_argument);
}
