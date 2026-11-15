#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>
#include "gtest/gtest.h"
#include "autonomous_drone_perception/depth_conversion.hpp"

namespace adp = autonomous_drone_perception;

class DepthConversion : public ::testing::Test
{
protected:
  void SetUp() override
  {
    left.header.stamp.sec = 42;
    left.header.frame_id = limits.left_frame;
    left.width = 8;
    left.height = 1;
    left.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    left.p = {400, 0, 320, 0, 0, 400, 240, 0, 0, 0, 1, 0};
    right = left;
    right.header.frame_id = limits.right_frame;
    right.p[3] = -48;
    disparity.header = right.header;
    disparity.f = 400;
    disparity.t = 48;  // Known 4.6 metadata issue: pixels*metres instead of metres.
    disparity.min_disparity = 0;
    disparity.max_disparity = 64;
    disparity.image.width = 8;
    disparity.image.height = 1;
    disparity.image.encoding = "32FC1";
    disparity.image.step = 8 * sizeof(float);
    pixels({48, 24, 12, 0, -1, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), 80});
  }
  void pixels(std::vector<float> values)
  {
    const size_t byte_count = values.size() * sizeof(float);
    disparity.image.data.resize(byte_count);
    std::memcpy(disparity.image.data.data(), values.data(), byte_count);
  }
  std::vector<float> convert()
  {
    auto result = adp::metric_depth(disparity, left, right, limits);
    EXPECT_EQ(result.header, left.header);
    EXPECT_EQ(result.encoding, "32FC1");
    std::vector<float> values(result.width * result.height);
    std::memcpy(values.data(), result.data.data(), result.data.size());
    return values;
  }
  adp::DepthLimits limits;
  sensor_msgs::msg::CameraInfo left, right;
  stereo_msgs::msg::DisparityImage disparity;
};

TEST_F(DepthConversion, UsesCalibratedBaselineAndLeftCaptureFrame)
{
  auto z = convert();
  EXPECT_FLOAT_EQ(z[0], 1);
  EXPECT_FLOAT_EQ(z[1], 2);
  EXPECT_FLOAT_EQ(z[2], 4);
  for (size_t i = 3; i < z.size(); ++i) {EXPECT_TRUE(std::isnan(z[i]));}
  disparity.t = 0.12f;  // Also works if the upstream bug is fixed.
  EXPECT_FLOAT_EQ(convert()[1], 2);
}
TEST_F(DepthConversion, MasksOutOfRangeInsteadOfClippingOrClearing)
{
  limits.min_depth_m = 1.5;
  limits.max_depth_m = 3;
  auto z = convert();
  EXPECT_TRUE(std::isnan(z[0]));
  EXPECT_FLOAT_EQ(z[1], 2);
  EXPECT_TRUE(std::isnan(z[2]));
}
TEST_F(DepthConversion, RejectsCalibrationFromAnotherCapture)
{
  right.header.stamp.nanosec = 1;
  EXPECT_THROW(convert(), std::invalid_argument);
}
TEST_F(DepthConversion, RejectsMissingOrReversedBaseline)
{
  right.p[3] = 0;
  EXPECT_THROW(convert(), std::invalid_argument);
  right.p[3] = 48;
  EXPECT_THROW(convert(), std::invalid_argument);
}
TEST_F(DepthConversion, RejectsInconsistentRectifiedProjections)
{
  right.p[2] = 319;
  EXPECT_THROW(convert(), std::invalid_argument);
  right.p[2] = 320;
  right.r[0] = 0;
  EXPECT_THROW(convert(), std::invalid_argument);
}
TEST_F(DepthConversion, RejectsMalformedBuffers)
{
  disparity.image.data.pop_back();
  EXPECT_THROW(convert(), std::invalid_argument);
}
TEST_F(DepthConversion, RejectsUnexpectedFrameAndEncoding)
{
  disparity.header.frame_id = "world";
  EXPECT_THROW(convert(), std::invalid_argument);
  disparity.header.frame_id = limits.right_frame;
  disparity.image.encoding = "16SC1";
  EXPECT_THROW(convert(), std::invalid_argument);
}
TEST_F(DepthConversion, AppliesAdvertisedValidWindow)
{
  disparity.valid_window.x_offset = 1;
  disparity.valid_window.width = 2;
  disparity.valid_window.height = 1;
  auto z = convert();
  EXPECT_TRUE(std::isnan(z[0]));
  EXPECT_FLOAT_EQ(z[1], 2);
  disparity.valid_window.width = 100;
  EXPECT_THROW(convert(), std::invalid_argument);
}
TEST_F(DepthConversion, HandlesPaddedRows)
{
  disparity.image.step += 4;
  disparity.image.data.resize(disparity.image.step, 0);
  EXPECT_FLOAT_EQ(convert()[2], 4);
}
