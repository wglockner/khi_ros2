// Copyright 2025 Kawasaki Heavy Industries, Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// OFFLINE tests of the extended KRNX services: the driver is never opened, so nothing here
// reaches a robot controller. Covers the pure-math Matrix-API, the refusal paths, and that
// controller-indexed calls fail cleanly (not crash) without a connection.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "khi_hardware/khi_krnx_driver.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_msgs/srv/forward_kinematics.hpp"
#include "khi_msgs/srv/get_frame.hpp"
#include "khi_msgs/srv/inverse_kinematics.hpp"
#include "khi_msgs/srv/program_control.hpp"
#include "khi_msgs/srv/set_io.hpp"
#include "khi_msgs/srv/set_panel_info.hpp"
#include "khi_msgs/srv/transform_math.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using namespace std::chrono_literals;  // NOLINT(build/namespaces)
namespace srv = khi_msgs::srv;

constexpr size_t EXTENDED_SERVICE_COUNT = 37;
constexpr float TOLERANCE = 1e-3F;

class KrnxApiServicesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void start(const bool enable_undocumented)
  {
    khi_hardware::KhiRobot robot;
    robot.name = "test";
    robot.ip_address = "0.0.0.0";
    robot.period = 4;
    // A controller slot no real cell on this host uses, to stay clear of any live driver.
    robot.controller_no = 7;
    robot.arms.resize(1);
    robot.enable_undocumented_krnx_api = enable_undocumented;
    driver_ = std::make_shared<khi_hardware::KhiKrnxDriver>(robot, khi_hardware::KhiPeriodicDataConfig{});
    node_ = rclcpp::Node::make_shared("krnx_api_services_test", "/khi_controller7");
    driver_->create_extended_services(*node_, services_);
  }

  void TearDown() override
  {
    services_.clear();
    node_.reset();
    driver_.reset();
  }

  template <typename SrvT>
  std::shared_ptr<typename SrvT::Response> call(
    const std::string & name, const std::shared_ptr<typename SrvT::Request> & request)
  {
    auto client = node_->create_client<SrvT>("~/" + name);
    EXPECT_TRUE(client->wait_for_service(2s)) << name;
    auto future = client->async_send_request(request);
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(node_, future, 5s), rclcpp::FutureReturnCode::SUCCESS)
      << name;
    return future.get();
  }

  std::shared_ptr<khi_hardware::KhiKrnxDriver> driver_;
  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
};

TEST_F(KrnxApiServicesTest, CreatesEveryService)
{
  start(false);
  EXPECT_EQ(services_.size(), EXTENDED_SERVICE_COUNT);
}

TEST_F(KrnxApiServicesTest, TransformTimesInverseIsIdentity)
{
  start(false);
  const std::array<float, 6> pose = {100.0F, 200.0F, 300.0F, 0.4F, 1.1F, -0.7F};

  auto inverse_req = std::make_shared<srv::TransformMath::Request>();
  inverse_req->op = srv::TransformMath::Request::OP_INVERSE;
  inverse_req->xyzoat_a = pose;
  const auto inverse = call<srv::TransformMath>("transform_math", inverse_req);
  ASSERT_TRUE(inverse->result.success) << inverse->result.krnx_err;

  auto multiply_req = std::make_shared<srv::TransformMath::Request>();
  multiply_req->op = srv::TransformMath::Request::OP_MULTIPLY;
  multiply_req->xyzoat_a = pose;
  multiply_req->xyzoat_b = inverse->xyzoat;
  const auto identity = call<srv::TransformMath>("transform_math", multiply_req);
  ASSERT_TRUE(identity->result.success) << identity->result.krnx_err;

  // Identity as a 3x4 matrix: n=(1,0,0) o=(0,1,0) a=(0,0,1) p=(0,0,0). Compared as a matrix
  // because the Euler angles of an identity rotation are not unique (O + T = 0).
  const std::array<float, 12> expected = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
  for (size_t i = 0; i < expected.size(); i++)
  {
    EXPECT_NEAR(identity->matrix[i], expected[i], TOLERANCE) << "matrix[" << i << "]";
  }
}

TEST_F(KrnxApiServicesTest, MatrixLayoutIsColumnMajorRadians)
{
  start(false);
  // O = +90 deg about Z, in RADIANS: X axis n -> (0,1,0), Y axis o -> (-1,0,0). KRNX API
  // Manual 9.1 / 9.2.
  auto req = std::make_shared<srv::TransformMath::Request>();
  req->op = srv::TransformMath::Request::OP_CONVERT;
  req->xyzoat_a = {10.0F, 20.0F, 30.0F, static_cast<float>(M_PI / 2), 0.0F, 0.0F};
  const auto resp = call<srv::TransformMath>("transform_math", req);
  ASSERT_TRUE(resp->result.success) << resp->result.krnx_err;

  const std::array<float, 12> expected = {0, 1, 0, -1, 0, 0, 0, 0, 1, 10, 20, 30};
  for (size_t i = 0; i < expected.size(); i++)
  {
    EXPECT_NEAR(resp->matrix[i], expected[i], TOLERANCE) << "matrix[" << i << "]";
  }

  // And back again through the matrix path.
  auto back_req = std::make_shared<srv::TransformMath::Request>();
  back_req->op = srv::TransformMath::Request::OP_CONVERT;
  back_req->use_matrix = true;
  back_req->matrix_a = resp->matrix;
  const auto back = call<srv::TransformMath>("transform_math", back_req);
  ASSERT_TRUE(back->result.success);
  for (size_t i = 0; i < 6; i++)
  {
    EXPECT_NEAR(back->xyzoat[i], req->xyzoat_a[i], TOLERANCE) << "xyzoat[" << i << "]";
  }
}

TEST_F(KrnxApiServicesTest, UndocumentedMutatorsRefuseByDefault)
{
  start(false);
  auto io_req = std::make_shared<srv::SetIo::Request>();
  io_req->data = {0x01};
  io_req->mask = {0x01};
  const auto io = call<srv::SetIo>("set_io", io_req);
  EXPECT_FALSE(io->result.success);
  EXPECT_NE(io->result.error_msg.find("enable_undocumented_krnx_api"), std::string::npos);

  const auto panel =
    call<srv::SetPanelInfo>("set_panel_info", std::make_shared<srv::SetPanelInfo::Request>());
  EXPECT_FALSE(panel->result.success);
  EXPECT_NE(panel->result.error_msg.find("enable_undocumented_krnx_api"), std::string::npos);
}

TEST_F(KrnxApiServicesTest, UndocumentedMutatorsValidateWhenEnabled)
{
  start(true);
  // Enabled, but the request itself is malformed: must refuse before reaching KRNX.
  auto io_req = std::make_shared<srv::SetIo::Request>();
  io_req->data = {0x01, 0x02};
  io_req->mask = {0x01};
  const auto io = call<srv::SetIo>("set_io", io_req);
  EXPECT_FALSE(io->result.success);
  EXPECT_NE(io->result.error_msg.find("same length"), std::string::npos);
}

TEST_F(KrnxApiServicesTest, BadArgumentsAreRefused)
{
  start(false);
  auto frame_req = std::make_shared<srv::GetFrame::Request>();
  frame_req->arm_no = 3;
  const auto frame = call<srv::GetFrame>("get_frame", frame_req);
  EXPECT_FALSE(frame->result.success);
  EXPECT_NE(frame->result.error_msg.find("arm_no 3 is out of range"), std::string::npos);

  auto program_req = std::make_shared<srv::ProgramControl::Request>();
  program_req->target = srv::ProgramControl::Request::TARGET_PC;
  program_req->number = 0;
  program_req->action = srv::ProgramControl::Request::ACTION_KILL;
  const auto program = call<srv::ProgramControl>("program_control", program_req);
  EXPECT_FALSE(program->result.success);
  EXPECT_NE(program->result.error_msg.find("1-3"), std::string::npos);

  // NULL reference joints are only documented for the two manual-covered IK methods.
  auto ik_req = std::make_shared<srv::InverseKinematics::Request>();
  ik_req->method = srv::InverseKinematics::Request::METHOD_XYZOAT_CONF;
  ik_req->reference = srv::InverseKinematics::Request::REFERENCE_NONE;
  const auto ik = call<srv::InverseKinematics>("inverse_kinematics", ik_req);
  EXPECT_FALSE(ik->result.success);
  EXPECT_NE(ik->result.error_msg.find("REFERENCE_NONE"), std::string::npos);
}

TEST_F(KrnxApiServicesTest, ControllerCallsFailCleanlyWithoutConnection)
{
  start(false);
  auto req = std::make_shared<srv::ForwardKinematics::Request>();
  req->joint = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  const auto resp = call<srv::ForwardKinematics>("forward_kinematics", req);
  EXPECT_FALSE(resp->result.success);
  EXPECT_LT(resp->result.krnx_code, 0);
  EXPECT_EQ(resp->result.krnx_err.rfind("-0x", 0), 0U) << resp->result.krnx_err;
}
}  // namespace
