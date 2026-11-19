#include <gtest/gtest.h>
#include <limits>

#include "../src/VelocityCommand.h"

using Clock = VelocityCommand::Clock;

TEST(VelocityCommand, StopsAfterOneSecondWithoutUpdates) {
  VelocityCommand command;
  const auto start = Clock::now();
  ASSERT_TRUE(command.set(0.3, 0.4, start));
  EXPECT_FALSE(command.expire(start + std::chrono::milliseconds(999)));
  EXPECT_DOUBLE_EQ(0.3, command.linear());
  EXPECT_TRUE(command.expire(start + std::chrono::milliseconds(1000)));
  EXPECT_DOUBLE_EQ(0.0, command.linear());
  EXPECT_DOUBLE_EQ(0.0, command.angular());
  EXPECT_FALSE(command.expire(start + std::chrono::milliseconds(2000)));
}

TEST(VelocityCommand, RepeatedCommandsRefreshTimeout) {
  VelocityCommand command;
  const auto start = Clock::now();
  ASSERT_TRUE(command.set(0.3, 0.0, start));
  ASSERT_TRUE(command.set(0.3, 0.0, start + std::chrono::milliseconds(900)));
  EXPECT_FALSE(command.expire(start + std::chrono::milliseconds(1000)));
  EXPECT_TRUE(command.expire(start + std::chrono::milliseconds(1900)));
}

TEST(VelocityCommand, ClearAndInvalidCommandsStopImmediately) {
  VelocityCommand command;
  const auto start = Clock::now();
  ASSERT_TRUE(command.set(0.3, 0.4, start));
  EXPECT_FALSE(command.set(std::numeric_limits<double>::quiet_NaN(), 0.0, start));
  EXPECT_DOUBLE_EQ(0.0, command.linear());
  EXPECT_DOUBLE_EQ(0.0, command.angular());
  ASSERT_TRUE(command.set(0.3, 0.4, start));
  command.clear();
  EXPECT_DOUBLE_EQ(0.0, command.linear());
  EXPECT_DOUBLE_EQ(0.0, command.angular());
}

TEST(VelocityCommand, ClockRewindStopsAndZeroTimeoutDoesNotReportMovement) {
  VelocityCommand command;
  const auto start = Clock::now();
  ASSERT_TRUE(command.set(0.0, 0.2, start));
  EXPECT_TRUE(command.expire(start - std::chrono::milliseconds(1)));
  ASSERT_TRUE(command.set(0.0, 0.0, start));
  EXPECT_FALSE(command.expire(start + std::chrono::seconds(1)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
