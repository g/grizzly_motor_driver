

#include <gtest/gtest.h>
#include <iostream>
#include "grizzly_motor_driver/frame.h"

TEST(TestCanFrame, testCanFrameId)
{

  for (uint8_t i = 1; i < 255; i++)
  {
    grizzly_motor_driver::Frame* test_frame = new grizzly_motor_driver::Frame(i);
    EXPECT_EQ(test_frame->getCanId(), i);
    delete test_frame;
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
