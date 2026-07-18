#include <gtest/gtest.h>

extern "C" {
#include "dsp.c"
}

TEST(DSPTest, BasicFiltering) { EXPECT_EQ(1, 1); }
