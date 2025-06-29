#include <gtest/gtest.h>
#include "FusedData/IMUDepthFusion.h"


TEST(DataFusionMid, midWeightVelocity)
{
    double velocity1 = 5.6;
    double velocity2 = 7.3;
    double weight = 0.5;
    double finalVelocity;
    double expectedVelocity = 6.45;

    finalVelocity = IMUDepthFusion::fusedVelocity(velocity1, velocity2, weight);
    EXPECT_NEAR(finalVelocity, expectedVelocity, 1e-5);
}

TEST(DataFusionZero, zeroWeightVelocity)
{
    double velocity1 = 10.655;
    double velocity2 = 17.553;
    double weight = 0;
    double finalVelocity;
    double expectedVelocity = velocity1;

    finalVelocity = IMUDepthFusion::fusedVelocity(velocity1, velocity2, weight);
    EXPECT_NEAR(finalVelocity, expectedVelocity, 1e-5);
}

TEST(DataFusionFull, fullWeighVelocity)
{
    double velocity1 = -25.6;
    double velocity2 = -7.3;
    double weight = 1.0;
    double finalVelocity;
    double expectedVelocity = velocity2;

    finalVelocity = IMUDepthFusion::fusedVelocity(velocity1, velocity2, weight);
    EXPECT_NEAR(finalVelocity, expectedVelocity, 1e-5);
}


// int main(int argc, char** argv)
// {
//     testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }