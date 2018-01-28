#include "catch.hpp"
#include "../src/tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

SCENARIO( "Test RMSE calculation" ) {

    GIVEN( "some estimations and ground truth" ) {

        ITools* t = new Tools();
        vector<VectorXd> estimations;
        vector<VectorXd> ground_truth;

        WHEN( "estimations are empty" ) {
          VectorXd a_ground_truth(4);
          a_ground_truth << 1, 2, 3, 4;
          ground_truth.push_back( a_ground_truth );

          VectorXd rmse = t->CalculateRMSE( estimations, ground_truth);

          REQUIRE( rmse.size() == 4 );
          REQUIRE( rmse.sum() == 0 );
        }

        WHEN( "ground-truths are empty" ) {
          VectorXd an_estimation(4);
          an_estimation << 1, 2, 3, 4;
          estimations.push_back( an_estimation );

          VectorXd rmse = t->CalculateRMSE( estimations, ground_truth);

          REQUIRE( rmse.size() == 4 );
          REQUIRE( rmse.sum() == 0 );
        }

        WHEN( "estimations and ground-truth doesn't have the same size") {
          VectorXd an_estimation(4);
          an_estimation << 1, 2, 3, 4;
          estimations.push_back( an_estimation );
          estimations.push_back( an_estimation );
          VectorXd a_ground_truth(4);
          a_ground_truth << 1, 2, 3, 4;
          ground_truth.push_back( a_ground_truth );

          VectorXd rmse = t->CalculateRMSE( estimations, ground_truth);

          REQUIRE( rmse.size() == 4 );
          REQUIRE( rmse.sum() == 0 );
        }

        WHEN( "estimation and ground-truth are good") {
          VectorXd an_estimation(4);
          an_estimation << 1, 2, 3, 4;
          estimations.push_back( an_estimation );
          estimations.push_back( an_estimation );
          VectorXd a_ground_truth(4);
          a_ground_truth << 4, 3, 2, 1;
          ground_truth.push_back( a_ground_truth );
          ground_truth.push_back( a_ground_truth );

          VectorXd rmse = t->CalculateRMSE( estimations, ground_truth);

          REQUIRE( rmse.size() == 4 );
          REQUIRE( rmse(0) == 3 );
          REQUIRE( rmse(1) == 1 );
          REQUIRE( rmse(2) == 1 );
          REQUIRE( rmse(3) == 3 );
        }
    }
}

SCENARIO( "Test Jacobian calculation" ) {
  GIVEN( "an state vector" ) {

    ITools* t = new Tools();

    WHEN( "the state vector size is not 4 an error is reported" ) {
      VectorXd state(2);
      state << 1, 2;
      MatrixXd Hj = t->CalculateJacobian(state);
    }

    WHEN( "px and py are zero an error should be printed.") {
      VectorXd state(4);
      state << 0, 0, 3, 4;
      MatrixXd Hj = t->CalculateJacobian(state);
    }

    WHEN( "with other values the calculation should success.") {
      VectorXd state(4);
      state << 1, 2, 3, 4;
      MatrixXd Hj = t->CalculateJacobian(state);
      REQUIRE( round(Hj(1, 1) * 10) == 2 );
      REQUIRE( round(Hj(1, 0) * 10) == -4 );
    }
  }
}

SCENARIO("Test polar to cartesian") {
  GIVEN("polar vector") {
    
    std::shared_ptr<ITools> t = std::make_shared<Tools>();
    WHEN("values of polar are provided") {
      VectorXd polar(3);
      polar << 1.014892e+00, 5.543292e-01, 4.892807e+00;
      VectorXd cartesian = t->p2c(polar);
      REQUIRE( cartesian(0) == Approx((double)0.862915701) );
      REQUIRE( cartesian(1) == Approx((double)0.5342118162) );
      REQUIRE( cartesian(2) == Approx((double)4.1601273657) );
      REQUIRE( cartesian(3) == Approx((double)2.575441834) );
    }
    WHEN("values of polar are all zero") {
      VectorXd polar(3);
      polar << 0.0, 0.0, 0.0;
      VectorXd cartesian = t->p2c(polar);
      REQUIRE( cartesian(0) == Approx((double)0.0001) );
      REQUIRE( cartesian(1) == Approx((double)0.0001) );
      REQUIRE( cartesian(2) == Approx((double)0.0) );
      REQUIRE( cartesian(3) == Approx((double)0.0) );
    }
  }
}

SCENARIO("Test cartesian to polar") {
  GIVEN("cartesian vector") {
    
    std::shared_ptr<ITools> t = std::make_shared<Tools>();
    WHEN("values of polar are provided") {
      VectorXd cartesian(4);
      cartesian << 3.122427e-01, 5.803398e-01, 4.892807e+00, 3.892807e+00;
      VectorXd polar = t->c2p(cartesian);
      REQUIRE( polar(0) == Approx((double)0.659006667) );
      REQUIRE( polar(1) == Approx((double)1.0771862489) );
      REQUIRE( polar(2) == Approx((double)5.7463669089) );
    }
    WHEN("values of polar are all zero") {
      VectorXd cartesian(4);
      cartesian << 0.0, 0.0, 0.0, 0.0;
      VectorXd polar = t->c2p(cartesian);
      REQUIRE( polar(0) == Approx((double)0.0001) );
      REQUIRE( polar(1) == Approx((double)0.0) );
      REQUIRE( polar(2) == Approx((double)0.0) );
    }
  }
}
