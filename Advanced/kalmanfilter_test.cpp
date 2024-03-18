// #include "kalmanfilter.h"
// #include "Matrix.hpp"
// #include <string>
// #include <limits>
// #include <iostream>

// // See the comments in Main.cpp
// // @http://www.boost.org/doc/libs/1_68_0/libs/test/doc/html/boost_test/adv_scenarios/shared_lib_customizations/entry_point.html
// //#define BOOST_TEST_MODULE MatrixTestModule
// //#define BOOST_TEST_DYN_LINK
// //#define BOOST_TEST_NO_MAIN

// // Only one of the following 3 includes may be active!

// // For the header only version
// //#include <boost/test/included/unit_test.hpp>
// // For the static library version
// //#include <boost/test/unit_test.hpp>
// // For the dynamic library version
// #include <boost/test/unit_test.hpp>
// BOOST_AUTO_TEST_SUITE(KalmanFilter)
//     BOOST_AUTO_TEST_CASE(TEST)
// 	{
//     // Initial conditions and dummy measurement
//     Matrix<float, 2, 1> estimate({0.0f, 0.0f);           //initial estimate
//     Matrix<float, 2, 1> errorInEstimate({0.0f, 0.0f);    //initial error in estimate
//     Matrix<float, 2, 1> measurement1({0.0f, 0.0f});       //initial measurement
//     Matrix<float, 2, 1> errorInMeasurement({0.0f, 0.0f}); //error in measurement
//     Matrix<float, 2, 1> processNoise({0.0f, 0.0f});       //not used  

//     kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

//     // Update the filter with the measurement
//     kf.update(measurement1);

//     // Check if the state vector is updated correctly
//     // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
//     Matrix<float, 2, 1> expectedCovariance({0.0f, 0.0f}); 
//     std::cout << "new value: " << kf.getCovariance().to_string() << std::endl;
//     std::cout << "expected value: " << expectedCovariance.to_string() << std::endl;
//     BOOST_CHECK_CLOSE_FRACTION(1.0f, 1.0f, 0.01);
// 	}

// BOOST_AUTO_TEST_SUITE_END()
