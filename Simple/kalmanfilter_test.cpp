#include "kalmanfilter.h"
#include <string>
#include <limits>
#include <iostream>

// See the comments in Main.cpp
// @http://www.boost.org/doc/libs/1_68_0/libs/test/doc/html/boost_test/adv_scenarios/shared_lib_customizations/entry_point.html
//#define BOOST_TEST_MODULE MatrixTestModule
//#define BOOST_TEST_DYN_LINK
//#define BOOST_TEST_NO_MAIN

// Only one of the following 3 includes may be active!

// For the header only version
//#include <boost/test/included/unit_test.hpp>
// For the static library version
//#include <boost/test/unit_test.hpp>
// For the dynamic library version
#include <boost/test/unit_test.hpp>
BOOST_AUTO_TEST_SUITE( KalmanFIlter)
	BOOST_AUTO_TEST_CASE( UpdateStateVectorfirstIteration)
	{
    // Initial conditions and dummy measurement
    float estimate = 68.0f;                         //initial estimate
    float errorInEstimate = 2.0f;                   //initial error in estimate
    float measurement = 75.0f;                      //initial measurement
    float errorInMeasurement = 4.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedStateAfterUpdate = 70.33;
    std::cout << "new value: " << kf.getState() << std::endl;
    std::cout << "expected value: " << expectedStateAfterUpdate << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getState(), expectedStateAfterUpdate, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateStateVectorSecondIteration)
	{
    // Initial conditions and dummy measurement
    float estimate = 68.0f;                         //initial estimate
    float errorInEstimate = 2.0f;                   //initial error in estimate
    float measurement1 = 75.0f;                      //initial measurement
    float measurement2 = 71.0f;                      //initial measurement
    float errorInMeasurement = 4.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedStateAfterUpdate = 70.5;
    std::cout << "new value: " << kf.getState() << std::endl;
    std::cout << "expected value: " << expectedStateAfterUpdate << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getState(), expectedStateAfterUpdate, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateStateVectorthirthIteration)
	{
    // Initial conditions and dummy measurement
    float estimate = 68.0f;                         //initial estimate
    float errorInEstimate = 2.0f;                   //initial error in estimate
    float measurement1 = 75.0f;                      //initial measurement
    float measurement2 = 71.0f;                      //initial measurement
    float measurement3 = 70.0f;                      //initial measurement
    float errorInMeasurement = 4.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);
    kf.predict();
    kf.update(measurement3);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedStateAfterUpdate = 70.4;
    std::cout << "new value: " << kf.getState() << std::endl;
    std::cout << "expected value: " << expectedStateAfterUpdate << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getState(), expectedStateAfterUpdate, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateStateVectortfourthIteration)
	{
    // Initial conditions and dummy measurement
    float estimate = 68.0f;                         //initial estimate
    float errorInEstimate = 2.0f;                   //initial error in estimate
    float measurement1 = 75.0f;                      //initial measurement
    float measurement2 = 71.0f;                      //initial measurement
    float measurement3 = 70.0f;                      //initial measurement
    float measurement4 = 74.0f;                      //initial measurement
    float errorInMeasurement = 4.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);
    kf.predict();
    kf.update(measurement3);
    kf.predict();
    kf.update(measurement4);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedStateAfterUpdate = 71.0;
    std::cout << "new value: " << kf.getState() << std::endl;
    std::cout << "expected value: " << expectedStateAfterUpdate << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getState(), expectedStateAfterUpdate, 0.01);
	}
    	
    BOOST_AUTO_TEST_CASE( UpdateCovarianceMatrixFirstIteration)
	{
    // Initial conditions and dummy measurement
    float estimate = 68.0f;                         //initial estimate
    float errorInEstimate = 2.0f;                   //initial error in estimate
    float measurement = 75.0f;                      //initial measurement
    float errorInMeasurement = 4.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedCovariance = 1.33;
    std::cout << "new value: " << kf.getCovariance() << std::endl;
    std::cout << "expected value: " << expectedCovariance << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getCovariance(), expectedCovariance, 0.01);
	}

        BOOST_AUTO_TEST_CASE( UpdateCovarianceMatrixSecondIteration)
	{
    // Initial conditions and dummy measurement
    float estimate = 68.0f;                         //initial estimate
    float errorInEstimate = 2.0f;                   //initial error in estimate
    float measurement1 = 75.0f;                      //initial measurement
    float measurement2 = 71.0f;                      //initial measurement
    float errorInMeasurement = 4.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedCovariance = 1.0;
    std::cout << "new value: " << kf.getCovariance() << std::endl;
    std::cout << "expected value: " << expectedCovariance << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getCovariance(), expectedCovariance, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateCovarianceMatrixthirthIteration)
	{
    // Initial conditions and dummy measurement
    float estimate = 68.0f;                         //initial estimate
    float errorInEstimate = 2.0f;                   //initial error in estimate
    float measurement1 = 75.0f;                      //initial measurement
    float measurement2 = 71.0f;                      //initial measurement
    float measurement3 = 70.0f;                      //initial measurement
    float errorInMeasurement = 4.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);
    kf.predict();
    kf.update(measurement3);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedCovariance = 0.8;
    std::cout << "new value: " << kf.getCovariance() << std::endl;
    std::cout << "expected value: " << expectedCovariance << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getCovariance(), expectedCovariance, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateCovarianceMatrixfourthIteration)
	{
    // Initial conditions and dummy measurement
    float estimate = 68.0f;                         //initial estimate
    float errorInEstimate = 2.0f;                   //initial error in estimate
    float measurement1 = 75.0f;                      //initial measurement
    float measurement2 = 71.0f;                      //initial measurement
    float measurement3 = 70.0f;                      //initial measurement
    float measurement4 = 74.0f;                      //initial measurement
    float errorInMeasurement = 4.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);
    kf.predict();
    kf.update(measurement3);
    kf.predict();
    kf.update(measurement4);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedCovariance = 0.66;
    std::cout << "new value: " << kf.getCovariance() << std::endl;
    std::cout << "expected value: " << expectedCovariance << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getCovariance(), expectedCovariance, 0.011);
	}

    // second dataset 

    BOOST_AUTO_TEST_CASE( UpdateStateVectorfirstIteration2)
	{
    // Initial conditions and dummy measurement
    float estimate = 2000.0f;                        //initial estimate
    float errorInEstimate = 50.0f;                   //initial error in estimate
    float measurement = 2300.0f;                     //initial measurement
    float errorInMeasurement = 10.0f;                //error in measurement

    float processNoise = 0.0f;                       //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedStateAfterUpdate = 2249.0;
    std::cout << "new value: " << kf.getState() << std::endl;
    std::cout << "expected value: " << expectedStateAfterUpdate << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getState(), expectedStateAfterUpdate, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateStateVectorSecondIteration2)
	{
    // Initial conditions and dummy measurement
    float estimate = 2000.0f;                        //initial estimate
    float errorInEstimate = 50.0f;                   //initial error in estimate
    float measurement1 = 2300.0f;                      //initial measurement
    float measurement2 = 2100.0f;                      //initial measurement
    float errorInMeasurement = 10.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedStateAfterUpdate = 2181.95;
    std::cout << "new value: " << kf.getState() << std::endl;
    std::cout << "expected value: " << expectedStateAfterUpdate << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getState(), expectedStateAfterUpdate, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateStateVectorthirthIteration2)
	{
    // Initial conditions and dummy measurement
    float estimate = 2000.0f;                        //initial estimate
    float errorInEstimate = 50.0f;                   //initial error in estimate
    float measurement1 = 2300.0f;                      //initial measurement
    float measurement2 = 2100.0f;                      //initial measurement
    float measurement3 = 2150.0f;                      //initial measurement
    float errorInMeasurement = 10.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);
    kf.predict();
    kf.update(measurement3);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedStateAfterUpdate = 2171.78;
    std::cout << "new value: " << kf.getState() << std::endl;
    std::cout << "expected value: " << expectedStateAfterUpdate << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getState(), expectedStateAfterUpdate, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateStateVectortfourthIteration2)
	{
    // Initial conditions and dummy measurement
    float estimate = 2000.0f;                        //initial estimate
    float errorInEstimate = 50.0f;                   //initial error in estimate
    float measurement1 = 2300.0f;                      //initial measurement
    float measurement2 = 2100.0f;                      //initial measurement
    float measurement3 = 2150.0f;                      //initial measurement
    float measurement4 = 2160.0f;                      //initial measurement
    float errorInMeasurement = 10.0f;                //error in measurement

    float processNoise = 0;                      //not used                      

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);
    kf.predict();
    kf.update(measurement3);
    kf.predict();
    kf.update(measurement4);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedStateAfterUpdate = 2168.94;
    std::cout << "new value: " << kf.getState() << std::endl;
    std::cout << "expected value: " << expectedStateAfterUpdate << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getState(), expectedStateAfterUpdate, 0.01);
	}
    	
    BOOST_AUTO_TEST_CASE( UpdateCovarianceMatrixFirstIteration2)
	{
    // Initial conditions and dummy measurement
    float estimate = 2000.0f;                        //initial estimate
    float errorInEstimate = 50.0f;                   //initial error in estimate
    float measurement = 2300.0f;                      //initial measurement
    float errorInMeasurement = 10.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                     

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedCovariance = 8.33;
    std::cout << "new value: " << kf.getCovariance() << std::endl;
    std::cout << "expected value: " << expectedCovariance << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getCovariance(), expectedCovariance, 0.01);
	}

        BOOST_AUTO_TEST_CASE( UpdateCovarianceMatrixSecondIteration2)
	{
    // Initial conditions and dummy measurement
    float estimate = 2000.0f;                        //initial estimate
    float errorInEstimate = 50.0f;                   //initial error in estimate
    float measurement1 = 2300.0f;                      //initial measurement
    float measurement2 = 2100.0f;                      //initial measurement
    float errorInMeasurement = 10.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                     

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedCovariance = 4.54;
    std::cout << "new value: " << kf.getCovariance() << std::endl;
    std::cout << "expected value: " << expectedCovariance << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getCovariance(), expectedCovariance, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateCovarianceMatrixthirthIteration2)
	{
    // Initial conditions and dummy measurement
    float estimate = 2000.0f;                        //initial estimate
    float errorInEstimate = 50.0f;                   //initial error in estimate
    float measurement1 = 2300.0f;                      //initial measurement
    float measurement2 = 2100.0f;                      //initial measurement
    float measurement3 = 2150.0f;                      //initial measurement
    float errorInMeasurement = 10.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used                     

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);
    kf.predict();
    kf.update(measurement3);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedCovariance = 3.125;
    std::cout << "new value: " << kf.getCovariance() << std::endl;
    std::cout << "expected value: " << expectedCovariance << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getCovariance(), expectedCovariance, 0.01);
	}

    BOOST_AUTO_TEST_CASE( UpdateCovarianceMatrixfourthIteration2)
	{
    // Initial conditions and dummy measurement
    float estimate = 2000.0f;                        //initial estimate
    float errorInEstimate = 50.0f;                   //initial error in estimate
    float measurement1 = 2300.0f;                      //initial measurement
    float measurement2 = 2100.0f;                      //initial measurement
    float measurement3 = 2150.0f;                      //initial measurement
    float measurement4 = 2160.0f;                      //initial measurement
    float errorInMeasurement = 10.0f;                //error in measurement

    float processNoise = 0.0f;                      //not used  

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);

    // Predict to initialize the predicted state vector and covariance matrix
    kf.predict();
    // Update the filter with the measurement
    kf.update(measurement1);
    kf.predict();
    kf.update(measurement2);
    kf.predict();
    kf.update(measurement3);
    kf.predict();
    kf.update(measurement4);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    float expectedCovariance = 2.38;
    std::cout << "new value: " << kf.getCovariance() << std::endl;
    std::cout << "expected value: " << expectedCovariance << std::endl;
    BOOST_CHECK_CLOSE_FRACTION(kf.getCovariance(), expectedCovariance, 0.011);
	}

BOOST_AUTO_TEST_SUITE_END()
