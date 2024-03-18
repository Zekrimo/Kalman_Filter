// This can be done in any of the test files but then we will have multiple executables
// in one Eclipse project which is impossible in a non-makefile project

// See http://www.boost.org/doc/libs/1_68_0/libs/test/doc/html/boost_test/adv_scenarios/shared_lib_customizations/entry_point.html
// #define BOOST_TEST_MODULE KalmanTestModule
// #define BOOST_TEST_DYN_LINK
// #define BOOST_TEST_NO_MAIN
// #include <boost/test/unit_test.hpp>

#include "kalmanfilter.h"

int main( 	int argc,
			char** argv)
{
	// try
	// {
	// 	// In a real program we should test whether we should call the real main or run the unit_test_main
	// 	return boost::unit_test::unit_test_main( &init_unit_test, argc, argv ); // @suppress("Symbol is not resolved") // @suppress("Invalid arguments")
	// }
	// catch (std::exception& e)
	// {
	// 	std::cout << e.what() << std::endl;
	// }

    Matrix<float, 2, 1> estimate{{{4000.0f}}, {{280.0f}}};               //initial estimate
    Matrix<float, 2, 1> errorInEstimate{{{20.0f}}, {{5.0f}}};    //initial error in estimate
    Matrix<float, 1, 1> update1{{{2.0f}}};                          //initial measurement
    Matrix<float, 2, 1> measurement1{{{4260.0f}}, {{282.0f}}};                           //initial measurement
    Matrix<float, 2, 1> errorInMeasurement{{{25.0f}}, {{6.0f}}}; //error in measurement
    Matrix<float, 2, 1> processNoise{{{0.0f}}, {{0.0f}}};                //not used  

    kalmanfilter kf(estimate, errorInEstimate, errorInMeasurement, processNoise);


    

    // Update the filter with the measurement
    kf.update(update1, measurement1);

    // Check if the state vector is updated correctly
    // This is a placeholder, replace with the actual expected value based on your kalmanfilter implementation
    std::cout << std::endl;
    std::cout << std::endl;
    Matrix<float, 2, 1> expectedStateVector{{{4272.5f}}, {{284.0f}}}; 
    std::cout << "new value: " << kf.getState().to_string() << std::endl;
    std::cout << "expected value: " << expectedStateVector.to_string() << std::endl;

    std::cout << std::endl;
    Matrix<float, 2, 1> expectedCovariance{{{253.0f}}, {{148.0f}}}; 
    std::cout << "new value: " << kf.getCovariance().to_string() << std::endl;
    std::cout << "expected value: " << expectedCovariance.to_string() << std::endl;


	
	return 0;
}
