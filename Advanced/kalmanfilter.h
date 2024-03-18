#include <utility>
#include <iostream> // Include the <iostream> header to access std::cerr
#include "Matrix.hpp"

#define R 2 //rows of matrix
#define C 1 //columns of matrix

class kalmanfilter
{ 

public:
    // measurementNoise: This matrix represents the uncertainty or noise in the measurements made by the sensor.
    // In the Kalman filter algorithm, it's denoted by the symbol RR. In the Kalman filter function, 
    // it's used to define the measurement noise covariance matrix QQ.
    // 
    // processNoise: Similar to measurementNoise, this matrix represents the uncertainty or noise in the process
    // itself. In the Kalman filter algorithm, it's denoted by the symbol QQ. In the Kalman filter function, 
    // it's used to define the process noise covariance matrix RR.
    // 
    // sensorCovarianceMatrix: This matrix seems to represent the covariance matrix associated with the 
    // sensor measurements. In the Kalman filter algorithm, this matrix is not explicitly used, 
    // but it seems to be related to the measurement noise or the uncertainty in the sensor measurements.
    //
    // processNoise: This matrix represents the uncertainty or noise in the process itself. 
    // It's similar to measurementNoise but is related specifically to the process dynamics. 
    // In the Kalman filter algorithm, it's also denoted by the symbol QQ, and it's used to define the 
    // process noise covariance matrix QQ in the Kalman filter function.
    kalmanfilter(Matrix<float, R, 1> initialState, Matrix<float, R, C> initialCovariance, Matrix<float, R, C> measurementNoise, Matrix<float, R, C> processNoise);
    ~kalmanfilter();

    void update(Matrix<float, C, 1> update, Matrix<float, R, 1> measurement);

    Matrix<float, R, 1>  getState() const { return currentStateVector; }
    Matrix<float, R, R>  getCovariance() const { return currentCovarianceMatrix; }

private: 



    

    Matrix<float, R, 1> adjustedStateVector;         // U_t
    Matrix<float, R, R> adjustedCovarianceMatrix;    // E_t
    Matrix<float, R, C> processNoise;                // Z_t


    //????
    Matrix<float, R, 1> currentStateVector;          // mu_t-1
    Matrix<float, R, R> currentCovarianceMatrix;     // sigma_t-1
    Matrix<float, R, R> A_t;
    Matrix<float, R, C> R_t;
    Matrix<float, R, C> B_t; 
    Matrix<float, R, C> e_t; 
    Matrix<float, R, R> sensorCovarianceMatrix;      // Q_t
    Matrix<float, C, 1> u_t;                         // update, not used
    Matrix<float, R, R> C_t;                         //
    Matrix<float, R, R> kalmanGain;                  // K_t
    Matrix<float, R, C> x_t;                         // measurement, not used
    Matrix<float, R, C> Z_t;                         // measurement vecor, not used
    Matrix<float, R, R> I;                           // identity matrix
    Matrix<float, R, 1> predictedStateVector;        // U^_t
    Matrix<float, R, R> predictedCovarianceMatrix;   // E^_t




    bool isdebug = true;

    void predict(Matrix<float, C, 1> update);


    /**
     * @brief Represents a pair of floating-point values.
     *
     * This class is used to store and manipulate a pair of floating-point values.
     * The first value represents the estimated state, and the second value represents the error.
     */
    //std::pair<Matrix<float, R, C>, Matrix<float, 2, 2>> calculatePredictedState(Matrix<float, 2, 2> estimate, Matrix<float, 2, 2> error);

    /**
     * Calculates the predicted state vector based on the given estimate and error.
     *
     * @param estimate The current estimate of the state vector.
     * @param error The error associated with the estimate.
     * @return The predicted state vector.
     */
    Matrix<float, R, 1> calculatePredictedStateVector(Matrix<float, C, 1> update);

    /**
     * Calculates the predicted covariance based on the given error.
     *
     * @param error The error value used to calculate the predicted covariance.
     * @return The calculated predicted covariance.
     */
    Matrix<float, R, R> calculatePredictedCovariance();
    
    /**
     * Calculates the Kalman gain for a given measurement.
     *
     * @param measurement The measurement value.
     * @return A pair of floats representing the Kalman gain.
     */
    Matrix<float, R, R> calculateKalmanGain();
    
    /**
     * Calculates the measurement vector based on the estimate and measurement values.
     *
     * @param estimate The estimated value.
     * @param measurement The measured value.
     * @return The calculated measurement vector.
     */
    Matrix<float, R, C> calculateMeasurementVector(Matrix<float, R, C> measurement);
    
    /**
     * Calculates the adjusted state based on the given estimate, error, and measurement.
     *
     * @param estimate The estimated state value.
     * @param error The error associated with the estimate.
     * @param measurement The measured state value.
     * @return A pair containing the adjusted state value and the updated error.
     */
    //std::pair<Matrix<float, 2, 2>, Matrix<float, 2, 2>> calculateAdjustedState(Matrix<float, 2, 2> estimate, Matrix<float, 2, 2> error, Matrix<float, 2, 2> measurement);
    
    /**
     * Calculates the adjusted state vector based on the estimate, error, and measurement.
     *
     * @param estimate The estimated state vector.
     * @param error The error in the estimate.
     * @param measurement The measured state vector.
     * @return The adjusted state vector.
     */
    Matrix<float, R, 1> calculateAdjustedStateVector();
    
    /**
     * Calculates the adjusted covariance based on the given error.
     *
     * @param error The error value used to adjust the covariance.
     * @return The adjusted covariance value.
     */
    Matrix<float, R, R> calculateAdjustedCovariance();

};  // class kalmanfilter