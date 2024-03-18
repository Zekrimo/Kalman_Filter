#include <utility>
#include <iostream> // Include the <iostream> header to access std::cerr


class kalmanfilter
{ 

public:
    kalmanfilter(float initialState, float initialCovariance, float measurementNoise, float processNoise);
    ~kalmanfilter();

    void predict();

    void update(float measurement);

    float getState() const { return currentStateVector; }
    float getCovariance() const { return currentCovarianceMatrix; }

private:
    float initialStateVector;           //                              | U_t0
    float initialCovarianceMatrix;      //                              | E_t0

    float currentStateVector;           //                              | U_t-1
    float currentCovarianceMatrix;      //                              | E_t-1

    float predictedStateVector;         //                              | U^_t
    float predictedCovarianceMatrix;    //                              | E^_t

    float kalmanGain;                   //                              | K_t

    float adjustedStateVector;          //                              | U_t
    float adjustedCovarianceMatrix;     //                              | E_t

    float sensorCovarianceMatrix;       //                              | Q_t

    float processNoise;                 //                              | Z_t

    bool isdebug = false;


    /**
     * @brief Represents a pair of floating-point values.
     *
     * This class is used to store and manipulate a pair of floating-point values.
     * The first value represents the estimated state, and the second value represents the error.
     */
    std::pair<float, float> calculatePredictedState(float estimate, float error);

    /**
     * Calculates the predicted state vector based on the given estimate and error.
     *
     * @param estimate The current estimate of the state vector.
     * @param error The error associated with the estimate.
     * @return The predicted state vector.
     */
    float calculatePredictedStateVector(float estimate, float error);

    /**
     * Calculates the predicted covariance based on the given error.
     *
     * @param error The error value used to calculate the predicted covariance.
     * @return The calculated predicted covariance.
     */
    float calculatePredictedCovariance(float error);
    
    /**
     * Calculates the Kalman gain for a given measurement.
     *
     * @param measurement The measurement value.
     * @return A pair of floats representing the Kalman gain.
     */
    float calculateKalmanGain();
    
    /**
     * Calculates the measurement vector based on the estimate and measurement values.
     *
     * @param estimate The estimated value.
     * @param measurement The measured value.
     * @return The calculated measurement vector.
     */
    float calculateMeasurementVector(float estimate, float measurement);
    
    /**
     * Calculates the adjusted state based on the given estimate, error, and measurement.
     *
     * @param estimate The estimated state value.
     * @param error The error associated with the estimate.
     * @param measurement The measured state value.
     * @return A pair containing the adjusted state value and the updated error.
     */
    std::pair<float, float> calculateAdjustedState(float estimate, float error, float measurement);
    
    /**
     * Calculates the adjusted state vector based on the estimate, error, and measurement.
     *
     * @param estimate The estimated state vector.
     * @param error The error in the estimate.
     * @param measurement The measured state vector.
     * @return The adjusted state vector.
     */
    float calculateAdjustedStateVector(float estimate, float error, float measurement);
    
    /**
     * Calculates the adjusted covariance based on the given error.
     *
     * @param error The error value used to adjust the covariance.
     * @return The adjusted covariance value.
     */
    float calculateAdjustedCovariance(float error);

};  // class kalmanfilter