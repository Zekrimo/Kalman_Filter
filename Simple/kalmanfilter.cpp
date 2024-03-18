#include "kalmanfilter.h"

kalmanfilter::kalmanfilter(float initialState, float initialCovariance, float measurementNoise, float processNoise)
    : currentStateVector(initialState),
      currentCovarianceMatrix(initialCovariance),
      sensorCovarianceMatrix(measurementNoise),
      processNoise(processNoise)
{
    // Initialize other member variables as necessary
}

kalmanfilter::~kalmanfilter()
{
}

void kalmanfilter::predict()
{
    std::cout << std::endl;
    // Use the state transition model to predict the next state
    currentStateVector = calculatePredictedStateVector(currentStateVector, currentCovarianceMatrix);
    currentCovarianceMatrix = calculatePredictedCovariance(currentCovarianceMatrix);// + processNoise; // Adding process noise to the prediction
}

void kalmanfilter::update(float measurement)
{
    // Calculate the Kalman gain
    calculateKalmanGain();
    // Update the state estimate.
    currentStateVector = calculateAdjustedStateVector(currentStateVector, currentCovarianceMatrix, measurement);
    // Update the estimate's uncertainty
    currentCovarianceMatrix = calculateAdjustedCovariance(currentCovarianceMatrix);
}

std::pair<float, float> kalmanfilter::calculatePredictedState(float estimate, float error)
{
    predictedStateVector = calculatePredictedStateVector(estimate, error);
    predictedCovarianceMatrix = calculatePredictedCovariance(error);

    return std::pair<float, float>(predictedStateVector,
                                   predictedCovarianceMatrix
                                   );
}

float kalmanfilter::calculatePredictedStateVector(float estimate, float error)
{
    predictedStateVector = currentStateVector;
    return predictedStateVector;
}

float kalmanfilter::calculatePredictedCovariance(float error)
{
    predictedCovarianceMatrix = currentCovarianceMatrix;
    return predictedCovarianceMatrix;
}

float kalmanfilter::calculateKalmanGain()
{

    kalmanGain = predictedCovarianceMatrix / (predictedCovarianceMatrix + sensorCovarianceMatrix);
    if(isdebug){std::cout << "calculateKalmanGain: " << predictedCovarianceMatrix << " / (" << predictedCovarianceMatrix << " + " << sensorCovarianceMatrix << ") = " << kalmanGain << std::endl;}
    return kalmanGain;
}

float kalmanfilter::calculateMeasurementVector(float estimate, float measurement)
{
    std::cerr << "kalmanfilter::calculateMeasurementVector: Not implemented" << std::endl;
    return 0.0f;
}

std::pair<float, float> kalmanfilter::calculateAdjustedState(float estimate, float error, float measurement)
{
    adjustedStateVector = calculateAdjustedStateVector(estimate, error, measurement);
    adjustedCovarianceMatrix = calculateAdjustedCovariance(error);

    return std::pair<float, float>(adjustedStateVector, 
                                   adjustedCovarianceMatrix
                                   );
}

float kalmanfilter::calculateAdjustedStateVector(float estimate, float error, float measurement)
{
    adjustedStateVector = predictedStateVector + kalmanGain * (measurement - predictedStateVector);
    if(isdebug){std::cout << "calculateAdjustedStateVector: " << predictedStateVector << " + " << kalmanGain << " * (" << measurement << " - " << predictedStateVector << ") = " << adjustedStateVector << std::endl;}
    return adjustedStateVector;
}

float kalmanfilter::calculateAdjustedCovariance(float error)
{
    adjustedCovarianceMatrix = (1 - kalmanGain) * predictedCovarianceMatrix;
    if(isdebug){std::cout << "calculateAdjustedCovariance: (1 - " << kalmanGain << ") * " << predictedCovarianceMatrix << " = " << adjustedCovarianceMatrix << std::endl;}
    return adjustedCovarianceMatrix;
}
