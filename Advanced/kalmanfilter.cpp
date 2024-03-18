#include "kalmanfilter.h"

kalmanfilter::kalmanfilter(Matrix<float, R, 1> initialState, Matrix<float, R, C> initialCovariance, Matrix<float, R, C> measurementNoise, Matrix<float, R, C> processNoise)
    :   currentStateVector(initialState),
        processNoise(processNoise)
{
    // Initialize other member variables as necessary
    A_t = Matrix<float, R, R>({1.0f, 1.0f, 0.0f, 1.0f});
    C_t = Matrix<float, R, R>({1.0f, 0.0f, 0.0f, 1.0f});
    I = C_t;
    B_t = Matrix<float, R, C>({0.5f, 1.0f});
    currentCovarianceMatrix = initialCovariance * initialCovariance.transpose();
    sensorCovarianceMatrix = measurementNoise * measurementNoise.transpose();
    sensorCovarianceMatrix.at(0, 1) = 0.0f;
    sensorCovarianceMatrix.at(1, 0) = 0.0f;
}

kalmanfilter::~kalmanfilter()
{
}

void kalmanfilter::predict(Matrix<float, C, 1> update)
{
    std::cout << std::endl;
    // Use the state transition model to predict the next state

    if(isdebug){std::cout << "initalStateVector: " << currentStateVector.to_string() << std::endl;}
    currentStateVector = calculatePredictedStateVector(update);

    if(isdebug){std::cout << "InitialCovarianceMatrix: " << currentCovarianceMatrix.to_string() << std::endl;}
    currentCovarianceMatrix = calculatePredictedCovariance();// + processNoise; // Adding process noise to the prediction
}

void kalmanfilter::update(Matrix<float, C, 1> update, Matrix<float, R, 1> measurement)
{
    if(isdebug){std::cout << "kalmanfilter::update: " << std::endl;}

    //predict state vector and covariance matrix
    predict(update);

    // Calculate the Kalman gain
    if(isdebug){std::cout << "calculateKalmanGain: " << std::endl;}
    calculateKalmanGain();
    calculateMeasurementVector(measurement);
    // Update the state estimate.
    if(isdebug){std::cout << "calculateAdjustedStateVector: " << std::endl;}
    currentStateVector = calculateAdjustedStateVector();
    
    // Update the estimate's uncertainty
    if(isdebug){std::cout << "calculateAdjustedCovariance: " << std::endl;}
    currentCovarianceMatrix = calculateAdjustedCovariance();
}


Matrix<float, R, 1> kalmanfilter::calculatePredictedStateVector(Matrix<float, C, 1> update)
{
    predictedStateVector = (A_t * currentStateVector) + (B_t * update);
    //predictedStateVector = A_t * currentStateVector + B_t * u_t + e_t;
    if(isdebug){std::cout << "calculatePredictedStateVector: " << A_t.to_string() << " * " << currentStateVector.to_string() << " + " << B_t.to_string() << " * " << u_t.to_string() << " = " << predictedStateVector.to_string() << std::endl;}
    return predictedStateVector;
}

Matrix<float, R, R> kalmanfilter::calculatePredictedCovariance()
{
    currentCovarianceMatrix.at(0, 1) = 0.0f;
    currentCovarianceMatrix.at(1, 0) = 0.0f;
    predictedCovarianceMatrix = A_t * currentCovarianceMatrix * A_t.transpose();
    predictedCovarianceMatrix.at(0, 1) = 0.0f;
    predictedCovarianceMatrix.at(1, 0) = 0.0f;
    std::cout << "calculatePredictedCovariance: " << A_t.to_string() << " * " << currentCovarianceMatrix.to_string() << " * " << A_t.transpose().to_string() << " = " << predictedCovarianceMatrix.to_string() << std::endl;
    return predictedCovarianceMatrix;
}

Matrix<float, R, R> kalmanfilter::calculateKalmanGain()
{
    kalmanGain = predictedCovarianceMatrix * (predictedCovarianceMatrix + sensorCovarianceMatrix).inverse();
    //kalmanGain = predictedCovarianceMatrix * C_t.transpose() * (C_t * predictedCovarianceMatrix * C_t.transpose() + Q_t).inverse();

    if(isdebug){std::cout << "calculateKalmanGain: " << predictedCovarianceMatrix.to_string() << " / (" << predictedCovarianceMatrix.to_string() << " + " << sensorCovarianceMatrix.to_string() << ") = " << kalmanGain.to_string() << std::endl;}
    return kalmanGain;
}

Matrix<float, R, C> kalmanfilter::calculateMeasurementVector(Matrix<float, R, C> measurement)
{
    Z_t = C_t * measurement;
    return Matrix<float, R, C>();
}


Matrix<float, R, 1> kalmanfilter::calculateAdjustedStateVector()
{
    adjustedStateVector = predictedStateVector + kalmanGain * (Z_t - C_t * predictedStateVector); // U_t = U^_t + K_t * (U^_t)
    //adjustedStateVector = predictedStateVector + kalmanGain * (measurement - predictedStateVector); // U_t = U^_t + K_t * (Z_t - U^_t)
    if(isdebug){std::cout << "calculateAdjustedStateVector: " << predictedStateVector.to_string() << " + " << kalmanGain.to_string() << " * (" << Z_t.to_string() << " - " << C_t.to_string() << " * " << predictedStateVector.to_string() << ") = " << adjustedStateVector.to_string() << std::endl;}
    return adjustedStateVector;
}

Matrix<float, R, R> kalmanfilter::calculateAdjustedCovariance()
{

    adjustedCovarianceMatrix = (I - kalmanGain * C_t) * predictedCovarianceMatrix;
    if(isdebug){std::cout << "calculateAdjustedCovariance: " << I.to_string() << kalmanGain.to_string() << " * " << C_t.to_string() << ") * " << predictedCovarianceMatrix.to_string() << " = " << adjustedCovarianceMatrix.to_string() << std::endl;}
    return adjustedCovarianceMatrix;
}
