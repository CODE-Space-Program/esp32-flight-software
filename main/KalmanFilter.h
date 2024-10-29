#pragma once
// basic implementation of a 1D Kalman filter for height and velocity
struct KalmanFilter {

    float dt;       // time step
    float A[2][2];  // state transition matrix
    float B[2];     // control input matrix
    float H[1][2];  // Measurement matrix
    float Q[2][2];  // Process noise covariance
    float R[1][1];  // measurement noise matrix
    float P[2][2];  // estimate covariance
    float x[2];     // state estimate (height, velocity)

    // initialize kalman filter
    void init(float init_height, float init_velocity, float dt) {

        this->dt = dt;
        x[0] = init_height;     // initial heigh
        x[1] = init_velocity;   // initial velocity

        // A matrix (State transition)
        A[0][0] = 1; A[0][1] = dt;
        A[1][0] = 0; A[1][1] = 1;

        // B matrix (Control input for acceleration)
        B[0] = 0.5 * dt * dt;
        B[1] = dt;

        // H matrix (Measurement function)
        H[0][0] = 1; H[0][1] = 0;

        // Process noise covariance (we have to adjust these values based on experimenting)
        Q[0][0] = 0.001; Q[0][1] = 0;
        Q[1][0] = 0;     Q[1][1] = 0.003;

        // Measurement noise covariance (adjust these based on sensor noise)
        R[0][0] = 0.02; // this is the recommended value from the manufacturer, we might have to increase it to .04 or .05 because of the rapid change in altitude and vibrations during the flight

        // initial covariance matrix
        P[0][0] = 1; P[0][1] = 0;
        P[1][0] = 0; P[1][1] = 1;
    }

    // Predict step
    void predict(float acc) {

        // predict the state
        float x_pred[2];
        x_pred[0] = A[0][0] * x[0] + A[0][1] * x[1] + B[0] * acc;
        x_pred[1] = A[1][0] * x[0] + A[1][1] * x[1] + B[1] * acc;

        // predict the covariance
        float P_pred[2][2];
        P_pred[0][0] = A[0][0] * P[0][0] + A[0][1] * P[1][0];
        P_pred[0][1] = A[0][0] * P[0][1] + A[0][1] * P[1][1];   
        P_pred[1][0] = A[1][0] * P[0][0] + A[1][1] * P[1][0];
        P_pred[1][1] = A[1][0] * P[0][1] + A[1][1] * P[1][1];

        // add process noise
        P_pred[0][0] += Q[0][0];
        P_pred[1][1] += Q[1][1];

        // update the state and covariance
        x[0] = x_pred[0];
        x[1] = x_pred[1];
        P[0][0] = P_pred[0][0];
        P[0][1] = P_pred[0][1];
        P[1][0] = P_pred[1][0];
        P[1][1] = P_pred[1][1];
    }

    // update step with measurement (height)
    void update(float height_measured) {

        // inovation (residual)
        float y = height_measured - (H[0][0] * x[0] + H[0][1] * x[1]);

        // inovation covariance
        float S = H[0][0] * P[0][0] + H[0][1] * P[1][0] + R[0][0];

        // Kalman gain
        float K[2];
        K[0] = (P[0][0] * H[0][0] + P[0][1] * H[0][1]) / S;
        K[1] = (P[1][0] * H[0][0] + P[1][1] * H[0][1]) / S;

        // update the state
        x[0] += K[0] * y;
        x[1] += K[1] * y;

        // update the covariance
        P[0][0] -= K[0] * H[0][0] * P[0][0];
        P[0][1] -= K[0] * H[0][1] * P[0][1];
        P[1][0] -= K[1] * H[0][0] * P[1][0];
        P[1][1] -= K[1] * H[0][1] * P[1][1];
    }

    // get the estimated height
    float get_estimated_height() {
        return x[0];
    }

    // get the estimated velocity
    float get_estimated_velocity() {
        return x[1];
    }
};


