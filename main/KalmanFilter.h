#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    float q;  // Quá trình nhiễu
    float r;  // Đo lường nhiễu
    float x;  // Dự đoán tọa độ
    float p;  // Dự đoán sai số
    float k;  // Hệ số Kalman
} KalmanFilter_t;

void kalmanFilterInit(KalmanFilter_t* filter, float q, float r) {
    filter->q = q;
    filter->r = r;
    filter->x = 0.0f;
    filter->p = 0.0f;
    filter->k = 0.0f;
}

float kalmanFilterUpdate(KalmanFilter_t* filter, float measurement) {
    // Predict
    filter->x = filter->x;
    filter->p = filter->p + filter->q;

    // Update
    filter->k = filter->p / (filter->p + filter->r);
    filter->x = filter->x + filter->k * (measurement - filter->x);
    filter->p = (1 - filter->k) * filter->p;

    return filter->x;
}

#endif /* KALMAN_FILTER_H */
