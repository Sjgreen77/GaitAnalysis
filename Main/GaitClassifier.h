#ifndef GAIT_CLASSIFIER_H
#define GAIT_CLASSIFIER_H

#include "Config.h"
#include <math.h>

enum MotionState { STATIONARY, FIDGETING, WALKING_CORRECT, WALKING_INCORRECT, UNKNOWN };

class GaitClassifier {
private:
    float ay_buffer[WINDOW_SIZE];
    float gy_buffer[WINDOW_SIZE];
    int heel_buffer[WINDOW_SIZE];
    int toe_buffer[WINDOW_SIZE];
    
    int insert_index = 0;
    bool buffer_full = false;

public:
    void addSample(float ay, float gy, int heelForce, int toeForce) {
        ay_buffer[insert_index] = ay;
        gy_buffer[insert_index] = gy;
        heel_buffer[insert_index] = heelForce;
        toe_buffer[insert_index] = toeForce;
        
        insert_index++;
        if (insert_index >= WINDOW_SIZE) {
            buffer_full = true;
            insert_index = 0;
        }
    }

    bool isReady() {
        bool ready = buffer_full;
        if (ready) buffer_full = false; // Reset flag after reading
        return ready;
    }

    MotionState classifyWindow() {
        // 1. Calculate IMU Features
        float sum_ay = 0, sum_gy = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            sum_ay += ay_buffer[i];
            sum_gy += gy_buffer[i];
        }
        float mean_ay = sum_ay / WINDOW_SIZE;
        float mean_gy = sum_gy / WINDOW_SIZE;

        float sum_sq_ay = 0, sum_sq_gy = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            sum_sq_ay += pow(ay_buffer[i] - mean_ay, 2);
            sum_sq_gy += pow(gy_buffer[i] - mean_gy, 2);
        }
        float std_ay = sqrt(sum_sq_ay / (WINDOW_SIZE - 1));
        float std_gy = sqrt(sum_sq_gy / (WINDOW_SIZE - 1));

        // 2. Decision Tree for Motion State
        if (std_ay < 0.0326435) return STATIONARY;
        if (mean_ay >= -0.213568 && std_gy < 60.6775) return FIDGETING;
        
        // 3. If Walking, evaluate gait using ADC buffers
        return evaluateGait();
    }

private:
    MotionState evaluateGait() {
        int max_heel = 0;
        int max_toe = 0;

        // Find the peak force exerted during this 1-second walking window
        for (int i = 0; i < WINDOW_SIZE; i++) {
            if (heel_buffer[i] > max_heel) max_heel = heel_buffer[i];
            if (toe_buffer[i] > max_toe) max_toe = toe_buffer[i];
        }

        // Logic: Correct step requires heel strike. Incorrect is toe only.
        if (max_heel > FORCE_THRESHOLD) {
            return WALKING_CORRECT;
        } else if (max_toe > FORCE_THRESHOLD) {
            return WALKING_INCORRECT;
        }
        return UNKNOWN; // Failsafe
    }
};

#endif