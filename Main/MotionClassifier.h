#ifndef MOTION_CLASSIFIER_H
#define MOTION_CLASSIFIER_H

#include "Config.h"
#include <math.h>

// top level activity classification, produced once per WINDOW_SIZE samples
// (default 50 samples,  1 second at 50Hz).
enum MotionState { STATIONARY, FIDGETING, WALKING, RUNNING };

class MotionClassifier {
private:
    // running sums, updated each sample, reset each window
    // cheaper than keeping full per sample buffers and gives us all the
    // features the new rertrained decision tree needs
    float sum_ax = 0,    sumSq_ax = 0;
    float sum_ay = 0,    sumSq_ay = 0;
    float sum_az = 0;
    float sum_gx = 0,    sumSq_gx = 0;
    float sum_gz = 0,    sumSq_gz = 0;
    float sum_accMag = 0;
    float sum_gyroMag = 0;
    float sum_heel = 0,  sumSq_heel = 0;
    float sum_toe  = 0,  sumSq_toe  = 0;
    int   sampleCount = 0;

    MotionState currentState = STATIONARY;

    // asymm hysteresis, number of consecutive same-classification
    // windows needed before committing to a state change
    static const int ENTER_WINDOWS         = 2;
    static const int LEAVE_WALKING_WINDOWS = 3;
    MotionState pendingState  = STATIONARY;
    int         pendingCount  = 0;

    // Decision tree exported from matlab Classification Learner (fine dec tree)
    // branches mirror the text-mode node dump one-for-one.
    MotionState classifyWindow(float std_toe,     float std_ax,   float mean_accMag,
                                float heelToeRatio, float std_ay,   float std_gx,
                                float mean_az,     float std_heel, float std_gz,
                                float mean_heel,   float mean_gyroMag) {
        if (std_toe < 93.5271f) {
            
            if (std_ax < 0.05086f) {
                return STATIONARY;                               
            }
          
            if (heelToeRatio < 63.9167f) {
     
                if (mean_az < 0.872302f) return FIDGETING;        
                return WALKING;                                   
            }
          
            if (std_ax < 0.279341f) {
              
                if (std_heel < 170.121f) {
                
                    if (std_ax < 0.0748567f) return WALKING;      
                    return RUNNING;                                
                }
                return FIDGETING;                                 
            }
            return WALKING;                                       
        }

        
        if (mean_accMag < 1.73156f) {
           
            if (std_ay < 0.13274f) return FIDGETING;            
           
            if (std_gz < 88.6074f) {
               
                if (mean_heel < 362.3f) return WALKING;           
                return RUNNING;                                    
            }
           
            if (mean_gyroMag < 181.119f) return WALKING;      
            return RUNNING;                                      
        }

     
        if (std_gx < 63.7776f) return WALKING;                  
       
        if (mean_heel < 212.25f) return RUNNING;              
        return WALKING;                                         
    }

    const char* stateName(MotionState s) {
        switch (s) {
            case STATIONARY: return "STATIONARY";
            case FIDGETING:  return "FIDGETING";
            case WALKING:    return "WALKING";
            case RUNNING:    return "RUNNING";
        }
        return "?";
    }

    void resetWindow() {
        sum_ax = sumSq_ax = 0;
        sum_ay = sumSq_ay = 0;
        sum_az = 0;
        sum_gx = sumSq_gx = 0;
        sum_gz = sumSq_gz = 0;
        sum_accMag = 0;
        sum_gyroMag = 0;
        sum_heel = sumSq_heel = 0;
        sum_toe  = sumSq_toe  = 0;
        sampleCount = 0;
    }

public:
    // call every sample (50Hz), returns the most recent classified state
    // which only changes once per window (every 50 samples / 1 second).
    MotionState processSample(float ax, float ay, float az,
                              float gx, float gy, float gz,
                              int heel, int toe) {
        float accMag  = sqrtf(ax*ax + ay*ay + az*az);
        float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
        float fh = (float)heel;
        float ft = (float)toe;

        sum_ax     += ax;   sumSq_ax   += ax * ax;
        sum_ay     += ay;   sumSq_ay   += ay * ay;
        sum_az     += az;
        sum_gx     += gx;   sumSq_gx   += gx * gx;
        sum_gz     += gz;   sumSq_gz   += gz * gz;
        sum_accMag += accMag;
        sum_gyroMag += gyroMag;
        sum_heel   += fh;   sumSq_heel += fh * fh;
        sum_toe    += ft;   sumSq_toe  += ft * ft;
        sampleCount++;

        if (sampleCount >= WINDOW_SIZE) {
            const float N = (float)WINDOW_SIZE;

            float mean_ax      = sum_ax      / N;
            float mean_ay      = sum_ay      / N;
            float mean_az      = sum_az      / N;
            float mean_gx      = sum_gx      / N;
            float mean_gz      = sum_gz      / N;
            float mean_heel    = sum_heel    / N;
            float mean_toe     = sum_toe     / N;
            float mean_accMag  = sum_accMag  / N;
            float mean_gyroMag = sum_gyroMag / N;

            float var_ax   = (sumSq_ax   / N) - mean_ax   * mean_ax;
            float var_ay   = (sumSq_ay   / N) - mean_ay   * mean_ay;
            float var_gx   = (sumSq_gx   / N) - mean_gx   * mean_gx;
            float var_gz   = (sumSq_gz   / N) - mean_gz   * mean_gz;
            float var_heel = (sumSq_heel / N) - mean_heel * mean_heel;
            float var_toe  = (sumSq_toe  / N) - mean_toe  * mean_toe;

            float std_ax   = sqrtf(var_ax   < 0 ? 0 : var_ax);
            float std_ay   = sqrtf(var_ay   < 0 ? 0 : var_ay);
            float std_gx   = sqrtf(var_gx   < 0 ? 0 : var_gx);
            float std_gz   = sqrtf(var_gz   < 0 ? 0 : var_gz);
            float std_heel = sqrtf(var_heel < 0 ? 0 : var_heel);
            float std_toe  = sqrtf(var_toe  < 0 ? 0 : var_toe);

            // macth the MATLAB formula, mean_heel / (mean_toe + 1)
            float heelToeRatio = mean_heel / (mean_toe + 1.0f);

            MotionState rawState = classifyWindow(
                std_toe, std_ax, mean_accMag, heelToeRatio,
                std_ay, std_gx, mean_az, std_heel, std_gz,
                mean_heel, mean_gyroMag);

            // toe walking fix
            float footForce = mean_heel + mean_toe;
            if (rawState == FIDGETING &&
                footForce   > 150.0f  &&   // any FSR actually loaded
                std_ax      > 0.35f   &&   // rhythmic forward/back motion
                mean_gyroMag > 60.0f) {    // leg really swinging
                rawState = WALKING;
            }

            // asymm hysyteriss
            // entering any state (including walking) requires ENTER_WINDOWS
            // confirmations but leaving walking requires LEAVE_WALKING_WINDOWS
            // brief classifier blips shouldn't interrupt step counting
            int requiredWindows =
                (currentState == WALKING) ? LEAVE_WALKING_WINDOWS : ENTER_WINDOWS;

            if (rawState == currentState) {
                // still in same state, clear any pending change
                pendingCount = 0;
            } else if (rawState == pendingState) {
                pendingCount++;
                if (pendingCount >= requiredWindows) {
                    Serial.print("[MOTION] ");
                    Serial.print(stateName(currentState));
                    Serial.print(" -> ");
                    Serial.println(stateName(rawState));
                    currentState = rawState;
                    pendingCount = 0;
                }
            } else {
                // new candidate state, start counting
                pendingState = rawState;
                pendingCount = 1;
            }

            resetWindow();
        }

        return currentState;
    }

    MotionState getState() const { return currentState; }
};

#endif
