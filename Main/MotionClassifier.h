#ifndef MOTION_CLASSIFIER_H
#define MOTION_CLASSIFIER_H

#include "Config.h"
#include <math.h>

// Top-level activity classification — produced once per WINDOW_SIZE samples
// (default 50 samples = 1 second at 50Hz).
enum MotionState { STATIONARY, FIDGETING, WALKING, RUNNING };

class MotionClassifier {
private:
    // Rolling 1-second buffers for each feature signal we need.
    // We only buffer the axes used by the decision tree — not every IMU channel.
    float axBuf[WINDOW_SIZE];
    float azBuf[WINDOW_SIZE];
    float gxBuf[WINDOW_SIZE];
    float gzBuf[WINDOW_SIZE];
    float accMagBuf[WINDOW_SIZE];
    int   heelBuf[WINDOW_SIZE];
    int   toeBuf[WINDOW_SIZE];

    int idx = 0;
    MotionState currentState = STATIONARY;

    // Asymmetric hysteresis: number of consecutive same-classification
    // windows needed before committing to a state change.
    //   - ENTER_WINDOWS: how many to move into any non-WALKING state
    //   - LEAVE_WALKING_WINDOWS: harder to leave WALKING than to enter it,
    //     so a brief classifier blip won't interrupt step counting.
    static const int ENTER_WINDOWS         = 2;
    static const int LEAVE_WALKING_WINDOWS = 3;
    MotionState pendingState  = STATIONARY;
    int         pendingCount  = 0;

    // Decision tree exported from MATLAB Classification Learner (Fine Tree).
    // Each branch mirrors one node in the text-mode dump.
    MotionState classifyWindow(float max_toe, float std_ax, float mean_accMag,
                                float heelToeRatio, float std_gz, float mean_gx,
                                float max_heel, float mean_az) {
        if (max_toe < 376.5f) {
            // Node 2
            if (std_ax < 0.05086f) {
                return STATIONARY;           // Node 4
            }
            // Node 5
            if (heelToeRatio < 63.9167f) {
                // Node 8
                if (mean_accMag < 1.02508f) {
                    return STATIONARY;       // Node 12
                }
                // Node 13
                if (mean_az < 0.872302f) {
                    // Node 20
                    if (std_ax < 0.086008f) return WALKING;   // Node 24
                    return FIDGETING;                         // Node 25
                }
                return WALKING;              // Node 21
            }
            // Node 9
            if (max_heel < 806.0f) {
                // Node 14
                if (max_heel < 657.0f) return WALKING;        // Node 22
                return RUNNING;                               // Node 23
            }
            return FIDGETING;                // Node 15
        }

        // Node 3
        if (mean_accMag < 1.73156f) {
            // Node 6
            if (std_gz < 88.6074f) {
                // Node 10
                if (std_gz < 32.0498f) return FIDGETING;      // Node 16
                return WALKING;                               // Node 17
            }
            // Node 11
            if (mean_gx < -12.2815f) return WALKING;          // Node 18
            return RUNNING;                                   // Node 19
        }
        return RUNNING;                      // Node 7
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

public:
    // Call every sample (50Hz).  Returns the most recent classified state —
    // which only changes once per window (every 50 samples / 1 second).
    MotionState processSample(float ax, float ay, float az,
                              float gx, float gy, float gz,
                              int heel, int toe) {
        axBuf[idx] = ax;
        azBuf[idx] = az;
        gxBuf[idx] = gx;
        gzBuf[idx] = gz;
        accMagBuf[idx] = sqrtf(ax*ax + ay*ay + az*az);
        heelBuf[idx] = heel;
        toeBuf[idx]  = toe;
        idx++;

        if (idx >= WINDOW_SIZE) {
            idx = 0;

            // Aggregate features across the full window
            float sum_ax = 0, sumSq_ax = 0;
            float sum_az = 0;
            float sum_gx = 0;
            float sum_gz = 0, sumSq_gz = 0;
            float sum_accMag = 0;
            float sum_heel = 0, sum_toe = 0;
            int max_heel = 0, max_toe = 0;

            for (int i = 0; i < WINDOW_SIZE; i++) {
                sum_ax     += axBuf[i];
                sumSq_ax   += axBuf[i] * axBuf[i];
                sum_az     += azBuf[i];
                sum_gx     += gxBuf[i];
                sum_gz     += gzBuf[i];
                sumSq_gz   += gzBuf[i] * gzBuf[i];
                sum_accMag += accMagBuf[i];
                sum_heel   += heelBuf[i];
                sum_toe    += toeBuf[i];
                if (heelBuf[i] > max_heel) max_heel = heelBuf[i];
                if (toeBuf[i]  > max_toe)  max_toe  = toeBuf[i];
            }

            const float N = (float)WINDOW_SIZE;
            float mean_ax      = sum_ax     / N;
            float mean_az      = sum_az     / N;
            float mean_gx      = sum_gx     / N;
            float mean_gz      = sum_gz     / N;
            float mean_accMag  = sum_accMag / N;
            float mean_heel    = sum_heel   / N;
            float mean_toe     = sum_toe    / N;

            float var_ax = (sumSq_ax / N) - mean_ax * mean_ax;
            float var_gz = (sumSq_gz / N) - mean_gz * mean_gz;
            float std_ax = sqrtf(var_ax < 0 ? 0 : var_ax);
            float std_gz = sqrtf(var_gz < 0 ? 0 : var_gz);

            // Match the MATLAB formula: mean_heel / (mean_toe + 1)
            float heelToeRatio = mean_heel / (mean_toe + 1.0f);

            MotionState rawState = classifyWindow(
                (float)max_toe, std_ax, mean_accMag,
                heelToeRatio, std_gz, mean_gx,
                (float)max_heel, mean_az);

            // --- Toe-walking override ---
            // The decision tree was trained primarily on heel-strike walking,
            // so toe-dominant gaits (as with idiopathic toe walking — which is
            // exactly what this device is meant to detect!) can fall through
            // to a FIDGETING leaf.  Rescue them: strong toe loading plus body
            // motion isn't fidgeting, it's walking.
            if (rawState == FIDGETING &&
                (float)max_toe > 400.0f &&
                mean_accMag > 1.05f) {
                rawState = WALKING;
            }

            // --- Asymmetric hysteresis ---
            // Entering any state (including WALKING) requires ENTER_WINDOWS
            // confirmations.  Leaving WALKING requires LEAVE_WALKING_WINDOWS
            // — brief classifier blips shouldn't interrupt step counting.
            int requiredWindows =
                (currentState == WALKING) ? LEAVE_WALKING_WINDOWS : ENTER_WINDOWS;

            if (rawState == currentState) {
                // Still in same state, clear any pending change
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
                // New candidate state — start counting
                pendingState = rawState;
                pendingCount = 1;
            }
        }

        return currentState;
    }

    MotionState getState() const { return currentState; }
};

#endif
