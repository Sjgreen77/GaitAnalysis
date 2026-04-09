#ifndef GAIT_CLASSIFIER_H
#define GAIT_CLASSIFIER_H

#include "Config.h"
#include <math.h>

enum MotionState { STATIONARY, FIDGETING, WALKING_CORRECT, WALKING_INCORRECT, UNKNOWN };

// Step detection state machine
enum StepPhase { SWING, STANCE };

class GaitClassifier {
private:
    StepPhase phase = SWING;

    // Track whether each sensor was activated during the current step
    bool heelActivated = false;
    bool toeActivated  = false;

    // Timing for debounce
    unsigned long stanceStartTime = 0;
    unsigned long swingStartTime  = 0;

    // A step must last at least this long to count (filters noise)
    static const unsigned long MIN_STANCE_MS = 150;
    // Foot must be off ground at least this long before next step
    static const unsigned long MIN_SWING_MS = 100;

public:
    // Call this every sample (~20ms at 50Hz).
    // Returns WALKING_CORRECT, WALKING_INCORRECT when a step completes,
    // or UNKNOWN if no step event this sample.
    MotionState processSample(int heelForce, int toeForce, unsigned long now) {
        bool heelOn = (heelForce > FORCE_THRESHOLD);
        bool toeOn  = (toeForce  > FORCE_THRESHOLD);
        bool anyForce = heelOn || toeOn;

        switch (phase) {
            case SWING:
                if (anyForce) {
                    // Require minimum swing time before accepting new step
                    if (now - swingStartTime < MIN_SWING_MS) {
                        return UNKNOWN;
                    }
                    // Foot just landed — start a new stance phase
                    phase = STANCE;
                    stanceStartTime = now;
                    heelActivated = heelOn;
                    toeActivated  = toeOn;
                }
                return UNKNOWN; // No step event yet

            case STANCE:
                if (anyForce) {
                    // Still on the ground — keep tracking which sensors fire
                    if (heelOn) heelActivated = true;
                    if (toeOn)  toeActivated  = true;
                    return UNKNOWN;
                } else {
                    // Force dropped — foot just lifted off
                    // Check if stance was long enough to be a real step
                    if (now - stanceStartTime < MIN_STANCE_MS) {
                        // Too short — probably noise, discard
                        phase = SWING;
                        swingStartTime = now;
                        return UNKNOWN;
                    }

                    // Valid step completed — classify it
                    phase = SWING;
                    swingStartTime = now;

                    if (heelActivated && toeActivated) {
                        return WALKING_CORRECT;
                    } else if (heelActivated || toeActivated) {
                        return WALKING_INCORRECT;
                    }
                    return UNKNOWN; // Neither sensor fired strongly enough (shouldn't happen)
                }
        }
        return UNKNOWN;
    }
};

#endif
