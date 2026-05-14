#ifndef GAIT_CLASSIFIER_H
#define GAIT_CLASSIFIER_H

#include "Config.h"
#include <math.h>

// step classification result, given at the moment a step completes.
enum StepType { NO_STEP, STEP_CORRECT, STEP_INCORRECT };

// step detection state machine
enum StepPhase { SWING, STANCE };

class GaitClassifier {
private:
    StepPhase phase = SWING;

    // track whether each sensor was activated during the current step
    bool heelActivated = false;
    bool toeActivated  = false;

    // timing for debounce
    unsigned long stanceStartTime = 0;
    unsigned long swingStartTime  = 0;

    // tracks when force first dropped below threshold during stance
    // 0 means currently loaded, non zero means force has been below
    // threshold since that time
    unsigned long liftOffStartTime = 0;

    // a step must last at least this long to count (filters noise)
    static const unsigned long MIN_STANCE_MS = 150;
    // foot must be off ground at least this long before next step
    static const unsigned long MIN_SWING_MS = 100;
    // stance longer than this is standing still, not a step
    static const unsigned long MAX_STANCE_MS = 1500;
    // brief gap between heel-off and toe-press that should NOT end the stance,
    // bridges the moment during a normal stride where both sensors briefly
    // drop below threshold as weight rolls forward from heel to toe
    static const unsigned long STANCE_GRACE_MS = 200;

public:
    // call this every sample (~20ms at 50Hz)
    // gives STEP_CORRECT / STEP_INCORRECT when a step completes,
    // or NO_STEP if no step
    StepType processSample(int heelForce, int toeForce, unsigned long now) {
        bool heelOn = (heelForce > FORCE_THRESHOLD);
        bool toeOn  = (toeForce  > FORCE_THRESHOLD);
        bool anyForce = heelOn || toeOn;

        switch (phase) {
            case SWING:
                if (anyForce) {
                    // require minimum swing time before accepting new step
                    if (now - swingStartTime < MIN_SWING_MS) {
                        return NO_STEP;
                    }
                    // foot just landed, start a new stance phase
                    phase = STANCE;
                    stanceStartTime = now;
                    liftOffStartTime = 0;
                    heelActivated = heelOn;
                    toeActivated  = toeOn;
                }
                return NO_STEP; // no step event yet

            case STANCE:
                if (anyForce) {
                    // still on the ground, keep tracking which sensors fire
                    if (heelOn) heelActivated = true;
                    if (toeOn)  toeActivated  = true;
                    liftOffStartTime = 0;  // reset grace timer, force is back
                    return NO_STEP;
                }

                // force dropped but don't exit stance immediately
                // Normal strides have a brief moment where weight transitions from
                // heel to toe and both sensors fall below threshold.  
                // Wait stance_grace_ms for force to return before declaring the
                // stance truly ended
                if (liftOffStartTime == 0) {
                    liftOffStartTime = now;   // start grace timer
                }
                if (now - liftOffStartTime < STANCE_GRACE_MS) {
                    return NO_STEP;           // still within grace window
                }

                // grace expired, stance really has ended. Now use liftOffStartTime
                // as the "true" stance end so the duration check isn't inflated
                // by the grace period
                {
                    unsigned long stanceDuration = liftOffStartTime - stanceStartTime;

                    // too short, probably noise, discard
                    if (stanceDuration < MIN_STANCE_MS) {
                        phase = SWING;
                        swingStartTime = now;
                        liftOffStartTime = 0;
                        return NO_STEP;
                    }

                    // too long, they were standing still, not stepping
                    if (stanceDuration > MAX_STANCE_MS) {
                        phase = SWING;
                        swingStartTime = now;
                        liftOffStartTime = 0;
                        return NO_STEP;
                    }

                    // valid step completed, classify it
                    phase = SWING;
                    swingStartTime = now;
                    liftOffStartTime = 0;

                    if (heelActivated && toeActivated) {
                        return STEP_CORRECT;
                    } else if (heelActivated || toeActivated) {
                        return STEP_INCORRECT;
                    }
                    return NO_STEP; // neither sensor fired strongly enough (shouldn't happen)
                }
        }
        return NO_STEP;
    }

    // reset state machine, call when leaving walking so old stance state
    // doesn't produce an incorrect step event on the next walking transition
    void reset() {
        phase = SWING;
        heelActivated = false;
        toeActivated = false;
        swingStartTime = 0;
        stanceStartTime = 0;
        liftOffStartTime = 0;
    }
};

#endif
