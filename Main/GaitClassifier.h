#ifndef GAIT_CLASSIFIER_H
#define GAIT_CLASSIFIER_H

#include "Config.h"
#include <math.h>

// Step classification result — emitted at the moment a step completes.
// (Top-level activity state like STATIONARY/FIDGETING/WALKING lives in MotionClassifier.h)
enum StepType { NO_STEP, STEP_CORRECT, STEP_INCORRECT };

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

    // Tracks when force first dropped below threshold during STANCE.
    // 0 means currently loaded; non-zero means force has been below
    // threshold since that time.
    unsigned long liftOffStartTime = 0;

    // A step must last at least this long to count (filters noise)
    static const unsigned long MIN_STANCE_MS = 150;
    // Foot must be off ground at least this long before next step
    static const unsigned long MIN_SWING_MS = 100;
    // Stance longer than this is standing still, not a step
    static const unsigned long MAX_STANCE_MS = 1500;
    // Brief gap between heel-off and toe-press that should NOT end stance.
    // Bridges the moment during a normal stride where both sensors briefly
    // drop below threshold as weight rolls forward from heel to toe.
    static const unsigned long STANCE_GRACE_MS = 200;

public:
    // Call this every sample (~20ms at 50Hz).
    // Returns STEP_CORRECT / STEP_INCORRECT when a step completes,
    // or NO_STEP if no step event this sample.
    StepType processSample(int heelForce, int toeForce, unsigned long now) {
        bool heelOn = (heelForce > FORCE_THRESHOLD);
        bool toeOn  = (toeForce  > FORCE_THRESHOLD);
        bool anyForce = heelOn || toeOn;

        switch (phase) {
            case SWING:
                if (anyForce) {
                    // Require minimum swing time before accepting new step
                    if (now - swingStartTime < MIN_SWING_MS) {
                        return NO_STEP;
                    }
                    // Foot just landed — start a new stance phase
                    phase = STANCE;
                    stanceStartTime = now;
                    liftOffStartTime = 0;
                    heelActivated = heelOn;
                    toeActivated  = toeOn;
                }
                return NO_STEP; // No step event yet

            case STANCE:
                if (anyForce) {
                    // Still on the ground — keep tracking which sensors fire
                    if (heelOn) heelActivated = true;
                    if (toeOn)  toeActivated  = true;
                    liftOffStartTime = 0;  // reset grace timer; force is back
                    return NO_STEP;
                }

                // Force dropped — but don't exit stance immediately.  Normal
                // strides have a brief moment where weight transitions from
                // heel to toe and both sensors fall below threshold.  Wait
                // STANCE_GRACE_MS for force to return before declaring the
                // stance truly ended.
                if (liftOffStartTime == 0) {
                    liftOffStartTime = now;   // start grace timer
                }
                if (now - liftOffStartTime < STANCE_GRACE_MS) {
                    return NO_STEP;           // still within grace window
                }

                // Grace expired — stance really has ended.  Use liftOffStartTime
                // as the "true" stance end so the duration check isn't inflated
                // by the grace period.
                {
                    unsigned long stanceDuration = liftOffStartTime - stanceStartTime;

                    // Too short — probably noise, discard
                    if (stanceDuration < MIN_STANCE_MS) {
                        phase = SWING;
                        swingStartTime = now;
                        liftOffStartTime = 0;
                        return NO_STEP;
                    }

                    // Too long — they were standing still, not stepping
                    if (stanceDuration > MAX_STANCE_MS) {
                        phase = SWING;
                        swingStartTime = now;
                        liftOffStartTime = 0;
                        return NO_STEP;
                    }

                    // Valid step completed — classify it
                    phase = SWING;
                    swingStartTime = now;
                    liftOffStartTime = 0;

                    if (heelActivated && toeActivated) {
                        return STEP_CORRECT;
                    } else if (heelActivated || toeActivated) {
                        return STEP_INCORRECT;
                    }
                    return NO_STEP; // Neither sensor fired strongly enough (shouldn't happen)
                }
        }
        return NO_STEP;
    }

    // Reset state machine — call when leaving WALKING so stale stance state
    // doesn't produce a bogus step event on the next walking transition.
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
