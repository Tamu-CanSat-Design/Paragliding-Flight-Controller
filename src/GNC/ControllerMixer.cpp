/**********************************************************************
 CONTROL MIXING CONCEPT

 Roll control  → differential brake
 Pitch control → symmetric brake

 Mixing equations:

    Left  = Pitch_cmd + Roll_cmd
    Right = Pitch_cmd - Roll_cmd

 Why this works:

    If Roll_cmd = 0:
        Left  = Pitch
        Right = Pitch
        → Both pull equally → symmetric brake → pitch control

    If Pitch_cmd = 0:
        Left  = +Roll
        Right = -Roll
        → One pulls more than the other → turning

    If both active:
        Pitch sets baseline brake
        Roll adds steering offset

**********************************************************************/

#include <algorithm>

struct MixerOutput {
    float left;   // Left toggle/brake command  [0, 100]
    float right;  // Right toggle/brake command [0, 100]
};


class ControllerMixer {
public:
    /**
     * Mix pitch (symmetric brake) and roll (differential) commands
     * into left / right parafoil toggle outputs.
     *
     * @param pitchCmd  Symmetric brake command from DescentController [0, 100]
     *                  0 = no brake, 100 = full brake
     * @param rollCmd   Differential command from RollController [-100, 100]
     *                  Positive = turn right (pull left toggle more)
     * @return MixerOutput with left and right in [0, 100]
     */
    MixerOutput Mix(float pitchCmd, float rollCmd)
    {
        MixerOutput out;

        // Raw mix: Left = Pitch + Roll,  Right = Pitch - Roll
        float rawLeft  = pitchCmd + rollCmd;
        float rawRight = pitchCmd - rollCmd;

        // Clamp to actuator limits [0, 100]
        out.left  = Clamp(rawLeft,  0.0f, 100.0f);
        out.right = Clamp(rawRight, 0.0f, 100.0f);

        return out;
    }

    /**
     * Full pipeline: take raw PID outputs and produce servo-ready commands.
     *
     * @param descentPID  Raw PID output from DescentController [-100, 100]
     * @param rollPID     Raw PID output from RollController    [-100, 100]
     * @return MixerOutput with left and right in [0, 100]
     */
    MixerOutput Compute(float descentPID, float rollPID)
    {
        // Convert descent PID → symmetric brake [0, 100]
        // Positive PID = falling too fast → more brake
        float pitchCmd = 50.0f + descentPID * 0.5f;
        pitchCmd = Clamp(pitchCmd, 0.0f, 100.0f);

        // Roll PID is already [-100, 100]; scale to a reasonable
        // differential offset (half range keeps headroom for pitch)
        float rollCmd = rollPID * 0.5f;

        return Mix(pitchCmd, rollCmd);
    }

private:
    static float Clamp(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
};
