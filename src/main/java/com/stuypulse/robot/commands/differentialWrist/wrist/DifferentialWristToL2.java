package com.stuypulse.robot.commands.differentialWrist.wrist;

import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist.WristState;

public class DifferentialWristToL2 extends DifferentialWristSetWristState {
    public DifferentialWristToL2() {
        super(WristState.CORAL_SCORE_L2);
    }
}
