package com.stuypulse.robot.commands.differentialWrist.wrist;

import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist.WristState;

public class DifferentialWristToL1 extends DifferentialWristSetWristState {
    public DifferentialWristToL1() {
        super(WristState.CORAL_SCORE_L1);
    }
}
