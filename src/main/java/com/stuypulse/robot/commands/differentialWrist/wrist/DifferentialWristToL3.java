package com.stuypulse.robot.commands.differentialWrist.wrist;

import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist.WristState;

public class DifferentialWristToL3 extends DifferentialWristSetWristState {
    public DifferentialWristToL3() {
        super(WristState.CORAL_SCORE_L3);
    }
}