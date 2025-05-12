package com.stuypulse.robot.subsystems.superStructure.telescope;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

public class TelescopeSim extends Telescope {

    protected TelescopeSim() {
        //TBD
    }

    // @Override
    // public double getCurrentHeight() {
       
    // }

    private double getTargetHeight() {
        return getState().getTargetHeight();
    }

    private boolean isWithinTolerance(double heightToleranceMeters) {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < heightToleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.Telescope.HEIGHT_TOLERANCE_METERS);
    }


    
}
