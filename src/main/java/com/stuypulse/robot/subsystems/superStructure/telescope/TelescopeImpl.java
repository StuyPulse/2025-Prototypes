package com.stuypulse.robot.subsystems.superStructure.telescope;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

public class TelescopeImpl extends Telescope {
    private final TalonFX motor;
    
    protected TelescopeImpl() {
        motor = new TalonFX(Ports.Telescope.MOTOR);
        Motors.Telescope.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Constants.Telescope.MIN_HEIGHT_METERS);
    }

    @Override
    public double getCurrentHeight() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getTargetHeight() {
        return getState().getTargetHeight();
    }

    private boolean isWithinTolerance(double heightToleranceMeters) {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < heightToleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.Telescope.HEIGHT_TOLERANCE_METERS);
    }

    @Override
    public void periodic() {

    }


}