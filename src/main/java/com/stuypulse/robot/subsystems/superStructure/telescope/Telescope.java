package com.stuypulse.robot.subsystems.superStructure.telescope;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Telescope extends SubsystemBase {

    private static final Telescope instance;

    static {
        if (Robot.isReal()) {
            instance = new TelescopeImpl();
        }
    }

    public static Telescope getInstance() {
        return instance;
    }

    public enum TelescopeState{
        S0(Settings.Telescope.MIN_HEIGHT_METERS), //stage 0; when fully retracted, stops with physical hardstop 
        L1(Settings.Telescope.L1_HEIGHT_METERS),
        L2_FRONT(Settings.Telescope.L2_FRONT_HEIGHT_METERS),
        L3_FRONT(Settings.Telescope.L3_FRONT_HEIGHT_METERS),
        L4_FRONT(Settings.Telescope.L4_FRONT_HEIGHT_METERS),
        L2_BACK(Settings.Telescope.L2_BACK_HEIGHT_METERS),
        L3_BACK(Settings.Telescope.L3_BACK_HEIGHT_METERS),
        L4_BACK(Settings.Telescope.L4_BACK_HEIGHT_METERS);
    
        private Number targetHeight;

        private TelescopeState(Number targetHeight) {
            this.targetHeight = targetHeight;
    
        }
    
        public double getTargetHeight() {
            return targetHeight.doubleValue();
        }
    }

    private TelescopeState state;

    public void setState(TelescopeState state) {
        this.state = state;
    }

    public TelescopeState getState() {
        return state;
    }

    public static TelescopeState getState(int level, boolean isFacingFrontReef) {
        switch (level) {
            case 1: 
                return TelescopeState.L1;
            case 2:
                return isFacingFrontReef ? TelescopeState.L2_FRONT : TelescopeState.L2_BACK;
            case 3:
                return isFacingFrontReef ? TelescopeState.L3_FRONT : TelescopeState.L3_BACK;
            case 4:
                return isFacingFrontReef ? TelescopeState.L4_FRONT : TelescopeState.L4_BACK;
            default: 
                return TelescopeState.S0;
        }
    }

    public abstract double getCurrentHeight();
    public abstract boolean atTargetHeight();

    
    @Override
    public void periodic() {
        // add SmartDashboard stuff
        SmartDashboard.putString("Telescope State", getState().toString());
        SmartDashboard.putBoolean("Telescope/At Target Height", atTargetHeight());
        SmartDashboard.putNumber("Telescope/Current Height", getCurrentHeight());
        SmartDashboard.putNumber("Telescope/Target Height", getState().getTargetHeight());

    }

}