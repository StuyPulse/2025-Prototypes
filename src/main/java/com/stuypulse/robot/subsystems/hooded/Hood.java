package com.stuypulse.robot.subsystems.hooded;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hood extends SubsystemBase{
    public enum HoodState {
        CLOSE(Settings.HoodedShooter.States.CLOSE_ANGLE),
        FAR(Settings.HoodedShooter.States.FAR_ANGLE),
        FERRY(Settings.HoodedShooter.States.FERRY_ANGLE);

        private final double targetAngle;

        private HoodState(double angle){
            targetAngle = angle;
        }

        public double getTargetAngle(){
            return targetAngle;
        }
    }

    private static final Hood instance;
    static {
        // TODO: change ts later
        instance = new HoodImpl();
    }

    public static Hood getInstance(){
        return instance;
    }
    
    private HoodState state;

    protected Hood() {
        state = HoodState.FERRY;
    }

    public HoodState getState(){
        return state;
    }

    public void setState(HoodState hoodState){
        state = hoodState;
    }

    public abstract double getAngle();
    public abstract boolean atTargetAngle();

    @Override
    public void periodic() {
        SmartDashboard.putString("Hooded Shooter/Hood State", getState().toString());
        SmartDashboard.putNumber("Hooded Shooter/Hood Angle", getAngle());
        SmartDashboard.putNumber("Hooded Shooter/Hood Target Angle", getState().getTargetAngle());
    }
}
