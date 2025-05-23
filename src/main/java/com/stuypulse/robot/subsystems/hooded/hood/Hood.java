package com.stuypulse.robot.subsystems.hooded.hood;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.hooded.turret.Turret.TurretState;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hood extends SubsystemBase{

    public enum HoodState {
        CLOSE(Settings.HoodedShooter.States.CLOSE_ANGLE),
        FAR(Settings.HoodedShooter.States.FAR_ANGLE),
        FERRY(Settings.HoodedShooter.States.FERRY_ANGLE);

        private final Rotation2d targetAngle;

        private HoodState(Rotation2d angle) {
            targetAngle = angle;
        }

        public Rotation2d getTargetAngle() {
            return targetAngle;
        }
    }

    private static final Hood instance;
    static {
        if (Robot.isReal()){
            instance = new HoodImpl();
        } else {
            instance = new HoodSim();
        }
    }

    public static Hood getInstance() {
        return instance;
    }
    
    private HoodState state;

    protected Hood() {
        state = HoodState.FERRY;
        visualizer = new HoodVisualizer();
    }

    public HoodState getState() {
        return state;
    }

    public void setState(HoodState hoodState) {
        state = hoodState;
    }

    public abstract Rotation2d getAngle();
    public abstract boolean atTargetAngle();
    private HoodVisualizer visualizer;

    @Override
    public void periodic() {
        SmartDashboard.putString("Hooded Shooter/Hood State", getState().toString());
        SmartDashboard.putNumber("Hooded Shooter/Hood Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Hooded Shooter/Hood Target Angle", getState().getTargetAngle().getDegrees());
    
        visualizer.updateAngle(getAngle());
    }
}
