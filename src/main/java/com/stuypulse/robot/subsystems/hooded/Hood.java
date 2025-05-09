package com.stuypulse.robot.subsystems.hooded;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hood extends SubsystemBase{
    public enum HoodState {
        CLOSE(0),
        FAR(0),
        FERRY(0);

        private final double targetAngle;

        private HoodState(double angle){
            targetAngle = angle;
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
    

    private HoodState state = HoodState.FERRY;

    public HoodState getState(){
        return state;
    }

    public double getTargetAngle() {
        return state.targetAngle;
    }
}
