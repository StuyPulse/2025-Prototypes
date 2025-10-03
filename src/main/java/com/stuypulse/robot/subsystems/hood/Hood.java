package com.stuypulse.robot.subsystems.hood;

import org.dyn4j.geometry.Rotation;

import com.stuypulse.robot.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    public static final Hood instance;

    static {
        if (Robot.isReal()){
            instance = new HoodImpl();
        } else {
            instance = new HoodSim();
        }
    }

    public static Hood getInstance(){
        return instance;
    }

    public enum HoodState{
        STOW(new Rotation2d(0), 0.0);

        private Rotation2d rotation;
        private double velocity;
        
        private HoodState(Rotation2d rotation, double velocity) {
            this.rotation = rotation;
            this.velocity = velocity;
        }

        public Rotation2d getRotation(){
            return this.rotation;
        }

        public double getVelocity() {
            return this.velocity;
        }

    }



    public void setTargetAngle(){
        
    }

    public void set

}
