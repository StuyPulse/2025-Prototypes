package com.stuypulse.robot.subsystems.CLIMBER;

import com.stuypulse.robot.subsystems.wiffleWorksShooter.WiffleWorksShooterImpl;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climber extends SubsystemBase{

    private static Climber instance;
    static {
        instance = new ClimberImpl();
    }

    private ClimberState state;

    public Climber() {
        //instance  = new ClimberImpl();
        state = ClimberState.STOW;
    }

    public static Climber getInstance() {
        return instance;
    }

    public enum ClimberState {
        STOW(0.0),
        CLIMB(3.0),
        DOWN(-3.0);

        private double targetVolts;

        ClimberState(double targetVolts) {
            this.targetVolts = targetVolts;
        }

        public double getClimberVolts() {
            return targetVolts;
        }

        public void setClimberCycle(double ClimberCycle) {
            ClimberCycle = this.targetVolts;
        }
    }

    public ClimberState getClimberState() {
        return state;
    }
    
    public void setClimberState(ClimberState state) {
        this.state = state;  
    }

    @Override
    public void periodic() {

    }

}
