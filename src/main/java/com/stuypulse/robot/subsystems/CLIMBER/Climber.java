package com.stuypulse.robot.subsystems.CLIMBER;

import com.stuypulse.robot.subsystems.wiffleWorksShooter.WiffleWorksShooterImpl;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climber extends SubsystemBase{

    private static Climber instance;
    static {
        instance = new ClimberImpl();
    }

    private static ClimberState state;

    public Climber() {
        //instance  = new ClimberImpl();
        state = ClimberState.STOW;
    }

    public static Climber getInstance() {
        return instance;
    }

    public enum ClimberState {
        STOW(0.0),
        CLIMB(1.0),
        DOWN(-1.0);

        private double ClimberCycle;

        ClimberState(double ClimberCycle) {
            this.ClimberCycle = ClimberCycle;
        }

        public double getClimberCycle() {
            return ClimberCycle;
        }

        public void setClimberCycle(double ClimberCycle) {
            ClimberCycle = this.ClimberCycle;
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
