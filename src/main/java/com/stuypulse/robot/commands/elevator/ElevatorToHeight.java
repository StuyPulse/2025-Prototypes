package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.subsystems.Elevator.ElevatorImpl;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElevatorToHeight extends InstantCommand {

    private final ElevatorImpl elevator;
    private double targetHeight;

    public ElevatorToHeight(double targetHeight) {
        elevator = ElevatorImpl.getInstance();
        this.targetHeight = targetHeight;
    }

    @Override
    public void initialize() {
        elevator.setTargetHeight(targetHeight);
    }
    

}
