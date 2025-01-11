package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.Arm;
import com.stuypulse.robot.subsystems.Arm.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.stuypulse.robot.subsystems.Arm;
import com.stuypulse.robot.constants.Settings;

public class MoveToState extends InstantCommand {
    public static Command untilDone(State state) {
        return untilDone(state, Settings.Arm.MAX_ANGLE_ERROR);
    }

    public static Command untilDone(State state, double epsilon) {
        return new MoveToState(state)
            .andThen(new WaitUntilCommand(() -> Arm.getInstance().isAtTargetState(epsilon)));
    }

    private final Arm arm; 
    private final State state;

    public MoveToState(State state) {
        arm = Arm.getInstance();
        this.state = state;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setState(state);
    }
}