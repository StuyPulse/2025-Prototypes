package com.stuypulse.robot.commands.hooded.hood;
import com.stuypulse.robot.subsystems.hooded.Hood;
import com.stuypulse.robot.subsystems.hooded.Hood.HoodState;

import edu.wpi.first.wpilibj2.command.Command;

public class HoodToState extends Command {

    private final Hood hood;
    private final HoodState state;

    public HoodToState(HoodState state) {
        hood = Hood.getInstance();
        this.state = state;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setState(state);
    }

}