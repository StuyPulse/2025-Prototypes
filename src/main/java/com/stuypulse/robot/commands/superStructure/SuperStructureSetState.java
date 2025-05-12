package com.stuypulse.robot.commands.superStructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

public class SuperStructureSetState extends InstantCommand {
    private final SuperStructure superStructure;
    private final SuperStructureState superStructureState;

    
    public SuperStructureSetState(SuperStructureState state) {
        this.superStructure = SuperStructure.getInstance();
        this.superStructureState = state;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(superStructureState);
    }

}
