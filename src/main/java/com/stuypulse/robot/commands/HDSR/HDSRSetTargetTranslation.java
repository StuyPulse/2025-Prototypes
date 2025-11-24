package com.stuypulse.robot.commands.HDSR;

import com.stuypulse.robot.subsystems.wiffleWorksShooter.WiffleWorksShooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HDSRSetTargetTranslation extends InstantCommand {
    private final WiffleWorksShooter wiffleWorksShooter;
    private final Translation2d targettranslation;

    public HDSRSetTargetTranslation(Translation2d targetTranslation) {
        wiffleWorksShooter = WiffleWorksShooter.getInstance();
        this.targettranslation = targetTranslation;

        addRequirements(wiffleWorksShooter);
    }

    @Override 
    public void initialize () {
        wiffleWorksShooter.setTargetTranslation(targettranslation);
    }
}
