package com.stuypulse.robot.commands.HDSR;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Translation2d;

public class HDSRShootXMeters extends HDSRSetTargetTranslation{
    public HDSRShootXMeters(double distance) {
        super(new Translation2d(Odometry.getInstance().getTranslation().getX() + distance, Odometry.getInstance().getTranslation().getY()));
    }
}
