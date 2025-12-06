package com.stuypulse.robot.commands.hdsr;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.ShooterState;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HDSRSetShootDistance extends InstantCommand {
    private final Odometry odometry;
    private final HoodedShooter hdsr; 
    private Supplier<Double> distance;
    private Field2d field;
    private FieldObject2d targetPose;
    private Pose2d targetPose2d;

    public HDSRSetShootDistance(Supplier<Double> distanceMeters) {
        odometry = Odometry.getInstance();
        hdsr = HoodedShooter.getInstance();
        this.field = odometry.getField();
        this.distance = distanceMeters;
        
        addRequirements(hdsr);
    }


        
    @Override 
    public void execute() {
        // double xcord = odometry.getPose().getTranslation().getAngle().getCos()*distance.get();
        // double ycord = odometry.getPose().getTranslation().getAngle().getSin()*distance.get();
        targetPose = field.getObject("HDSR/TargetPose");
        targetPose2d = new Pose2d(new Translation2d(odometry.getPose().getX() + distance.get(), odometry.getPose().getY()), Rotation2d.kZero);
        targetPose.setPose(Robot.isBlue() ? targetPose2d : Field.transformToOppositeAlliance(targetPose2d));

        hdsr.setShooterState(ShooterState.INTERP);
        hdsr.setTargetTranslation(targetPose2d.getTranslation());

        SmartDashboard.putNumber("HDSR/Distance to Target Meters", distance.get());
    }
    
}
