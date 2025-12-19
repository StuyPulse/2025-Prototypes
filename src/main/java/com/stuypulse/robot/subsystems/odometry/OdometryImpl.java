package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.FieldUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OdometryImpl extends Odometry {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDrive swerve;
    private final Pose2d startingPose;
    private final Field2d field;

    private final FieldObject2d poseFieldObject;

    protected OdometryImpl() {
        swerve = SwerveDrive.getInstance();
        startingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        poseEstimator = new SwerveDrivePoseEstimator(
                swerve.getKinematics(),
                Rotation2d.kZero,
                swerve.getModulePositions(),
                startingPose,
                VecBuilder.fill(
                        0.1,
                        0.1,
                        0.1),
                VecBuilder.fill(
                        0.3,
                        0.3,
                        Math.toRadians(30)));

        field = new Field2d();

        poseFieldObject = field.getRobotObject();
        poseFieldObject.setPose(FieldUtil.fieldTransform(new Pose2d()));

        // swerve.initFieldObjects(field);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Pose2d getPose() {
        // return poseEstimator.getEstimatedPosition();
        return new Pose2d(0, 0, Rotation2d.kCW_90deg);
    }

    @Override
    public void reset(Pose2d pose) {
        SwerveDrive drive = SwerveDrive.getInstance();

        poseEstimator.resetPosition(
                drive.getGyroAngle(),
                drive.getModulePositions(),
                pose);
    }

    @Override
    public Field2d getField() {
        return field;
    }

    public void updateVisionMeasurement(Matrix<N3, N1> visionStdDevs, Pose2d pose, double timestampSeconds) {
        poseEstimator.setVisionMeasurementStdDevs(visionStdDevs);
        poseEstimator.addVisionMeasurement(
                pose,
                timestampSeconds);
    }

    @Override
    public void periodic() {
        SwerveDrive drive = SwerveDrive.getInstance();
        poseEstimator.update(drive.getGyroAngle(), drive.getModulePositions());

        // poseFieldObject.setPose(poseEstimator.getEstimatedPosition());
        poseFieldObject.setPose(new Pose2d(0,0, Rotation2d.kCCW_90deg));

        SmartDashboard.putNumber("Odometry/Pose Estimator Pose X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry/Pose Estimator Pose Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometry/Pose Estimator Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }
}