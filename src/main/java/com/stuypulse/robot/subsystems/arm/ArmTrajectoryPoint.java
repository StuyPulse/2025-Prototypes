package com.stuypulse.robot.subsystems.arm;

public class ArmTrajectoryPoint {
    public final double theta1;
    public final double theta2;
    public final double omega1;
    public final double omega2;
    public final double alpha1;
    public final double alpha2;

    public ArmTrajectoryPoint(double theta1, double theta2, 
                            double omega1, double omega2,
                            double alpha1, double alpha2) {
        this.theta1 = theta1;
        this.theta2 = theta2;
        this.omega1 = omega1;
        this.omega2 = omega2;
        this.alpha1 = alpha1;
        this.alpha2 = alpha2;
    }
}