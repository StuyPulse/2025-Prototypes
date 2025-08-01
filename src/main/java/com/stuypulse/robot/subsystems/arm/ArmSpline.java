package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.spline.QuinticHermiteSpline;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

public class ArmSpline {
    private final QuinticHermiteSpline spline;
    private final double duration;

    public static class ArmTrajectoryPoint {
        public final double theta1;    // Shoulder angle (rad)
        public final double theta2;    // Elbow angle (rad)
        public final double omega1;    // Shoulder velocity (rad/s)
        public final double omega2;    // Elbow velocity (rad/s)
        public final double alpha1;    // Shoulder acceleration (rad/s^2)
        public final double alpha2;    // Elbow acceleration (rad/s^2)
        public final double time;      // Time since start (s)

        public ArmTrajectoryPoint(double theta1, double theta2,
                                double omega1, double omega2,
                                double alpha1, double alpha2,
                                double time) {
            this.theta1 = theta1;
            this.theta2 = theta2;
            this.omega1 = omega1;
            this.omega2 = omega2;
            this.alpha1 = alpha1;
            this.alpha2 = alpha2;
            this.time = time;
        }
    }

    public ArmSpline(double[] startTheta, double[] startVel, double[] startAccel,
                    double[] endTheta, double[] endVel, double[] endAccel,
                    double durationSec) {
        this.duration = durationSec;

        this.spline = new QuinticHermiteSpline(
            // X control vectors (theta1)
            new double[]{startTheta[0], startVel[0] * duration, startAccel[0] * duration * duration},
            new double[]{endTheta[0], endVel[0] * duration, endAccel[0] * duration * duration},
            
            // Y control vectors (theta2)
            new double[]{startTheta[1], startVel[1] * duration, startAccel[1] * duration * duration},
            new double[]{endTheta[1], endVel[1] * duration, endAccel[1] * duration * duration}
        );
    }

    public ArmTrajectoryPoint getPoint(double t) {
        // Get coefficients for both dimensions
        double[] coeffsX = Stream.of(spline.getCoefficients().toArray2()).flatMapToDouble(DoubleStream::of).toArray(); //not x coeff needs fixing
        double[] coeffsY = Stream.of(spline.getCoefficients().toArray2()).flatMapToDouble(DoubleStream::of).toArray(); //not y coeff needs fixing

        // Calculate position (rad)
        double theta1 = calculatePolynomial(coeffsX, t);
        double theta2 = calculatePolynomial(coeffsY, t);
        
        // Calculate velocity (rad/s)
        double omega1 = calculatePolynomialDerivative(coeffsX, t) / duration;
        double omega2 = calculatePolynomialDerivative(coeffsY, t) / duration;
        
        // Calculate acceleration (rad/s²)
        double alpha1 = calculatePolynomialSecondDerivative(coeffsX, t) / (duration * duration);
        double alpha2 = calculatePolynomialSecondDerivative(coeffsY, t) / (duration * duration);
        
        return new ArmTrajectoryPoint(
            theta1, theta2,
            omega1, omega2,
            alpha1, alpha2,
            t * duration
        );
    }

    // Optimize with Math.pow() later
    private double calculatePolynomial(double[] coeffs, double t) {
        return coeffs[0] + coeffs[1] * t + coeffs[2] * t * t 
             + coeffs[3] * t * t * t + coeffs[4] * t * t * t * t 
             + coeffs[5] * t * t * t * t * t;
    }

    private double calculatePolynomialDerivative(double[] coeffs, double t) {
        return coeffs[1] + 2 * coeffs[2] * t 
             + 3 * coeffs[3] * t * t + 4 * coeffs[4] * t * t * t 
             + 5 * coeffs[5] * t * t * t * t;
    }

    private double calculatePolynomialSecondDerivative(double[] coeffs, double t) {
        return 2 * coeffs[2] + 6 * coeffs[3] * t 
             + 12 * coeffs[4] * t * t + 20 * coeffs[5] * t * t * t;
    }

    public List<ArmTrajectoryPoint> sampleTrajectory(int sampleCount) {
        List<ArmTrajectoryPoint> points = new ArrayList<>();
        for (int i = 0; i <= sampleCount; i++) {
            double t = i / (double) sampleCount;
            points.add(getPoint(t));
        }
        return points;
    }
}