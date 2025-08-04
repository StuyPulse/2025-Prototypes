// package com.stuypulse.robot.subsystems.arm;

// import com.stuypulse.robot.constants.Constants;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//fix
// public class ArmSim extends Arm {
//     // Physics simulations
//     private final SingleJointedArmSim shoulderSim;
//     private final SingleJointedArmSim elbowSim;
    
//     private double prevShoulderPos = 0;
//     private double prevElbowPos = 0;

//     public ArmSim() {
//         // Shoulder joint (with gravity)
//         shoulderSim = new SingleJointedArmSim(

//         );

//         elbowSim = new SingleJointedArmSim(

//         );
//     }

//     @Override
//     public void periodic() {
//         simulationPeriodic();
        
//         // Logging
//         SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngleDegrees());
//         SmartDashboard.putNumber("Arm/Elbow Angle", getElbowAngleDegrees());
//         SmartDashboard.putNumber("Arm/End Height", getEndPosition().getY());
//     }

//     @Override
//     public void simulationPeriodic() {
//         shoulderSim.setInputVoltage(shoulderVolts);
//         elbowSim.setInputVoltage(elbowVolts);
//         shoulderSim.update(0.020);
//         elbowSim.update(0.020);
//     }

//     @Override
//     public double getShoulderAngleDegrees() {
//         return Units.radiansToDegrees(shoulderSim.getAngleRads());
//     }

//     @Override
//     public double getElbowAngleDegrees() {
//         return Units.radiansToDegrees(elbowSim.getAngleRads());
//     }

//     @Override
//     public Translation2d getEndPosition() {
//         double shoulderRad = Units.degreesToRadians(getShoulderAngleDegrees());
//         double elbowRad = Units.degreesToRadians(getElbowAngleDegrees());
        
//         return new Translation2d(
//             SHOULDER_LENGTH * Math.cos(shoulderRad) + ELBOW_LENGTH * Math.cos(shoulderRad + elbowRad),
//             SHOULDER_LENGTH * Math.sin(shoulderRad) + ELBOW_LENGTH * Math.sin(shoulderRad + elbowRad)
//         );
//     }


//     @Override
//     public void setTargetAngles(double shoulderDeg, double elbowDeg) {
//         // Apply voltages through the control loop
//         double shoulderRad = Units.degreesToRadians(shoulderDeg);
//         double elbowRad = Units.degreesToRadians(elbowDeg);
        
//         shoulderSim.setState(shoulderRad, 0);
//         elbowSim.setState(elbowRad, 0);
//     }
// }