package com.stuypulse.robot.subsystems.differentialWrist;

import java.util.Optional;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class DifferentialWristSim extends DifferentialWrist {

    private final LinearSystemSim<N2, N1, N2> leftSim;
    private final LinearSystemSim<N2, N1, N2> rightSim;

    private final AnglePIDController leftController;
    private final AnglePIDController rightController;

    protected DifferentialWristSim() {
        leftSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(3, 0.1));
        rightSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(3, 0.1));
        
        leftController = new AnglePIDController(0, 0, 0);
        rightController = new AnglePIDController(0, 0, 0);
    }

    public Rotation2d getLeftCurrentAngle() {
        return Rotation2d.fromRotations(leftSim.getOutput(0));
    }
    
    public Rotation2d getRightCurrentAngle() {
        return Rotation2d.fromRotations(rightSim.getOutput(0));
    }

    @Override
    public Rotation2d getCurrentRollAngle() {
        return Rotation2d.fromRotations((getLeftCurrentAngle().getRotations() - getRightCurrentAngle().getRotations()) / 2);
    }

    @Override
    public Rotation2d getCurrentPitchAngle() {
        return Rotation2d.fromRotations((getLeftCurrentAngle().getRotations() + getRightCurrentAngle().getRotations()) / 2);
    }

    public void simulationPeriodic() {
        super.periodic();
        
        leftController.update(
            Angle.fromRotation2d(getLeftTargetAngle()),
            Angle.fromRotation2d(getLeftCurrentAngle())
        );

        rightController.update(
            Angle.fromRotation2d(getRightTargetAngle()),
            Angle.fromRotation2d(getRightCurrentAngle())
        );



    }
}
