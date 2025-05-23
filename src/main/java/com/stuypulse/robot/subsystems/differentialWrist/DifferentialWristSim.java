package com.stuypulse.robot.subsystems.differentialWrist;

import java.util.Optional;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;


public class DifferentialWristSim extends DifferentialWrist {

    private final LinearSystemSim<N2, N1, N2> leftSim;
    private final LinearSystemSim<N2, N1, N2> rightSim;

    private final AnglePIDController leftController;
    private final AnglePIDController rightController;

    protected DifferentialWristSim() {
        leftSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(3, 0.1));
        rightSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(3, 0.1));
        
        leftController = new AnglePIDController(0.5, 0, 0.1);
        rightController = new AnglePIDController(0.5, 0, 0.1);
    }

    @Override
    public Rotation2d getLeftCurrentAngle() {
        return Rotation2d.fromRotations(leftSim.getOutput(0));
    }
    
    @Override
    public Rotation2d getRightCurrentAngle() {
        return Rotation2d.fromRotations(rightSim.getOutput(0));
    }

    

    @Override
    public void periodic() {
        super.periodic();
        
        leftController.update(
            Angle.fromRotations(getLeftTargetAngle().getRotations()),
            Angle.fromRotations(getLeftCurrentAngle().getRotations())
        );

        rightController.update(
            Angle.fromRotations(getRightTargetAngle().getRotations()),
            Angle.fromRotations(getRightCurrentAngle().getRotations())
        );

        leftSim.setInput(leftController.getOutput());
        
        
        rightSim.setInput(rightController.getOutput());

    }

    @Override
    public void simulationPeriodic(){
        super.simulationPeriodic();

        leftSim.update(Settings.DT);
        rightSim.update(Settings.DT);
    }
}
