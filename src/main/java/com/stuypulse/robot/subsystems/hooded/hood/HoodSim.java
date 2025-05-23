package com.stuypulse.robot.subsystems.hooded.hood;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class HoodSim extends Hood {
    private final LinearSystemSim<N2, N1, N2> hoodSim;
    private final AngleController controller;

    protected HoodSim() {
        hoodSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(0.25, 0.01));
        controller = new AnglePIDController(0.5, 0.0, 0.0);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(hoodSim.getOutput(0));
    }

    // TODO Same method move to abstract
    @Override
    public boolean atTargetAngle() {
        return Math.abs(getAngle().getDegrees() - getState().getTargetAngle().getDegrees()) < Settings.HoodedShooter.TOLERANCE;
    }
    
    @Override
    public void periodic() {
        super.periodic();

        controller.update(
            Angle.fromRotation2d(getState().getTargetAngle()),
            Angle.fromRotation2d(getAngle()));
        
        hoodSim.setInput(controller.getOutput());
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            hoodSim.getInput(0)
        ));
        hoodSim.update(Settings.DT);
    }
}
