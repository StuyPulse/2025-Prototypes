package com.stuypulse.robot.subsystems.hooded.turret;

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

public class TurretSim extends Turret {
    private final LinearSystemSim<N2, N1, N2> turretSim;
    private final AngleController controller;

    protected TurretSim(){
        turretSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(0.25, 0.01));
        controller = new AnglePIDController(0.5, 0.0, 0.0);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turretSim.getOutput(0));
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.update(
            Angle.fromRotation2d(getTargetAngle()),
            Angle.fromRotation2d(getAngle()));

        turretSim.setInput(controller.getOutput());

    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turretSim.getInput(0)
        ));
        turretSim.update(Settings.DT);
    }
}
