package com.stuypulse.robot.subsystems.hooded;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class TurretSim extends Turret {
    private final LinearSystemSim<N2, N1, N2> turretSim;
    private final AngleController controller;

    protected TurretSim(){
        turretSim = new LinearSystemSim<N2,N1,N2>(LinearSystemId.identifyPositionSystem(0.25, 0.01));
        controller = new AnglePIDController(1.0, 0.0, 0.0);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turretSim.getOutput(0));
    }

    @Override
    public boolean atTargetAngle() {
        return getAngle().getDegrees() == getTargetAngle().getDegrees();
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.update(
            Angle.fromRotation2d(getTargetAngle()),
            Angle.fromRotation2d(getAngle()));

        if (controller.getOutput() < 0.1)
            turretSim.setInput(0);
        else
            turretSim.setInput(MathUtil.clamp(controller.getOutput(), 12, -12));

    }

    @Override
    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            turretSim.getInput(0)
        ));

        turretSim.update(Settings.DT);
    }
}
