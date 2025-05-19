package com.stuypulse.robot.subsystems.superStructure;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm;
import com.stuypulse.robot.subsystems.superStructure.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.superStructure.arm.ArmImpl;
import com.stuypulse.robot.subsystems.superStructure.telescope.TelescopeImpl;
import com.stuypulse.robot.subsystems.superStructure.telescope.Telescope;
import com.stuypulse.robot.subsystems.superStructure.telescope.Telescope.TelescopeState;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
    private static final SuperStructure instance;

    static {
        instance = new SuperStructure();
    }

    public static SuperStructure getInstance() {
        return instance;
    }

    public enum SuperStructureState {
        S0(TelescopeState.S0, ArmState.S0), //default position; at_min_height && at_min_angle
        L1(TelescopeState.L1, ArmState.L1),
        L2_FRONT(TelescopeState.L2_FRONT, ArmState.L2_FRONT),
        L2_BACK(TelescopeState.L2_BACK, ArmState.L2_BACK),
        L3_FRONT(TelescopeState.L3_FRONT, ArmState.L3_FRONT),
        L3_BACK(TelescopeState.L3_BACK, ArmState.L3_BACK),
        L4_FRONT(TelescopeState.L4_FRONT, ArmState.L4_FRONT),
        L4_BACK(TelescopeState.L3_BACK, ArmState.L4_BACK);

        private TelescopeState teleState;
        private ArmState armState;

        private SuperStructureState(TelescopeState teleState, ArmState armState) {
            this.teleState = teleState;
            this.armState = armState;
        }

        public TelescopeState getTelescopeState() {
            return this.teleState;
        }

        public ArmState getArmState() {
            return this.armState;
        }
    }

    private SuperStructureState state;

    private final Arm arm;
    private final Telescope telescope;

    private SuperStructure() {
        this.arm = Arm.getInstance();
        this.telescope = Telescope.getInstance();
    }

    public void setState(SuperStructureState state) {
        this.state = state;
        arm.setState(state.getArmState());
        telescope.setState(state.getTelescopeState());
    }
   
    public SuperStructureState getState() {
        return this.state;
    }

    private double getCurrentKg() {
        return Settings.Telescope.kG_min * Math.cos(ArmImpl.getInstance().getCurrentAngle().getDegrees());
    }

    public void periodic() {
        
    }
}
