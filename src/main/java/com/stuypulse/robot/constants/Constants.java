package com.stuypulse.robot.constants;

public interface Constants {
    public interface Turret {
        public interface Encoders {
            double GEAR_RATIO = 112.0 / 15.0;
    
            double NUM_ROTATIONS_TO_REACH_TOP = 0.0; // Number of rotations that the motor has to spin, NOT the gear
            double POSITION_CONVERSION_FACTOR = 0.0 / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = 0.0 / NUM_ROTATIONS_TO_REACH_TOP / 60.0;

            double OFFSET = 0.0;
        }

        double MAX_ACCEL = 0.0;
        double MAX_VEL = 0.0;
    }

}