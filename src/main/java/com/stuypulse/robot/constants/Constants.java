package com.stuypulse.robot.constants;

public interface Constants {
    public interface Turret {
        public interface Encoders {
            double GEAR_RATIO = 112.0 / 15.0;

            double OFFSET = 0.0;
        }

        double MAX_ACCEL = 0.0;
        double MAX_VEL = 0.0;
    }

    public interface HoodedShooter {
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;

        double DEFAULT_MAX_ANGULAR_VELOCITY = 0.0;
        double DEFAULT_MAX_ANGULAR_ACCELERATION = 0.0;

        int SMART_CURRENT_LIMIT = 60;

        public interface Encoders {
            double GEAR_RATIO = 0;

            double OFFSET = 0;
        }
    }

}