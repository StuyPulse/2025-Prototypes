package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {
    public interface Shooter {
        public interface PID {
            double kP = 2.0;
            double kI = 0.0;
            double kD = 0.20;
        }

        public interface FF {
            double kS = 0.1;
            double kV = 0.2;
            double kA = 0.01;
            double kG = 0.0;
        }
    }
}