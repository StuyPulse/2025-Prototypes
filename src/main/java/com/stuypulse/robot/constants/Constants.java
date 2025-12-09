package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {
    public interface Swerve {
        double WIDTH = Units.inchesToMeters(18.75);
        double LENGTH = Units.inchesToMeters(18.75);

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 5.36;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }

        public interface FrontLeft {
            boolean inverted = true;
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            boolean inverted = true;
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * +0.5);
        }

        public interface BackRight {
            boolean inverted = false;
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * -0.5);

        }

        public interface FrontRight {
            boolean inverted = false;
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * -0.5);
        }
    }

    public interface HDSR {
        public interface Shooter {
            public static final double SHOOTER_DIAMETER = Units.inchesToMeters(5.0); // PLACEHOLDER
            public static final double SHOOTER_CIRCUMFERENCE_METERS = SHOOTER_DIAMETER * Math.PI; 
            
        }
    
        
    }

    public enum Cameras{
        Limelight("Limelight", new Pose3d(Units.inchesToMeters(-14.412) , Units.inchesToMeters(4.8399), Units.inchesToMeters(4.359), new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(75), Units.degreesToRadians(180))));

        private Pose3d location;
        private String name;

        private Cameras(String name, Pose3d location) {
            this.name = name;
            this.location = location;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }
    }

    public enum Tags{
        GoalTag(new Pose3d(), 4);

        private Pose3d pose;
        private int id;

        private Tags(Pose3d pose, int id) {
            this.pose = pose;
            this.id = id;
        }

        public Pose3d getpose() {
            return pose;
        }

        public int getID() {
            return id;
        }
        
    }
}
