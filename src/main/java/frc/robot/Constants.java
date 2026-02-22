package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    public static class ClimberConstants {
        public static final double extendPosition = -1;
        public static final double retractPosition = 16.0;
    }
    public static class IntakeConstants {
        public static final double deployPosition = -11.38;
        public static final double retractPosition = -0.1;
    }
    public static class ShooterConstants {
        public static final double hoodRetractPosition = 0;
        public static final double hoodStage1Position = -7.81;
        public static final double hoodStage2Position = -13.857;

    }
    public static class HopperConstants {

    }
    
    
    public static class AutoWaypoint{
        public Pose2d waypoint = new Pose2d();
        public double cruiseSpeed;
        public double maxSpeed;
        public double maxAngularSpeed;
        public double translationTolerance;
        public double rotationTolerance;
        public double defaultCruiseSpeed = 3.0;
        public double defaultMaxAngularSpeed = Math.PI * 4;
        public double defaultMaxSpeed = 5.5;
        public double defaultTranslationTolerance = 0.1; //10 cm
        public double defaultRotationTolerance = 10; //10 deegrees
        public AutoWaypoint(Pose2d waypoint, double cruiseSpeed, double maxSpeed, double maxAngularSpeed, double translationTolerance, double rotationTolerance) {
            this.waypoint = waypoint;
            this.cruiseSpeed = cruiseSpeed;
            this.maxSpeed = maxSpeed;
            this.maxAngularSpeed = maxAngularSpeed;
            this.translationTolerance = translationTolerance;
            this.rotationTolerance = rotationTolerance;
        }
        public AutoWaypoint(Pose2d waypoint) {
            this.waypoint = waypoint;
            this.cruiseSpeed = defaultCruiseSpeed;
            this.maxSpeed = defaultMaxSpeed;
            this.maxAngularSpeed = defaultMaxAngularSpeed;
            this.translationTolerance = defaultTranslationTolerance;
            this.rotationTolerance = defaultRotationTolerance;
        }
        public AutoWaypoint(Pose2d waypoint, double cruiseSpeed, double maxSpeed) {
            this.waypoint = waypoint;
            this.cruiseSpeed = cruiseSpeed;
            this.maxSpeed = maxSpeed;
             this.maxAngularSpeed = defaultMaxAngularSpeed;
            this.translationTolerance = defaultTranslationTolerance;
            this.rotationTolerance = defaultRotationTolerance;
        }
      }
    

}
