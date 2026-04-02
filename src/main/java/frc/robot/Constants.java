package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static final Pose2d BLUE_HUB_POSITION = new Pose2d(4.675, 8.07/2, new Rotation2d());
    public static final Pose2d RED_HUB_POSITION = new Pose2d(4.675 + 7.19, 8.07/2, new Rotation2d());
    public static final double FIELD_WIDTH_METERS = 8.069;
    public static final double FIELD_LENGTH_METERS = 16.54;

    public static final double RightSideRotateToSeeTagsTarget = 135;
    public static final double LeftSideRotateToSeeTagsTarget = -135;
    public static class ClimberConstants {
        public static final double extendPosition = 0;
        public static final double retractPosition = 40.0;
    }
    public static class IntakeConstants {
        public static final double deployPosition = 67.4;
        public static final double retractPosition = 0;
        public static final double IntakeRPS = 90;
    }
    public static class ShooterConstants {
        public static final double hoodRetractPosition = 0;
        public static final double hoodStage1Position = 10.5;
        public static final double hoodStage2Position = 18.0;
        public static final double idleRPS = 45;
        public static final double feederShootRPS = 95;
        public static final double trenchShotRPS = 47.2;
        public static final double towerShotRPS = 45.5;
        public static final double midwayShotRPS = 44.5;

    }
    public static class HopperConstants {
        public static final double floorShootSpeed = 112;
    }
    public static class SwerveConstants {
        public static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static double MaxAngularRate = RotationsPerSecond.of(2.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static double RightCLimbLidar = 66.8;
        public static double LeftCLimbLidar = 58.8;
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
