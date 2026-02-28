package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoWaypoint;

public final class AutoPaths {
    public static final double FIELD_WIDTH_METERS = 8.042656;
    public static final List<AutoWaypoint[]> RightSideGatherFuel1 = List.of(
        // First segment
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.715, 2.788, new Rotation2d().fromDegrees(154)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.93, 2.075, new Rotation2d().fromDegrees(154)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(5.873, 2.399, new Rotation2d().fromDegrees(45)), 2, 3.0, Math.PI * 3, 0.15, 6)
        },
        // Second segment
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.391, 1.330, new Rotation2d().fromDegrees(-90)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.914, 0.974, new Rotation2d().fromDegrees(-90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.914, 2.48, new Rotation2d().fromDegrees(-90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(5.873, 2.399, new Rotation2d().fromDegrees(45)), 2, 3.0, Math.PI * 3, 0.15, 6)
        }
    );

    public static final List<AutoWaypoint[]> RightSideGatherFuel2 = List.of(
        // First segment
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.472, 3.729, new Rotation2d().fromDegrees(-151.124)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.749, 4.117, new Rotation2d().fromDegrees(-151.124)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(5.873, 2.399, new Rotation2d().fromDegrees(45)), 2, 3.0, Math.PI * 3, 0.15, 6)
        },
        // Second segment
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.391, 1.330, new Rotation2d().fromDegrees(-90)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.914, 0.974, new Rotation2d().fromDegrees(-90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.914, 2.48, new Rotation2d().fromDegrees(-90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(5.873, 2.399, new Rotation2d().fromDegrees(45)), 2, 3.0, Math.PI * 3, 0.15, 6)
        }
    );

    public static final List<AutoWaypoint[]> DriveToClimberLeftSide = List.of(
        // First segment
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(15.492, 3.14, new Rotation2d().fromDegrees(180)), 0.0, 1.0, Math.PI * 3, 0.01, 4),
            new AutoWaypoint(new Pose2d(15.492, 3.43, new Rotation2d().fromDegrees(180)), 0.0, 1.0, Math.PI * 3, 0.02, 2),
        },
        // Second segment
        new AutoWaypoint[] {
            new AutoWaypoint(new Pose2d(6.391, 1.330, new Rotation2d().fromDegrees(-90)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.914, 0.974, new Rotation2d().fromDegrees(-90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(7.914, 2.48, new Rotation2d().fromDegrees(-90)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
            new AutoWaypoint(new Pose2d(5.873, 2.399, new Rotation2d().fromDegrees(45)), 2, 3.0, Math.PI * 3, 0.15, 6)
        }
    );



    
    public static final List<AutoWaypoint[]> LeftSideGatherFuel1 =
    RightSideGatherFuel1.stream()
        .map(segment -> mirrorWaypointsLeftRight(segment, FIELD_WIDTH_METERS))
        .toList();




    public static AutoWaypoint[] mirrorWaypointsLeftRight(AutoWaypoint[] original, double fieldWidthMeters) {
        AutoWaypoint[] mirrored = new AutoWaypoint[original.length];
        for (int i = 0; i < original.length; i++) {
            AutoWaypoint wp = original[i];
            mirrored[i] = new AutoWaypoint(
                mirrorBlueLeftRight(wp.waypoint, fieldWidthMeters), // mirrored Pose2d
                wp.cruiseSpeed,
                wp.maxSpeed,
                wp.maxAngularSpeed,
                wp.translationTolerance,
                wp.rotationTolerance
            );
        }
        return mirrored;
    }
    public static Pose2d mirrorBlueLeftRight(Pose2d original, double fieldWidthMeters) {
        return new Pose2d(
            original.getX(),
            fieldWidthMeters - original.getY(),
            new Rotation2d(-original.getRotation().getRadians())
        );
    }
    public static AutoWaypoint[] rotateRed(AutoWaypoint[] original, double fieldLengthMeters, double fieldWidthMeters) {
        AutoWaypoint[] mirrored = new AutoWaypoint[original.length];
        for (int i = 0; i < original.length; i++) {
            AutoWaypoint wp = original[i];
            mirrored[i] = new AutoWaypoint(
                rotateRed(wp.waypoint, fieldLengthMeters, fieldWidthMeters), // mirrored Pose2d
                wp.cruiseSpeed,
                wp.maxSpeed,
                wp.maxAngularSpeed,
                wp.translationTolerance,
                wp.rotationTolerance
            );
        }
        return mirrored;
    }
    public static Pose2d rotateRed(Pose2d original, double fieldLengthMeters, double fieldWidthMeters) {
        
        if (original.getRotation().getRadians() > 0) {
            return new Pose2d(
            fieldLengthMeters - original.getX(),
            fieldWidthMeters - original.getY(),
            new Rotation2d(original.getRotation().getRadians() + Math.PI));
        } else {
            return new Pose2d(
            fieldLengthMeters - original.getX(),
            fieldWidthMeters - original.getY(),
            new Rotation2d(original.getRotation().getRadians() - Math.PI));
        }
    }
        public static AutoWaypoint[] rotateRedMirrorLeftRight(AutoWaypoint[] original, double fieldLengthMeters) {
        AutoWaypoint[] mirrored = new AutoWaypoint[original.length];
        for (int i = 0; i < original.length; i++) {
            AutoWaypoint wp = original[i];
            mirrored[i] = new AutoWaypoint(
                rotateRedMirrorLeftRight(wp.waypoint, fieldLengthMeters), // mirrored Pose2d
                wp.cruiseSpeed,
                wp.maxSpeed,
                wp.maxAngularSpeed,
                wp.translationTolerance,
                wp.rotationTolerance
            );
        }
        return mirrored;
    }
    public static Pose2d rotateRedMirrorLeftRight(Pose2d original, double fieldLengthMeters) {
        if (original.getRotation().getRadians() > 0) {
            return new Pose2d(
            fieldLengthMeters - original.getX(),
            original.getY(),
            new Rotation2d(-(original.getRotation().getRadians() + Math.PI)));
        } else {
            return new Pose2d(
            fieldLengthMeters - original.getX(),
            original.getY(),
            new Rotation2d(-(original.getRotation().getRadians() - Math.PI)));
        }
    }
    public static Pose2d[] extractPoses(List<AutoWaypoint[]> waypointSegments) {
        // Count total number of waypoints
        int total = waypointSegments.stream().mapToInt(arr -> arr.length).sum();
        Pose2d[] poses = new Pose2d[total];

        int index = 0;
        for (AutoWaypoint[] segment : waypointSegments) {
            for (AutoWaypoint wp : segment) {
                poses[index++] = wp.waypoint;
            }
        }

        return poses;
    }








    public static AutoWaypoint[] mirrorBlueRightToLeft(AutoWaypoint[] original,
        double fieldWidthMeters
    ) {
        AutoWaypoint[] out = new AutoWaypoint[original.length];

        for (int i = 0; i < original.length; i++) {
            AutoWaypoint wp = original[i];
            out[i] = new AutoWaypoint(
                mirrorBlueRightToLeft(wp.waypoint, fieldWidthMeters),
                wp.cruiseSpeed,
                wp.maxSpeed,
                wp.maxAngularSpeed,
                wp.translationTolerance,
                wp.rotationTolerance
            );
        }
        return out;
    }
    public static Pose2d mirrorBlueRightToLeft(Pose2d original, double fieldWidthMeters) {
        return new Pose2d(
            original.getX(),
            fieldWidthMeters - original.getY(),
            original.getRotation().unaryMinus()
        );
    }
    public static Pose2d rotateBlueToRed(
        Pose2d original,
        double fieldLengthMeters,
        double fieldWidthMeters
    ) {
        return new Pose2d(
            fieldLengthMeters - original.getX(),
            fieldWidthMeters  - original.getY(),
            original.getRotation().plus(Rotation2d.fromRadians(Math.PI))
        );
    }
    public static AutoWaypoint[] rotateBlueToRed(
        AutoWaypoint[] original,
        double fieldLengthMeters,
        double fieldWidthMeters
    ) {
        AutoWaypoint[] out = new AutoWaypoint[original.length];

        for (int i = 0; i < original.length; i++) {
            AutoWaypoint wp = original[i];
            out[i] = new AutoWaypoint(
                rotateBlueToRed(wp.waypoint, fieldLengthMeters, fieldWidthMeters),
                wp.cruiseSpeed,
                wp.maxSpeed,
                wp.maxAngularSpeed,
                wp.translationTolerance,
                wp.rotationTolerance
            );
        }
        return out;
    }

}
