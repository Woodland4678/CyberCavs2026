// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoWaypoint;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveOverBump;
import frc.robot.subsystems.CommandSwerveDrivetrain;

      
      

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightSideToNeutralTwice extends SequentialCommandGroup {
  /** Creates a new RightSideToNeutralTwice. */
  CommandSwerveDrivetrain S_Swerve;
  AutoWaypoint[] waypoints = {
    new AutoWaypoint(new Pose2d(6.715, 2.788, new Rotation2d().fromDegrees(154)), 1.5, 2.0, Math.PI * 3, 0.15, 12),
    new AutoWaypoint(new Pose2d(7.93, 2.075, new Rotation2d().fromDegrees(154)), 1.0, 1.5, Math.PI * 3, 0.15, 12),
    new AutoWaypoint(new Pose2d(5.873, 2.399, new Rotation2d().fromDegrees(45)), 2, 3.0, Math.PI * 3, 0.15, 6)
  };
  public RightSideToNeutralTwice(CommandSwerveDrivetrain S_Swerve) {
    this.S_Swerve = S_Swerve;
    addRequirements(S_Swerve);
    //var currentPose = S_Swerve.getState().Pose;
    // PathConstraints constraints = new PathConstraints(2.0, 2.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //     new Pose2d(currentPose.getX() + 1, currentPose.getY(), Rotation2d.fromDegrees(0)),
    //     new Pose2d(currentPose.getX() + 2, currentPose.getY() + 1, Rotation2d.fromDegrees(45)),
    //     //new Pose2d(currentPose.getX(), currentPose.getY() + 1, Rotation2d.fromDegrees(0)),
    //     new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromDegrees(0))
    // );
    // List<RotationTarget> rotationTargets = List.of(
    //   new RotationTarget(0.5, Rotation2d.fromDegrees(-45))
    // );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveOverBump(S_Swerve,0),
      new AutoDrive(S_Swerve, waypoints),
      new DriveOverBump(S_Swerve, 1),
      new AutoAim(S_Swerve)
      //S_Swerve.pathOnTheFly(waypoints, rotationTargets, constraints)
      //new DriveOverBump(S_Swerve),
      //S_Swerve.findPath(path1, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI))
      //S_Swerve.getAutonomousCommand(),
      //new AutoDrive(S_Swerve)


    );
  }
}
