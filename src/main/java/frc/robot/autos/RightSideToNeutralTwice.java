// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
  public static final Field2d field = new Field2d();
  Pose2d startingPose;
  
  
  public RightSideToNeutralTwice(CommandSwerveDrivetrain S_Swerve, List<AutoWaypoint[]> waypoints, Pose2d startingPose) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    this.startingPose = startingPose;
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        this.startingPose = AutoPaths.rotateBlueToRed(startingPose, Constants.FIELD_LENGTH_METERS, Constants.FIELD_WIDTH_METERS);
        waypoints = waypoints.stream()
          .map(segment -> AutoPaths.rotateBlueToRed(segment, Constants.FIELD_LENGTH_METERS, Constants.FIELD_WIDTH_METERS))
          .toList();       
      }
    }
    this.S_Swerve = S_Swerve;
    this.S_Swerve.resetPose(startingPose);
    SmartDashboard.putNumber("Auto starting pose y value", this.startingPose.getX());
    addRequirements(S_Swerve);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new InstantCommand(() -> S_Swerve.resetPose(this.startingPose)),
      new DriveOverBump(S_Swerve,0),
      new AutoDrive(S_Swerve, waypoints.get(0)),
      new DriveOverBump(S_Swerve, 1),
      new AutoAim(S_Swerve).withTimeout(3.5),
      new DriveOverBump(S_Swerve, 0),
      new AutoDrive(S_Swerve, waypoints.get(1)),
      new DriveOverBump(S_Swerve, 1),
      new AutoAim(S_Swerve).withTimeout(3.5)
      //S_Swerve.pathOnTheFly(waypoints, rotationTargets, constraints)
      //new DriveOverBump(S_Swerve),
      //S_Swerve.findPath(path1, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI))
      //S_Swerve.getAutonomousCommand(),
      //new AutoDrive(S_Swerve)


    );
  }
  
  
}
