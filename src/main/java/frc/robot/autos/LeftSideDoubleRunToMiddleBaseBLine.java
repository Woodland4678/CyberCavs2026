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
import frc.robot.commands.RotateToAngleUntilTagsSeen;
import frc.robot.commands.Shoot;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

      
      

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftSideDoubleRunToMiddleBaseBLine extends SequentialCommandGroup {
  /** Creates a new RightSideToNeutralTwice. */
  CommandSwerveDrivetrain S_Swerve;
  Hopper S_Hopper;
  Intake S_Intake;
  Shooter S_Shooter;
  
  
  public LeftSideDoubleRunToMiddleBaseBLine(CommandSwerveDrivetrain S_Swerve, Intake S_Intake, Hopper S_Hopper, Shooter S_Shooter, List<Path> waypoints) {
    this.S_Swerve = S_Swerve;
    this.S_Hopper = S_Hopper;
    this.S_Intake = S_Intake;
    addRequirements(S_Swerve);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new InstantCommand(() -> S_Swerve.resetPose(AutoPaths.rotateBlueToRed(Constants.bumpLeftStartingPose, Constants.FIELD_LENGTH_METERS, Constants.FIELD_WIDTH_METERS))),
      new DriveOverBump(S_Swerve, 2)
        .withTimeout(1.5), 
      S_Swerve.pathBuilder.build(waypoints.get(1))
        .withTimeout(3.0)
        .alongWith(new InstantCommand(() -> S_Intake.deployIntake()))
        .alongWith(new InstantCommand(() -> S_Hopper.setFloorRPS(40))),
      S_Swerve.pathBuilder.build(waypoints.get(0)), //drive back to bump
      new DriveOverBump(S_Swerve,3)
        .withTimeout(2)
        .alongWith(new InstantCommand(() -> S_Intake.retractIntake())), 
      new Shoot(S_Swerve, S_Shooter, S_Hopper)
        .withTimeout(3.0),
      new DriveOverBump(S_Swerve, 2)
        .withTimeout(1.5),
      new RotateToAngleUntilTagsSeen(S_Swerve, Constants.LeftSideRotateToSeeTagsTarget),
      S_Swerve.pathBuilder.build(waypoints.get(2))
        .withTimeout(3.0)
        .alongWith(new InstantCommand(() -> S_Intake.deployIntake()))
        .alongWith(new InstantCommand(() -> S_Hopper.setFloorRPS(40))),
      S_Swerve.pathBuilder.build(waypoints.get(0)),
      new DriveOverBump(S_Swerve,3)
        .withTimeout(2)
        .alongWith(new InstantCommand(() -> S_Intake.retractIntake())),
      new Shoot(S_Swerve, S_Shooter, S_Hopper)
      
    );
  }
  
}
