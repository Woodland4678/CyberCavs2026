// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {
  CommandSwerveDrivetrain S_Swerve;

  Pose2d currentPose;
  Pose2d[] waypoints;
  int currentIndex = 0;
  boolean isDone = false;

  double POSITION_TOLERANCE = 0.2; // 5 cm
  static final double MAX_LINEAR_SPEED = 4.4;   // m/s (use your drivetrain max)
  static final double MAX_ANGULAR_SPEED = Math.PI * 2; // rad/s
  static final double CRUISE_SPEED = 4.0;

  PIDController xController = new PIDController(8.0, 0.0, 0.0);
  PIDController yController = new PIDController(8.0, 0.0, 0.0);

  ProfiledPIDController thetaController =
      new ProfiledPIDController(
          8.0, 0.0, 0.0,
          new TrapezoidProfile.Constraints(
              2 * Math.PI, Math.PI * 4));

  HolonomicDriveController controller =
      new HolonomicDriveController(
          xController, yController, thetaController);

  /** Creates a new AutoDrive. */
  public AutoDrive(CommandSwerveDrivetrain S_Swerve) {
    this.S_Swerve = S_Swerve;
   // this.waypoints = waypoints;

    addRequirements(S_Swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    isDone = false;
    currentIndex = 0;
    currentPose = S_Swerve.getState().Pose;
    POSITION_TOLERANCE = 0.20;
    Pose2d[] path = {
      new Pose2d(currentPose.getX() + 1, currentPose.getY(), currentPose.getRotation()),
      new Pose2d(currentPose.getX() + 2, currentPose.getY() + 1,  Rotation2d.fromDegrees(20)),
      new Pose2d(currentPose.getX(), currentPose.getY(),  Rotation2d.fromDegrees(0))
      //new Pose2d(currentPose.getX(), currentPose.getY() + 1, currentPose.getRotation()),
      //new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation())
    };
    waypoints = path;
    SmartDashboard.putString("AutoDrive Status", "Started");
  }

  @Override
  public void execute() {
    currentPose = S_Swerve.getState().Pose;
    Pose2d targetPose = waypoints[currentIndex];
    SmartDashboard.putString("Auto Drive Target Pose", targetPose.toString());
    if (currentIndex == waypoints.length - 1) {
      POSITION_TOLERANCE = 0.01;
    }
    // Distance check (XY only)
    double distance =
        currentPose.getTranslation()
            .getDistance(targetPose.getTranslation());
    SmartDashboard.putNumber("Auto Drive Error", distance);
    if (distance <= POSITION_TOLERANCE) {
      currentIndex++;
      if (currentIndex >= waypoints.length) {
        isDone = true;
      } else {
        targetPose = waypoints[currentIndex];
      }
    }

    ChassisSpeeds speeds =
        controller.calculate(
            currentPose,
            targetPose,
            0.0,
            targetPose.getRotation());
      
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    if (currentIndex < waypoints.length - 1) {
      if (linearSpeed < CRUISE_SPEED) {
        double scale = CRUISE_SPEED / linearSpeed;
        speeds.vxMetersPerSecond *= scale;
        speeds.vyMetersPerSecond *= scale;
      }
    }
    if (linearSpeed > MAX_LINEAR_SPEED) {
      double scale = MAX_LINEAR_SPEED / linearSpeed;
      speeds.vxMetersPerSecond *= scale;
      speeds.vyMetersPerSecond *= scale;
    }
    // speeds.omegaRadiansPerSecond =
    // MathUtil.clamp(
    //     speeds.omegaRadiansPerSecond,
    //     -MAX_ANGULAR_SPEED,
    //     MAX_ANGULAR_SPEED);

    S_Swerve.setControl(
        new SwerveRequest.ApplyFieldSpeeds()
            .withSpeeds(speeds));
    SmartDashboard.putString("Auto Drive speeds", speeds.toString());
    SmartDashboard.putNumber("AutoDrive Target Index", currentIndex);
    SmartDashboard.putNumber("AutoDrive Distance", distance);
  }

  @Override
  public void end(boolean interrupted) {
    S_Swerve.setControl(
        new SwerveRequest.ApplyFieldSpeeds()
            .withSpeeds(new ChassisSpeeds()));
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}

