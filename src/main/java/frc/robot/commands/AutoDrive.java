// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoWaypoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {
  CommandSwerveDrivetrain S_Swerve;
  Optional<Alliance> ally = DriverStation.getAlliance();
  AutoWaypoint[] waypoints;
  Pose2d currentPose;
  int currentIndex = 0;
  double POSITION_TOLERANCE = 0.15; // 5 cm
  double MAX_LINEAR_SPEED = 3.5;   // m/s (use your drivetrain max)
  double MAX_ANGULAR_SPEED = Math.PI * 4; // rad/s
  double CRUISE_SPEED = 2.75;
  double ROTATION_TOLERANCE_RAD = Math.toRadians(8.0);
  boolean isDone = false;
  PIDController xController = new PIDController(8.0, 0.0, 0.0);
  PIDController yController = new PIDController(8.0, 0.0, 0.0);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
 // ProfiledPIDController xController2 = new ProfiledPIDController(8, 0, 0, new TrapezoidProfile.Constraints(5, 1));
 // ProfiledPIDController yController2 = new ProfiledPIDController(8, 0, 0, new TrapezoidProfile.Constraints(5, 1));
  ProfiledPIDController thetaController =
      new ProfiledPIDController(
          6.0, 0.0, 0.0,
          new TrapezoidProfile.Constraints(
              3 * Math.PI, Math.PI * 6));

  HolonomicDriveController controller =
      new HolonomicDriveController(
          xController, yController, thetaController);

  /** Creates a new AutoDrive. */
  public AutoDrive(CommandSwerveDrivetrain S_Swerve, AutoWaypoint[] waypoints) {
    this.S_Swerve = S_Swerve;
    this.waypoints = waypoints;
   // this.waypoints = waypoints;

    addRequirements(S_Swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    isDone = false;
    currentIndex = 0;
    currentPose = S_Swerve.getState().Pose;
    CRUISE_SPEED = waypoints[currentIndex].cruiseSpeed;
    MAX_LINEAR_SPEED = waypoints[currentIndex].maxSpeed;
    MAX_ANGULAR_SPEED = waypoints[currentIndex].maxAngularSpeed;
    POSITION_TOLERANCE = waypoints[currentIndex].translationTolerance;
    ROTATION_TOLERANCE_RAD = waypoints[currentIndex].rotationTolerance;
    // Pose2d[] path = {
    //   new Pose2d(6.715, 2.788, new Rotation2d().fromDegrees(154)),
    //   new Pose2d(7.93, 2.075, new Rotation2d().fromDegrees(154)),
    //   new Pose2d(5.873, 2.299, new Rotation2d().fromDegrees(45)),
    //   //new Pose2d(currentPose.getX() + 1, currentPose.getY(), Rotation2d.fromDegrees(-15)),
    //   //new Pose2d(currentPose.getX() + 2, currentPose.getY() - 0.25,  Rotation2d.fromDegrees(-45)),
    //   //new Pose2d(currentPose.getX() + 2, currentPose.getY() - 0.5,  Rotation2d.fromDegrees(-90)),
    //   //new Pose2d(currentPose.getX() + 1.5, currentPose.getY() - 0.25,  Rotation2d.fromDegrees(-140)),
    //   //new Pose2d(currentPose.getX() + 0.75, currentPose.getY() - 0.25,  Rotation2d.fromDegrees(-180)),
    //   //new Pose2d(currentPose.getX() + 1, currentPose.getY() - 1,  Rotation2d.fromDegrees(0)),
    //   //new Pose2d(currentPose.getX(), currentPose.getY(),  Rotation2d.fromDegrees(180)),
    //   //new Pose2d(5.648, 1.977,  Rotation2d.fromDegrees(45))
    //   //new Pose2d(currentPose.getX(), currentPose.getY() + 1, currentPose.getRotation()),
    //   //new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation())
    // };
   
  }

  @Override
  public void execute() {
    //double currentTime = timer.get();
    //Trajectory.State goal = trajectory.sample(currentTime);
    currentPose = S_Swerve.getState().Pose;
    Pose2d targetPose = waypoints[currentIndex].waypoint;
    SmartDashboard.putString("Auto Drive Target Pose", targetPose.toString());
    
    // Distance check (XY only)
    double distance =
        currentPose.getTranslation()
            .getDistance(targetPose.getTranslation());

    double rotationError =
    Math.abs(
        MathUtil.angleModulus(
            targetPose.getRotation().getRadians()
                - currentPose.getRotation().getRadians()
        )
    );
    SmartDashboard.putNumber("Auto Drive Error", distance);
    if (distance <= POSITION_TOLERANCE && rotationError < ROTATION_TOLERANCE_RAD) {
      currentIndex++;
      if (currentIndex >= waypoints.length) {
        isDone = true;
      } else {
        targetPose = waypoints[currentIndex].waypoint;
        CRUISE_SPEED = waypoints[currentIndex].cruiseSpeed;
        MAX_LINEAR_SPEED = waypoints[currentIndex].maxSpeed;
        MAX_ANGULAR_SPEED = waypoints[currentIndex].maxAngularSpeed;
        POSITION_TOLERANCE = waypoints[currentIndex].translationTolerance;
        ROTATION_TOLERANCE_RAD = waypoints[currentIndex].rotationTolerance;
      }
    }
    
    ChassisSpeeds speeds =
        controller.calculate(
            currentPose,
            targetPose,
            0.0,
            targetPose.getRotation());
      
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    //if (currentIndex < waypoints.length - 1) {
      if (linearSpeed < CRUISE_SPEED && distance > POSITION_TOLERANCE) {
        double scale = CRUISE_SPEED / linearSpeed;
        speeds.vxMetersPerSecond *= scale;
        speeds.vyMetersPerSecond *= scale;
      }
    //}
    if (linearSpeed > MAX_LINEAR_SPEED) {
      double scale = MAX_LINEAR_SPEED / linearSpeed;
      speeds.vxMetersPerSecond *= scale;
      speeds.vyMetersPerSecond *= scale;
    }
    speeds.omegaRadiansPerSecond =
    MathUtil.clamp(
        speeds.omegaRadiansPerSecond,
        -MAX_ANGULAR_SPEED,
        MAX_ANGULAR_SPEED);
    ChassisSpeeds fieldSpeeds =
    ChassisSpeeds.fromRobotRelativeSpeeds(
        speeds,
        currentPose.getRotation()
    );
    S_Swerve.setControl(
          drive.withVelocityX(-fieldSpeeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
              .withVelocityY(-fieldSpeeds.vyMetersPerSecond) // Drive left with negative X (left)
              .withRotationalRate(fieldSpeeds.omegaRadiansPerSecond)
              .withDriveRequestType(DriveRequestType.Velocity) // Drive counterclockwise with negative X (left)
      );
    // S_Swerve.setControl(
    //     new SwerveRequest.ApplyFieldSpeeds()
    //         .withSpeeds(fieldSpeeds));
    SmartDashboard.putNumber("Auto Drive x speed", fieldSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Auto Drive y speed ", fieldSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Auto Drive r speed", fieldSpeeds.omegaRadiansPerSecond);
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

