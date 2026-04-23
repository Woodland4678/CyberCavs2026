// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  Shooter S_Shooter;
  Hopper S_Hopper;
  CommandSwerveDrivetrain S_Swerve;
  double shooterTargetRPS = 45;
  double startTime = 0;
  Translation2d hubDist;
  double hubX = 0;
  double hubY = 0;
  int hoodPos = 0;
  int state = 0;
  double xSpeed = 0;
  Debouncer shooterReadyDebounce = new Debouncer(0.06, Debouncer.DebounceType.kRising);
   private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);

   private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
   PhoenixPIDController rController = new PhoenixPIDController(19.1, 0, 0.15);
    private final SwerveRequest.RobotCentric m_driveRequestDrive = new SwerveRequest.RobotCentric()
            .withDeadband(4 * 0.1).withRotationalDeadband(6 * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  
  /** Creates a new Shoot. */
  public Shoot(CommandSwerveDrivetrain S_Swerve, Shooter S_Shooter, Hopper S_Hopper) {
    this.S_Hopper = S_Hopper;
    this.S_Shooter = S_Shooter;
    this.S_Swerve = S_Swerve;
    addRequirements(S_Shooter, S_Hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    //hoodPos = 0;
    S_Shooter.setHoodPosition(Constants.ShooterConstants.hoodRetractPosition);
   // S_Shooter.setShooterSpeedRPS(shooterTargetRPS);
    S_Hopper.setFloorRPS(117);

    startTime = Timer.getFPGATimestamp();
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        hubX = 11.915;
        hubY = 8.07/2;
        hubDist = Constants.RED_HUB_POSITION.getTranslation();

      } else {
        hubX = 4.626;
        hubY = 8.07/2;
        hubDist = Constants.BLUE_HUB_POSITION.getTranslation();
      }
    }
    rController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      // case 0:
      //   if (S_Swerve.getHasMultiTagTarget()) {
      //     state++;
      //   }
      //   else {
      //     S_Swerve.setControl(
      //       m_driveRequestAutoAlign.withVelocityX(-1)
      //           .withVelocityY(0)
      //           .withRotationalRate(0)
      //           .withDriveRequestType(DriveRequestType.Velocity)
      //     );
      //   }
      // break;
      case 0:
        // if (S_Swerve.isOdometryLikelyBad()) {
        //   xSpeed = -1.00;
        // }
        // else {
        //   xSpeed = 0.0;
        // }
        Pose2d robotPose = S_Swerve.getState().Pose;
        
        // Vector to hub
        double dX = hubX - robotPose.getX();
        double dY = hubY - robotPose.getY();

        // Desired field-relative heading
        Rotation2d targetHeading =
            Rotation2d.fromRadians(Math.atan2(dY, dX));

        double rSpeed = rController.calculate(robotPose.getRotation().getRadians(), targetHeading.getRadians(), Timer.getFPGATimestamp());
        S_Swerve.setControl(m_driveRequestDrive.withVelocityX(xSpeed).withVelocityY(0).withRotationalRate(rSpeed));
        SmartDashboard.putNumber("Auto Aim rspeed", rSpeed);
        SmartDashboard.putNumber("Target Angle", targetHeading.getDegrees());

        double distance =
            robotPose.getTranslation()
                .getDistance(hubDist);
        if (distance < 2.4) {
          hoodPos = 0;
        }
        else if (distance > 2.75) {
          hoodPos = 1;
        }
        shooterTargetRPS = S_Shooter.getDesiredShooterRPS(distance, hoodPos);
        if (Timer.getFPGATimestamp() - startTime < 0.35) {
          //shooterTargetRPS = S_Shooter.getDesiredShooterRPS(distance, hoodPos) * 1.05;
        }
        // else {
        //   shooterTargetRPS = 47.0;
        // }
        //shooterTargetRPS = 80;
        S_Shooter.setShooterSpeedRPS(shooterTargetRPS);
        boolean rawReady = Math.abs(S_Shooter.getShooterSpeedRPS() - shooterTargetRPS) < 0.85;
        boolean rotationReady = Math.abs(rController.getPositionError()) < Math.toRadians(3); //this radians
        boolean shooterReady = shooterReadyDebounce.calculate(rawReady && rotationReady);
        //double error = Math.abs(S_Shooter.getShooterSpeedRPS() - shooterTargetRPS);
        if (shooterReady) {
          S_Shooter.setFeederSpeed(Constants.ShooterConstants.feederShootRPS);
        }
        // else if (error > 8.0) {//slow down if shooter not within speed
        //   S_Shooter.stopFeeder();
        // }
        // else {
        //   S_Shooter.setFeederSpeed(20);
        // }
        SmartDashboard.putNumber("Distance to hub middle", distance);
        SmartDashboard.putNumber("Calculated RPS Speed", shooterTargetRPS);
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Shooter.stopFeeder();
    S_Shooter.setShooterSpeedRPS(Constants.ShooterConstants.idleRPS);
    S_Hopper.stopFloor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
