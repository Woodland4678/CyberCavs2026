// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoClimb extends Command {

  int state = 0;
  double xControllerSetpoint = 0;
  double rControllerSetpoint = 0;
  double yStartMovementTime = 0;
  double xSpeed = 0;
  double ySpeed = 0;
  double rSpeed = 0;
  double lidarReading = 0;
  boolean isDone = false;
  //PhoenixPIDController xController = new PhoenixPIDController(2, 0, 0.03); //for meters
  PhoenixPIDController xController = new PhoenixPIDController(0.04, 0, 0.001); //for cm
  //PhoenixPIDController yController = new PhoenixPIDController(0.37, 0, 0.02); //0.32, 0, 0.022 //for using camera pitch for lineup
  PhoenixPIDController yController = new PhoenixPIDController(0.036, 0, 0.002); //0.32, 0, 0.022 //for using lidar to line up
  PhoenixPIDController rController = new PhoenixPIDController(7.1, 0, 0.15);
  CommandSwerveDrivetrain S_Swerve;
  Climber S_Climber;
  boolean isShooterForward;
  private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  /** Creates a new AutoClimb. */
  public AutoClimb(CommandSwerveDrivetrain S_Swerve, Climber S_Climber, boolean isShooterForward) {
    this.S_Swerve = S_Swerve;
    this.isShooterForward = isShooterForward;
    this.S_Climber = S_Climber;
    addRequirements(S_Swerve, S_Climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    xController.setTolerance(2.5);
    rController.setTolerance(Math.toRadians(2));
    isDone = false;
    Optional<Alliance> ally = DriverStation.getAlliance();
    rController.enableContinuousInput(-Math.PI, Math.PI);
    if (isShooterForward) {
      xControllerSetpoint = Constants.SwerveConstants.LeftCLimbLidar;
    }
    else {
      xControllerSetpoint = Constants.SwerveConstants.RightCLimbLidar;
    }
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        if (isShooterForward) {
          rControllerSetpoint = 180;
        }
        else {
          rControllerSetpoint = 0;
        }
      }
      else {
        if (isShooterForward) {
          rControllerSetpoint = 0;
        }
        else {
          rControllerSetpoint = 180;
        }
      }
    }
    else {
      rControllerSetpoint = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case 0:
        if (ySpeed > 0) {
          state++;
          yStartMovementTime = Timer.getFPGATimestamp();
        }
      break;
      case 1:
        if (S_Swerve.getState().Speeds.vyMetersPerSecond < 0.5 && (Timer.getFPGATimestamp() - yStartMovementTime) > 0.5) {
          //S_Climber.moveClimberToPosition(Constants.ClimberConstants.retractPosition);
          isDone = true;
        }
      break;
    }
    if (isShooterForward) {
      lidarReading = S_Swerve.getRearLidar();
    }
    else {
      lidarReading = S_Swerve.getFrontLidar();
    }
    xSpeed = xController.calculate(lidarReading, xControllerSetpoint, Timer.getFPGATimestamp());
    rSpeed = rController.calculate(Math.toRadians(S_Swerve.getGyroValue()), rControllerSetpoint, Timer.getFPGATimestamp());
    if (xController.atSetpoint() && rController.atSetpoint()) {
      if (isShooterForward) {
        ySpeed = 1.0;
      }
      else {
        ySpeed = -1.0;
      }
    }
    else {
      state = 0;
      ySpeed = 0;
    }

     S_Swerve.setControl(
        m_driveRequestAutoAlign.withVelocityX(-xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rSpeed)
            .withDriveRequestType(DriveRequestType.Velocity)
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Swerve.setControl(
          m_driveRequestAutoAlign.withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(0)

        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
