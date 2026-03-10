// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassFuel extends Command {

  CommandSwerveDrivetrain S_Swerve;

  // The x and y values of the hub.
  Double passPointX;
  Double passPointY;

  double robotX;
  double robotY;

  // Angle to turn to for passing fuel.
  Double targetAngle;

  PhoenixPIDController rController = new PhoenixPIDController(19.1, 0, 0.15);
    private final SwerveRequest.FieldCentric m_driveRequestDrive = new SwerveRequest.FieldCentric()
            .withDeadband(4 * 0.1).withRotationalDeadband(6 * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** Creates a new PassFuel. */
  public PassFuel(CommandSwerveDrivetrain S_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.S_Swerve = S_Swerve;
    addRequirements(S_Swerve);

    rController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
    public void initialize() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {

        passPointX = 15.3;
        passPointY = 1.33;

      } else if (ally.get() == Alliance.Blue) {

        passPointX = Constants.FIELD_LENGTH_METERS - 15.3;
        passPointY = 1.33;
      }
    }
    robotX = S_Swerve.getState().Pose.getX();
    robotY = S_Swerve.getState().Pose.getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = S_Swerve.getState().Pose;

    if (robotPose.getY() > Constants.FIELD_WIDTH_METERS/2) {
      passPointY = Constants.FIELD_WIDTH_METERS - 1.33;
    } else {
      passPointY = 1.33;
    }

    // Vector to passing point
    double dX = passPointX - robotPose.getX();
    double dY = passPointY - robotPose.getY();

    // Desired field-relative heading
    Rotation2d targetHeading =
        Rotation2d.fromRadians(Math.atan2(dY, dX));

    double rSpeed = rController.calculate(robotPose.getRotation().getRadians(), targetHeading.getRadians(), Timer.getFPGATimestamp());
    S_Swerve.setControl(m_driveRequestDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(rSpeed));
    SmartDashboard.putNumber("Passing rspeed", rSpeed);
    SmartDashboard.putNumber("Target Angle", targetHeading.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set all speeds to 0.
    S_Swerve.setControl(m_driveRequestDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
