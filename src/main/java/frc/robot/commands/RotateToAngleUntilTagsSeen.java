// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LEDStrip;
import frc.robot.LEDStrip.LEDModes;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngleUntilTagsSeen extends Command {
  CommandSwerveDrivetrain S_Swerve;
  double angle;
  boolean isDone = false;
      private Command m_autonomousCommand;
    private LEDStrip ledStrip;
  PhoenixPIDController rController = new PhoenixPIDController(0.15, 0, 0.0); //14.1, 0, 0.15
   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  /** Creates a new RotateToAngleUntilTagsSeen. */
  public RotateToAngleUntilTagsSeen(CommandSwerveDrivetrain S_Swerve, double angle) {
    this.angle = angle;
    this.S_Swerve = S_Swerve;
     ledStrip = LEDStrip.getInstance();
        ledStrip.setLEDMode(LEDModes.SOLIDRED); 
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     Optional<Alliance> ally = DriverStation.getAlliance();
     if (ally.get() == Alliance.Red) {
      if (angle > 0) {
        angle -= 180;
      }
      else {
        angle += 180;
      }
     }
    isDone = false;
    rController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rSpeed = rController.calculate(S_Swerve.getGyroValue(), this.angle, Timer.getFPGATimestamp());
    S_Swerve.setControl(
          drive.withVelocityX(0) // Drive forward with negative Y (forward)
              .withVelocityY(0) // Drive left with negative X (left)
              .withRotationalRate(rSpeed)
              .withDriveRequestType(DriveRequestType.Velocity) // Drive counterclockwise with negative X (left)
      );
    if (S_Swerve.getHasMultiTagTarget()) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     S_Swerve.setControl(
          drive.withVelocityX(0) // Drive forward with negative Y (forward)
              .withVelocityY(0) // Drive left with negative X (left)
              .withRotationalRate(0)
              .withDriveRequestType(DriveRequestType.Velocity) // Drive counterclockwise with negative X (left)
      );
      ledStrip.setLEDMode(LEDModes.SOLIDBLUE); // SDW
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
