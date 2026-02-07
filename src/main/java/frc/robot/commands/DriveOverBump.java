// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveOverBump extends Command {
  /** Creates a new DriveOverBump. */
   PhoenixPIDController rController = new PhoenixPIDController(14.1, 0, 0.15);
  CommandSwerveDrivetrain S_Swerve;
  double angleToTarget;
  double targetX = 5.223; //from blue alliance wall to middle of blue hub
  double targetY = 4.035;
  int state = 0;
  int cnt = 0;
  double xSpeed;
  double ySpeed = 0.0;
  boolean isDone = false;
  private final SwerveRequest.FieldCentric m_driveRequestDrive = new SwerveRequest.FieldCentric()
            .withDeadband(4 * 0.1).withRotationalDeadband(6 * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  public DriveOverBump(CommandSwerveDrivetrain S_Swerve) {
    this.S_Swerve = S_Swerve;
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    cnt = 0;
    xSpeed = 4.5;
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    S_Swerve.setControl(
          m_driveRequestDrive.withVelocityX(xSpeed)
              .withVelocityY(ySpeed)
              .withRotationalRate(0)

        );
        //I conclude looking for one direction, then the other, then flat will be more consistent
      switch(state) {
        case 0:
          if (S_Swerve.getGyroRoll() > 3) {
            state++;
          }
        break;
        case 1:
          if (S_Swerve.getGyroRoll() < -3) {
            state++;
          }
        break;
        case 2:
          if ((S_Swerve.getGyroRoll() > -1.5 && S_Swerve.getGyroRoll() < 1.5)) {
            //state++;
            cnt++;
          }
          else {
            cnt = 0;
          }
          if (cnt > 5) {
            state++;
          }
        break;
        case 3:
          xSpeed = 0;
          ySpeed = 0;
          isDone = true;
        break;
      }
      SmartDashboard.putNumber("Get Over Bump State", state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Swerve.setControl(
          m_driveRequestDrive.withVelocityX(0)
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
