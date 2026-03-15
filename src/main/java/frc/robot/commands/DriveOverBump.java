// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveOverBump extends Command {
  /** Creates a new DriveOverBump. */
   PhoenixPIDController rController = new PhoenixPIDController(0.25, 0, 0.0); //14.1, 0, 0.15
   Optional<Alliance> ally = DriverStation.getAlliance();
  CommandSwerveDrivetrain S_Swerve;
  double angleToTarget;
  double targetX = 5.223; //from blue alliance wall to middle of blue hub
  double targetY = 4.035;
  int state = 0;
  int cnt = 0;
  double xSpeed;
  double ySpeed = 0.0;
  boolean isDone = false;
  double gyroAxisValue = 0.0;
  double firstThreshold = -9.0;
  double secondThreshold = 9.0;
  int directionType = 0;
  /*
  
  0 & 1 are used for right side auto (0 goes to middle zone, 1 comes back)
  2 & 3 are used for left side auto (2 goes to middle zone and 3 comes back)
  */
  double[] directionTypeRotationAngles = {135, 45, -135,-45};
  // private final SwerveRequest.FieldCentric m_driveRequestDrive = new SwerveRequest.FieldCentric()
  //           .withDeadband(4 * 0.1).withRotationalDeadband(6 * 0.1) // Add a 10% deadband
  //           .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.FieldCentric m_driveRequestDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
  public DriveOverBump(CommandSwerveDrivetrain S_Swerve, int directionType) {
    this.S_Swerve = S_Swerve;
    this.directionType = directionType;
    addRequirements(S_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    rController.enableContinuousInput(-180, 180);
    if (directionType == 0 || directionType == 2) {
      xSpeed = 3.5;
    }
    else {
      xSpeed = -3.5;
    }
    state = 0;
    cnt = 0;
    isDone = false;
     if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        //xSpeed *= -1;
        directionTypeRotationAngles[0] = -45;
        directionTypeRotationAngles[1] = -135;
        directionTypeRotationAngles[2] = 45;
        directionTypeRotationAngles[3] = 135;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (directionType == 0 || directionType == 3) {
      gyroAxisValue = S_Swerve.getGyroPitch();
      firstThreshold = -3.0;
      secondThreshold = 3.0;
    }
    else if (directionType == 1 || directionType == 2) {
      gyroAxisValue = S_Swerve.getGyroRoll();
      firstThreshold = -3.0;
      secondThreshold = 3.0;
    }
    
    double rSpeed = rController.calculate(S_Swerve.getGyroValue(), directionTypeRotationAngles[directionType], Timer.getFPGATimestamp());
    S_Swerve.setControl(
          m_driveRequestDrive.withVelocityX(xSpeed)
              .withVelocityY(0)
              .withRotationalRate(rSpeed)
              .withDriveRequestType(DriveRequestType.Velocity) // Drive counterclockwise with negative X (left)

        );
        //I conclude looking for one direction, then the other, then flat will be more consistent
      switch(state) {
        case 0:
          if (firstThreshold > 0) {
            if (gyroAxisValue > firstThreshold) {
              state++;
            }
          }
          else {
            if (gyroAxisValue < firstThreshold) {
              state++;
            }
          }
          
        break;
        case 1:
          if (secondThreshold > 0) {
            if (gyroAxisValue > secondThreshold) {
              state++;
            }
          }
          else {
            if (gyroAxisValue < secondThreshold) {
              state++;
            }
          }
        break;
        case 2:
          if ((gyroAxisValue > -7 && gyroAxisValue < 7)) {
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
