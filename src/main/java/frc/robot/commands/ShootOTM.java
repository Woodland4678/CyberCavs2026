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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOTM extends Command {
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
  double k = 1.5;
  CommandXboxController joystick;
  double xSpeed = 0;
  Debouncer shooterReadyDebounce = new Debouncer(0.06, Debouncer.DebounceType.kRising);
   private final SwerveRequest.RobotCentric m_driveRequestAutoAlign = new SwerveRequest.RobotCentric()
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagicExpo);
   PhoenixPIDController rController = new PhoenixPIDController(11.1, 0, 0.15);

  private final SwerveRequest.FieldCentric m_driveRequestDrive = new SwerveRequest.FieldCentric()
            .withDeadband(4 * 0.1).withRotationalDeadband(6 * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
  
  /** Creates a new Shoot. */
  public ShootOTM(CommandSwerveDrivetrain S_Swerve, Shooter S_Shooter, Hopper S_Hopper, CommandXboxController joystick) {
    this.S_Hopper = S_Hopper;
    this.S_Shooter = S_Shooter;
    this.S_Swerve = S_Swerve;
    this.joystick = joystick;
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
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
            S_Swerve.getState().Speeds,
            S_Swerve.getState().Pose.getRotation()
        );
        Pose2d robotPose = S_Swerve.getState().Pose;
         double distance =
            robotPose.getTranslation()
                .getDistance(hubDist);
        //Time of flight needed, get a map for that too
       // double timeOfFlight = 0.9;
        double timeOfFlight = S_Shooter.getTimeOfFlight(distance);
        double lookaheadX = robotPose.getX() + robotVelocity.vxMetersPerSecond * timeOfFlight;
        double lookaheadY = robotPose.getY() + robotVelocity.vyMetersPerSecond * timeOfFlight;

        double targetAngle = Math.atan2(hubY - lookaheadY, hubX - lookaheadX);

        double lookaheadDistance = Math.hypot(hubX - lookaheadX, hubY - lookaheadY);

        if (distance < 2.4) {
          hoodPos = 0;
        }
        else if (distance > 2.75) { 
          hoodPos = 1;
        }
        double shotUnitX = (hubX - robotPose.getX()) / distance;
        double shotUnitY = (hubY - robotPose.getY())/distance;
        double velocityAlongShot = (robotVelocity.vxMetersPerSecond * shotUnitX) + (robotVelocity.vyMetersPerSecond * shotUnitY);

        shooterTargetRPS = S_Shooter.getDesiredShooterRPS(lookaheadDistance, hoodPos);
        // if (Timer.getFPGATimestamp() - startTime < 0.55) {
        //   shooterTargetRPS = S_Shooter.getDesiredShooterRPS(lookaheadDistance, hoodPos) * 1.07;
        // }
        shooterTargetRPS += -velocityAlongShot * k;
        double rSpeed = rController.calculate(robotPose.getRotation().getRadians(), targetAngle, Timer.getFPGATimestamp());
        S_Swerve.setControl(m_driveRequestDrive.withVelocityX(-joystick.getLeftY() * Constants.SwerveConstants.MaxSpeed * 0.2).withVelocityY(-joystick.getLeftX() * Constants.SwerveConstants.MaxSpeed * 0.2).withRotationalRate(rSpeed));
        
        SmartDashboard.putNumber("Auto Aim rspeed", rSpeed);
        SmartDashboard.putNumber("Target Angle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Look ahead X", lookaheadX);
        SmartDashboard.putNumber("Look ahead Y", lookaheadY);

        S_Shooter.setShooterSpeedRPS(shooterTargetRPS);
        boolean rawReady = Math.abs(S_Shooter.getShooterSpeedRPS() - shooterTargetRPS) < 0.85;
        boolean rotationReady = Math.abs(rController.getPositionError()) < Math.toRadians(3); //this radians
        boolean shooterReady = shooterReadyDebounce.calculate(rawReady && rotationReady);
        //double error = Math.abs(S_Shooter.getShooterSpeedRPS() - shooterTargetRPS);
        if (shooterReady) {
          S_Shooter.setFeederSpeed(Constants.ShooterConstants.feederShootRPS);
          S_Hopper.setFloorRPS(117);
        }

        SmartDashboard.putNumber("Distance to hub middle", lookaheadDistance);
        SmartDashboard.putNumber("Calculated RPS Speed", shooterTargetRPS);
      
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
