// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
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
  Debouncer shooterReadyDebounce = new Debouncer(0.04, Debouncer.DebounceType.kRising);
  
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
   // S_Shooter.setShooterSpeedRPS(shooterTargetRPS);
    S_Hopper.setFloorRPS(30);
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double distance =
        S_Swerve.getState().Pose.getTranslation()
            .getDistance(Constants.RED_HUB_POSITION.getTranslation());
    shooterTargetRPS = S_Shooter.getDesiredShooterRPS(distance);
    if (Timer.getFPGATimestamp() - startTime < 0.5) {
      shooterTargetRPS = S_Shooter.getDesiredShooterRPS(distance) * 1.15;
    }
    // else {
    //   shooterTargetRPS = 47.0;
    // }
    S_Shooter.setShooterSpeedRPS(shooterTargetRPS);
    boolean rawReady = Math.abs(S_Shooter.getShooterSpeedRPS() - shooterTargetRPS) < 1.75;
    boolean shooterReady = shooterReadyDebounce.calculate(rawReady);
    double error = Math.abs(S_Shooter.getShooterSpeedRPS() - shooterTargetRPS);
    if (shooterReady) {
      S_Shooter.setFeederSpeed(80);
    }
    // else if (error > 8.0) {//slow down if shooter not within speed
    //   S_Shooter.stopFeeder();
    // }
    // else {
    //   S_Shooter.setFeederSpeed(20);
    // }
     SmartDashboard.putNumber("Distance to hub middle", distance);
     SmartDashboard.putNumber("Calculated RPS Speed", shooterTargetRPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    S_Shooter.stopFeeder();
    S_Shooter.stopShooterMotor();
    S_Hopper.stopFloor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
