// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualShot extends Command {
  /** Creates a new ManualShot. */
  Shooter S_Shooter;
  Hopper S_Hopper;
  double shooterRPS;
  double hoodPos;
  Debouncer shooterReadyDebounce = new Debouncer(0.06, Debouncer.DebounceType.kRising);
  
  public ManualShot(Shooter S_Shooter, Hopper S_Hopper, double shooterRPS, double hoodPos) {
    this.S_Shooter = S_Shooter;
    this.S_Hopper = S_Hopper;
    this.shooterRPS = shooterRPS;
    this.hoodPos = hoodPos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    S_Shooter.setHoodPosition(hoodPos);
    S_Shooter.setShooterSpeedRPS(shooterRPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean rawReady = Math.abs(S_Shooter.getShooterSpeedRPS() - shooterRPS) < 0.75;
    boolean shooterReady = shooterReadyDebounce.calculate(rawReady);
    if (shooterReady) {
      S_Shooter.setFeederSpeed(Constants.ShooterConstants.feederShootRPS);
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
