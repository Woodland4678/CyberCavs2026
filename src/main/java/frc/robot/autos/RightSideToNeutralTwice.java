// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveOverBump;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightSideToNeutralTwice extends SequentialCommandGroup {
  /** Creates a new RightSideToNeutralTwice. */
  CommandSwerveDrivetrain S_Swerve;
  public RightSideToNeutralTwice(CommandSwerveDrivetrain S_Swerve) {
    this.S_Swerve = S_Swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveOverBump(S_Swerve),
      S_Swerve.getAutonomousCommand()
      //new AutoDrive(S_Swerve)


    );
  }
}
