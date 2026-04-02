// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestBLine extends SequentialCommandGroup {
  CommandSwerveDrivetrain S_Swerve;
  /** Creates a new TestBLine. */
  Path scorePath = new Path("path1");
  public TestBLine(CommandSwerveDrivetrain S_Swerve) {
    this.S_Swerve = S_Swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       S_Swerve.pathBuilder.build(scorePath)
    );
  }
}
