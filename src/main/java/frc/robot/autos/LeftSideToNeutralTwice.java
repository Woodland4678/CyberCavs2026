// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoWaypoint;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveOverBump;
import frc.robot.subsystems.CommandSwerveDrivetrain;

      
      

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftSideToNeutralTwice extends SequentialCommandGroup {
  /** Creates a new RightSideToNeutralTwice. */
  CommandSwerveDrivetrain S_Swerve;
  public static final Field2d field = new Field2d();
  
  public LeftSideToNeutralTwice(CommandSwerveDrivetrain S_Swerve, List<AutoWaypoint[]> waypoints) {
    this.S_Swerve = S_Swerve;
    addRequirements(S_Swerve);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // field.getObject("Auto Waypoints")
    //  .setPoses(extractPoses(waypoints.get(0)));
    //  SmartDashboard.putData("Auto Field", field);
    addCommands(
      new DriveOverBump(S_Swerve,2),
      new AutoDrive(S_Swerve, waypoints.get(0)),
      new DriveOverBump(S_Swerve, 3),
      new AutoAim(S_Swerve).withTimeout(3.5),
      new DriveOverBump(S_Swerve, 2),
      new AutoDrive(S_Swerve, waypoints.get(1)),
      new DriveOverBump(S_Swerve, 3),
      new AutoAim(S_Swerve).withTimeout(3.5)
      //S_Swerve.pathOnTheFly(waypoints, rotationTargets, constraints)
      //new DriveOverBump(S_Swerve),
      //S_Swerve.findPath(path1, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI))
      //S_Swerve.getAutonomousCommand(),
      //new AutoDrive(S_Swerve)


    );
  }
  public static Pose2d[] extractPoses(AutoWaypoint[] waypoints) {
    Pose2d[] poses = new Pose2d[waypoints.length];
    for (int i = 0; i < waypoints.length; i++) {
        poses[i] = waypoints[i].waypoint;
    }
    return poses;
  }
}
