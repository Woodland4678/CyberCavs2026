// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoWaypoint;
import frc.robot.autos.AutoPaths;
import frc.robot.autos.LeftSideToNeutralTwice;
import frc.robot.autos.RightSideToNeutralTwice;
import frc.robot.autos.RightSideToNeutralTwiceBehindHub;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveOverBump;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private String lastSelectedAuto = "";
    private final Field2d field = new Field2d();
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private PathPlannerPath examplePath;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final Map<String, AutoDefinition> autos = Map.of(
        "RightSideToNeutralTwice",
            new AutoDefinition(
                new RightSideToNeutralTwice(drivetrain, AutoPaths.RightSideGatherFuel1),
                AutoPaths.RightSideGatherFuel1
            ),

        "LeftSideToNeutralTwice",
            new AutoDefinition(
                new LeftSideToNeutralTwice(drivetrain, AutoPaths.LeftSideGatherFuel1),
                AutoPaths.LeftSideGatherFuel1
            )
    );

    public final Hopper S_Hopper = new Hopper();
    //public final Climber S_Climber = new Climber();
    public final Shooter S_Shooter = new Shooter();
    public final Intake S_Intake = new Intake();
    public RobotContainer() {
        configureBindings();
        configureAutoChooser();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

       // joystick.povUp().onTrue(new InstantCommand(() -> S_Hopper.setFloorRPM(500)));
       // joystick.povUp().onFalse(new InstantCommand(() -> S_Hopper.stopFloor()));
//
       // joystick.povLeft().onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(100)));//normal position (placeholder vlaue)
       // joystick.povLeft().onFalse(new InstantCommand(() -> S_Climber.stopClimber()));
       // joystick.povRight().onTrue(new InstantCommand(() -> S_Climber.moveClimberToPosition(200)));//up position (placeholder value)
       // joystick.povRight().onFalse(new InstantCommand(() -> S_Climber.stopClimber()));

        //joystick.leftTrigger().onTrue(new InstantCommand(() -> S_Shooter.setShooterSpeedRPS(70)));//up position (placeholder value)
        //joystick.leftTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Hopper.setFloorVoltage(10)));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Hopper.stopFloor()));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setFeederVoltage(9)));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopFeeder()));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setShooterVoltage(2)));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Intake.setIntakeWheelVoltage(3)));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Intake.stopIntakeWheels()));

        joystick.leftTrigger().onTrue(new InstantCommand(() -> S_Hopper.setFloorVoltage(5)));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> S_Hopper.stopFloor()));
        joystick.leftTrigger().onTrue(new InstantCommand(() -> S_Intake.setIntakeWheelVoltage(8)));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> S_Intake.stopIntakeWheels()));
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

       // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

       // joystick.rightTrigger().whileTrue(new AutoAim(drivetrain));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.x().whileTrue(new DriveOverBump(drivetrain, 0));
        joystick.y().whileTrue(new DriveOverBump(drivetrain, 1));
        joystick.povUp().whileTrue(new DriveOverBump(drivetrain, 2));
        joystick.povDown().whileTrue(new DriveOverBump(drivetrain, 3));
        //joystick.a().whileTrue(new AutoDrive(drivetrain));
        //joystick.y().whileTrue(drivetrain.getAutonomousCommand());
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return new RightSideToNeutralTwiceBehindHub(drivetrain, AutoPaths.RightSideGatherFuel2);
       
    }
    private void configureAutoChooser() {

        boolean first = true;

        for (Map.Entry<String, AutoDefinition> entry : autos.entrySet()) {
            if (first) {
                autoChooser.setDefaultOption(entry.getKey(), entry.getValue().command);
                first = false;
            } else {
                autoChooser.addOption(entry.getKey(), entry.getValue().command);
            }
        }

        SmartDashboard.putData("Autonomous Mode", autoChooser);
        
        //// Example commands
        //Command rightNeutralZoneTwice1 = new RightSideToNeutralTwice(drivetrain, AutoPaths.RightSideGatherFuel1);
        //Command leftNeutralZoneTwice1 = new LeftSideToNeutralTwice(drivetrain, AutoPaths.LeftSideGatherFuel1);
        ////Command simpleDriveAuto = new SimpleDriveAuto(s_Swerve);
//
        //// Add them to the chooser
        //autoChooser.setDefaultOption("Right side neutral zone twice", rightNeutralZoneTwice1); // default
        //autoChooser.addOption("Left side neutral zone twice", leftNeutralZoneTwice1);
        ////autoChooser.addOption("Simple Drive Auto", simpleDriveAuto);
    }
    public void updateAutoPreview() {
        Command selectedAuto = autoChooser.getSelected();

        if (selectedAuto == null) {
            // Nothing selected yet — dashboard not ready
            return;
        }

        String selectedKey = selectedAuto.getName();
        //System.out.println("selected key " + selectedKey);

        if (selectedKey.equals(lastSelectedAuto)) {
            return;
        }

        lastSelectedAuto = selectedKey;

        AutoDefinition auto = autos.get(selectedKey);
        if (auto == null) return;

        Pose2d[] poses = AutoPaths.extractPoses(auto.paths());

       // field.getObject("AutoPath").close();
        field.getObject("AutoPath").setPoses(poses);
        SmartDashboard.putData("Auto Field", field);
    }
    public record AutoDefinition(
        Command command,
        List<AutoWaypoint[]> paths
    ) {}
}
