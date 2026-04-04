// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
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
import frc.robot.RobotContainer.AutoDefinition;
import frc.robot.autos.AutoPaths;
import frc.robot.autos.LeftSideDisruption;
import frc.robot.autos.LeftSideDisruptionDepot;
import frc.robot.autos.LeftSideDoubleRunToMiddleBase;
import frc.robot.autos.LeftSideDoubleRunToMiddleBaseBLine;
import frc.robot.autos.LeftSideHubSweepCorralClimb;
import frc.robot.autos.LeftSideMiddleCorralClimb;
import frc.robot.autos.LeftSideSweepWallCorralClimb;
import frc.robot.autos.RightSideDisruption;
import frc.robot.autos.RightSideDoubleRunToMiddleBase;
import frc.robot.autos.RightSideDoubleRunToMiddleBaseBLine;
import frc.robot.autos.RightSideFullHopperThenClimb;
import frc.robot.autos.RightSideFullHopperThenFill;
import frc.robot.autos.TestBLine;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveOverBump;
import frc.robot.commands.ManualShot;
import frc.robot.commands.PassFuel;
import frc.robot.commands.RotateToAngleUntilTagsSeen;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private LEDStrip ledStrip;
    private String lastSelectedAuto = "";
    private final Field2d field = new Field2d();
    Optional<Alliance> ally;
    //private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private PathPlannerPath examplePath;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.SwerveConstants.MaxSpeed * 0.1).withRotationalDeadband(Constants.SwerveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.SwerveConstants.MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    public final Hopper S_Hopper = new Hopper();
    public final Climber S_Climber = new Climber();
    public final Shooter S_Shooter = new Shooter();
    public final Intake S_Intake = new Intake();

    private final Map<String, BLineAutoDefinition> autos_BLine = Map.ofEntries(
        Map.entry("Right Side Middle Run Sweep Hub",
            new BLineAutoDefinition(
                paths -> new RightSideDoubleRunToMiddleBaseBLine(
                    drivetrain, S_Intake, S_Hopper, S_Shooter, paths
                ),
                new String[] { "ReturnToBump", "RunMiddle", "SweepHub" },
                Constants.bumpRightStartingPose,
                false
            )
        ),
        Map.entry("Right Side Disruption Then Sweep Hub",
            new BLineAutoDefinition(
                paths -> new RightSideDoubleRunToMiddleBaseBLine(
                    drivetrain, S_Intake, S_Hopper, S_Shooter, paths
                ),
                new String[] { "ReturnToBump", "Disruption", "SweepHub" },
                Constants.bumpRightStartingPose,
                false
            )
        ),
        Map.entry("Right Side Shallow Semi Circle",
            new BLineAutoDefinition(
                paths -> new RightSideDoubleRunToMiddleBaseBLine(
                    drivetrain, S_Intake, S_Hopper, S_Shooter, paths
                ),
                new String[] { "ReturnToBump", "ShallowSemiCircle", "ShallowSemiCircle2" },
                Constants.bumpRightStartingPose,
                false
            )
        ),


        Map.entry("Left Side Middle Run Sweep Hub",
            new BLineAutoDefinition(
                paths -> new LeftSideDoubleRunToMiddleBaseBLine(
                    drivetrain, S_Intake, S_Hopper, S_Shooter, paths
                ),
                new String[] { "ReturnToBump", "RunMiddle", "SweepHub" },
                Constants.bumpLeftStartingPose,
                true
            )
        ),
        Map.entry("Left Side Disruption Then Sweep Hub",
            new BLineAutoDefinition(
                paths -> new LeftSideDoubleRunToMiddleBaseBLine(
                    drivetrain, S_Intake, S_Hopper, S_Shooter, paths
                ),
                new String[] { "ReturnToBump", "Disruption", "SweepHub" },
                Constants.bumpLeftStartingPose,
                true
            )
        ),
        Map.entry("Left Side Shallow Semi Circle",
            new BLineAutoDefinition(
                paths -> new LeftSideDoubleRunToMiddleBaseBLine(
                    drivetrain, S_Intake, S_Hopper, S_Shooter, paths
                ),
                new String[] { "ReturnToBump", "ShallowSemiCircle", "ShallowSemiCircle2" },
                Constants.bumpLeftStartingPose,
                true
            )
        )
    );

    private final Map<String, AutoDefinition> autos = Map.ofEntries(
        // Map.entry("RightSideMiddleSemiCircleThenSweepHub",
        //     new AutoDefinition(
        //         paths -> new RightSideDoubleRunToMiddleBase(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.RightSideSemiCircleThenSweepHub,
        //         new Pose2d(3.573, 2.579, Rotation2d.fromDegrees(90))
        //     )
        // ),

        // Map.entry("RightSideFullHopperThenClimb",
        //     new AutoDefinition(
        //         paths -> new RightSideFullHopperThenClimb(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.RightSideFullHopperThenClimb,
        //         new Pose2d(3.573, 2.23, Rotation2d.fromDegrees(90))
        //     )
        // ),
        // Map.entry("RightSideFullHopperThenFill",
        //     new AutoDefinition(
        //         paths -> new RightSideFullHopperThenFill(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.RightSideFullHopperThenFill,
        //         new Pose2d(3.573, 2.23, Rotation2d.fromDegrees(90))
        //     )
        // ),

        Map.entry("LeftSideMiddleThenHubSweep",
            new AutoDefinition(
                paths -> new LeftSideDoubleRunToMiddleBase(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.LeftSideMiddleThenSweepHub,
                new Pose2d(3.573, 5.3633, Rotation2d.fromDegrees(-90))
            )
        ),
        //   Map.entry("LeftSideMiddleThenHubSweepOption2",
        //     new AutoDefinition(
        //         paths -> new LeftSideDoubleRunToMiddleBase(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.LeftSideMiddleThenSweepHub,
        //         new Pose2d(3.573, 5.3633, Rotation2d.fromDegrees(-90))
        //     )
        // ),

        // Map.entry("LeftSideDisturbedMiddlePathOption1",
        //     new AutoDefinition(
        //         paths -> new LeftSideDoubleRunToMiddleBase(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.LeftSideDisturbedMiddlePathOption1,
        //         new Pose2d(3.573, 2.579, Rotation2d.fromDegrees(90))
        //     )
        // ),

        // Map.entry("LeftSideDisturbedMiddlePathOption2",
        //     new AutoDefinition(
        //         paths -> new LeftSideDoubleRunToMiddleBase(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.LeftSideDisturbedMiddlePathOption2,
        //         new Pose2d(3.573, 2.579, Rotation2d.fromDegrees(90))
        //     )
        // ),

        // Map.entry("LeftSideMiddleOnceCorralClimb",
        //     new AutoDefinition(
        //         paths -> new LeftSideMiddleCorralClimb(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.LeftSideMiddleCorralClimbPaths,
        //         new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
        //     )
        // ),

        // Map.entry("LeftSideHubSweepCorralClimb",
        //     new AutoDefinition(
        //         paths -> new LeftSideHubSweepCorralClimb(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.LeftSideHubSweepCorralClimb,
        //         new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
        //     )
        // ),

        // Map.entry("LeftSideSweepWallCorralClimb",
        //     new AutoDefinition(
        //         paths -> new LeftSideSweepWallCorralClimb(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
        //         AutoPaths.LeftSideSweepWallCorral,
        //         new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
        //     )
        // ),

        Map.entry("RightSideMiddleThenSweepHub",
            new AutoDefinition(
                paths -> new RightSideDoubleRunToMiddleBase(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.RightSideMiddleThenSweepHub,
                new Pose2d(3.573, 2.23, Rotation2d.fromDegrees(90))
            )
        ),

        Map.entry("RightSideShallowSemicircle",
            new AutoDefinition(
                paths -> new RightSideDoubleRunToMiddleBase(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.RightSideShallowSemicircle,
                new Pose2d(3.573, 2.23, Rotation2d.fromDegrees(90))
            )
        ),
         Map.entry("LeftSideShallowSemicircle",
            new AutoDefinition(
                paths -> new LeftSideDoubleRunToMiddleBase(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.LeftSideShallowSemicircle,
                new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
            )
        ),

        Map.entry("RightSideDisruption",
            new AutoDefinition(
                paths -> new RightSideDisruption(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.RightSideDisruption,
                new Pose2d(3.573, 2.23, Rotation2d.fromDegrees(90))
            )
        ),
         Map.entry("RightSideDisruptionLegal",
            new AutoDefinition(
                paths -> new RightSideDisruption(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.RightSideDisruptionLegal,
                new Pose2d(3.573, 2.23, Rotation2d.fromDegrees(90))
            )
        ),
        Map.entry("LeftSideDisruption",
            new AutoDefinition(
                paths -> new LeftSideDisruption(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.LeftSideDisruption,
                new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
            )
        ),
        Map.entry("LeftSideDisruptionLegal",
            new AutoDefinition(
                paths -> new LeftSideDisruption(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.LeftSideDisruptionLegal,
                new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
            )
        ),
        Map.entry("LeftSideDisruptionDepot",
            new AutoDefinition(
                paths -> new LeftSideDisruptionDepot(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
                AutoPaths.LeftSideDisruptionDepot,
                new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
            )
        )
    );

   
    public RobotContainer() {
        configureBindings();
        configureAutoChooser();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

       // joystick.povUp().onTrue(new InstantCommand(() -> S_Hopper.setFloorRPM(500)));
       // joystick.povUp().onFalse(new InstantCommand(() -> S_Hopper.stopFloor()));
//        joystick.povUp().whileTrue(new RotateToAngleUntilTagsSeen(drivetrain, Constants.RightSideRotateToSeeTagsTarget));
//        joystick.povDown().whileTrue(new RotateToAngleUntilTagsSeen(drivetrain, Constants.LeftSideRotateToSeeTagsTarget));
//
       joystick.povRight().onTrue(new InstantCommand(() -> S_Climber.extendClimber()));
       joystick.povLeft().onTrue(new InstantCommand(() -> S_Climber.retractClimber()));

        //joystick.leftTrigger().onTrue(new InstantCommand(() -> S_Shooter.setShooterSpeedRPS(70)));//up position (placeholder value)
        //joystick.leftTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopShooterMotor()));

       // joystick.povUp().onTrue(new InstantCommand(() -> S_Shooter.setHoodPosition(Constants.ShooterConstants.hoodRetractPosition)));
       // joystick.povRight().onTrue(new InstantCommand(() -> S_Shooter.setHoodPosition(Constants.ShooterConstants.hoodStage1Position)));
       // joystick.povDown().onTrue(new InstantCommand(() -> S_Shooter.setHoodPosition(Constants.ShooterConstants.hoodStage2Position)));


        // joystick.povDown().onTrue(new InstantCommand(() -> S_Climber.setClimberVoltage(8)));
        // joystick.povDown().onFalse(new InstantCommand(() -> S_Climber.stopClimber()));
        // joystick.povUp().onTrue(new InstantCommand(() -> S_Climber.setClimberVoltage(-8)));
        // joystick.povUp().onFalse(new InstantCommand(() -> S_Climber.stopClimber()));
        // joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Hopper.setFloorRPS(70)));
        // joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Hopper.stopFloor()));
        // joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setFeederSpeed(95)));
        // joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopFeeder()));
        // joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setHoodPosition(Constants.ShooterConstants.hoodStage2Position)));
       // joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setShooterSpeedRPS(45)));
        //joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
        
        
        
        joystick.rightTrigger().whileTrue(new Shoot(drivetrain,S_Shooter, S_Hopper));
        //joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Intake.setIntakeWheelSpeed(40)));
        //joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Intake.stopIntakeWheels()));

        //joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setShooterVoltage(2)));
        //joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
        //joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Intake.setIntakeWheelVoltage(3)));
       // joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Intake.stopIntakeWheels()));

        // joystick.leftTrigger().onTrue(new InstantCommand(() -> S_Hopper.setFloorRPS(25)));
        // joystick.leftTrigger().onFalse(new InstantCommand(() -> S_Hopper.stopFloor()));
       // joystick.leftTrigger().onTrue(new InstantCommand(() -> S_Shooter.setFeederSpeed(80)));
       // joystick.leftTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopFeeder()));
        //joystick.leftTrigger().onTrue(new InstantCommand(() -> S_Intake.setIntakeWheelVoltage(10)));
        //joystick.leftTrigger().onFalse(new InstantCommand(() -> S_Intake.stopIntakeWheels()));
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Constants.SwerveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Constants.SwerveConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.SwerveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
       // joystick.a().whileTrue(new AutoAim(drivetrain));
       // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.start().onTrue(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
       // joystick.rightTrigger().whileTrue(new AutoAim(drivetrain));

        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    //    joystick.a().whileTrue(new DriveOverBump(drivetrain, 0));
    //    joystick.b().whileTrue(new DriveOverBump(drivetrain, 1));
    //    joystick.y().whileTrue(new DriveOverBump(drivetrain, 2));
    //    joystick.x().whileTrue(new DriveOverBump(drivetrain, 3));
       joystick.y().whileTrue(new RightSideDoubleRunToMiddleBaseBLine(drivetrain, S_Intake, S_Hopper, S_Shooter, AutoPaths.ZachAndEliSpecial));
       joystick.a().whileTrue(new LeftSideDoubleRunToMiddleBaseBLine(drivetrain, S_Intake, S_Hopper, S_Shooter, AutoPaths.leftSideShallowSemiCircle));
       joystick.x().whileTrue(drivetrain.pathBuilder.build(new Path("ReturnToBump")));


       // joystick.x().whileTrue(new DriveOverBump(drivetrain, 2));
       // joystick.y().whileTrue(new DriveOverBump(drivetrain, 3));
      // joystick.y().whileTrue(new AutoDrive(drivetrain,AutoPaths.LeftSideTestCorral2.get(0) ));
       //joystick.y().onTrue(new InstantCommand(() -> S_Intake.deployIntake()));
       //joystick.y().whileTrue(new AutoDrive(drivetrain, AutoPaths.DriveToClimberLeftSide.get(0)));
       // joystick.povUp().whileTrue(new DriveOverBump(drivetrain, 2));
       // joystick.povDown().whileTrue(new DriveOverBump(drivetrain, 3));
        //joystick.a().whileTrue(new AutoDrive(drivetrain));
        //joystick.y().whileTrue(drivetrain.getAutonomousCommand());
        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.rightBumper().onTrue(new InstantCommand(() -> S_Intake.deployIntake()));
        joystick.rightBumper().onTrue(new InstantCommand(() -> S_Hopper.setFloorRPS(30)));
        joystick.leftBumper().onTrue(new InstantCommand(() -> S_Intake.retractIntake()));
        joystick.leftBumper().whileTrue(new InstantCommand(() -> S_Intake.setIntakeWheelSpeed(Constants.IntakeConstants.IntakeRPS)));
        joystick.leftBumper().onFalse(new InstantCommand(() -> S_Intake.stopIntakeWheels()));
        joystick.leftBumper().onTrue(new InstantCommand(() -> S_Hopper.stopFloor()));

        joystick.leftTrigger().whileTrue(new PassFuel(drivetrain,S_Shooter,S_Hopper,joystick));

        operatorJoystick.rightTrigger().whileTrue(new ManualShot(S_Shooter, S_Hopper, Constants.ShooterConstants.trenchShotRPS, Constants.ShooterConstants.hoodStage1Position));
        operatorJoystick.leftTrigger().whileTrue(new ManualShot(S_Shooter, S_Hopper, Constants.ShooterConstants.towerShotRPS, Constants.ShooterConstants.hoodStage1Position));
        operatorJoystick.x().whileTrue(new ManualShot(S_Shooter, S_Hopper, Constants.ShooterConstants.midwayShotRPS, Constants.ShooterConstants.hoodRetractPosition));
        operatorJoystick.leftBumper().onTrue(new InstantCommand(() -> S_Intake.stopDeployMotor()));
    }

    public Command getAutonomousCommand() {
        // String key = autoChooser.getSelected();
        // if (key == null) return Commands.none();

        // AutoDefinition auto = autos.get(key);
        // return buildAuto(auto);
        //return new LeftSideDoubleRunToMiddleBaseBLine(drivetrain, S_Intake, S_Hopper, S_Shooter, AutoPaths.leftSideDisruption);
        //return new RightSideDoubleRunToMiddleBaseBLine(drivetrain, S_Intake, S_Hopper, S_Shooter, AutoPaths.rightSideDisruption);





         //return autoChooser.getSelected();
        //return new RightSideToNeutralTwice(drivetrain, S_Intake, AutoPaths.RightSideGatherFuel2, new Pose2d(3.573, 2.579, new Rotation2d().fromDegrees(-90)));


        String key = autoChooser.getSelected();
        if (key == null) return Commands.none();

        BLineAutoDefinition auto = autos_BLine.get(key);
        return buildBLineAuto(auto);
       
    }
    private void configureAutoChooser() {
        boolean first = true;

        for (String key : autos_BLine.keySet()) {
            if (first) {
                autoChooser.setDefaultOption(key, key);
                first = false;
            } else {
                autoChooser.addOption(key, key);
            }
        }

        SmartDashboard.putData("Autonomous Mode", autoChooser);
    }
    public static Pose2d[] extractPosesFromPaths(String[] pathNames) {
        List<Pose2d> allPoses = new ArrayList<>();

        for (String name : pathNames) {
            Pose2d[] poses = extractPosesFromJson(name + ".json");

            Collections.addAll(allPoses, poses);
        }

        return allPoses.toArray(new Pose2d[0]);
    }
    public void updateAutoPreview() {
        String selectedKey = autoChooser.getSelected();
        if (selectedKey == null || selectedKey.equals(lastSelectedAuto)) {
            return;
        }

        lastSelectedAuto = selectedKey;

        Object auto = autos_BLine.get(selectedKey);
        if (auto == null) return;

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        Pose2d[] poses;

        if (auto instanceof BLineAutoDefinition bline) {

            poses = extractPosesFromPaths(bline.pathNames());
            if (bline.mirrorForLeft()) {
                for (int i = 0; i < poses.length; i++) {
                    poses[i] = AutoPaths.mirrorBlueRightToLeft(
                        poses[i],
                        Constants.FIELD_WIDTH_METERS
                    );
                }
            }

        } 
         else {
            return;
        }

        // Apply red flip
        if (alliance == Alliance.Red) {
            for (int i = 0; i < poses.length; i++) {
                poses[i] = AutoPaths.rotateBlueToRed(
                    poses[i],
                    Constants.FIELD_LENGTH_METERS,
                    Constants.FIELD_WIDTH_METERS
                );
            }
        }

        field.getObject("AutoPath").setPoses(poses);
        SmartDashboard.putData("Auto Field", field);
    }
    public void updateAutoPreview_old() {
        String selectedKey = autoChooser.getSelected();
        if (selectedKey == null) {
            return;
        }

        if (selectedKey.equals(lastSelectedAuto)) {
            return;
        }
        lastSelectedAuto = selectedKey;

        AutoDefinition auto = autos.get(selectedKey);
        if (auto == null) return;

        // Safely read alliance
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        // Extract original (blue) poses
        Pose2d[] poses = AutoPaths.extractPoses(auto.bluePaths());

        // Flip preview if red
        if (alliance == Alliance.Red) {
            for (int i = 0; i < poses.length; i++) {
                poses[i] = AutoPaths.rotateBlueToRed(
                    poses[i],
                    Constants.FIELD_LENGTH_METERS,
                    Constants.FIELD_WIDTH_METERS
                );
            }
        }

        field.getObject("AutoPath").setPoses(poses);
        SmartDashboard.putData("Auto Field", field);
    }
    private Command buildBLineAuto(BLineAutoDefinition def) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        boolean rotate = (alliance == Alliance.Red);

        List<Path> paths = buildBLinePaths(def);

        Pose2d startPose = def.blueStartPose();

        if (rotate) {
            startPose = AutoPaths.rotateBlueToRed(
                startPose,
                Constants.FIELD_LENGTH_METERS,
                Constants.FIELD_WIDTH_METERS
            );
        }

        Pose2d finalStartPose = startPose;

        return def.builder().apply(paths)
            .beforeStarting(() -> drivetrain.resetPose(finalStartPose));
    }
    private Command buildAuto(AutoDefinition def) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        List<AutoWaypoint[]> paths = def.bluePaths();
        Pose2d startPose = def.blueStartPose();

        if (alliance == Alliance.Red) {

            paths = paths.stream()
                .map(segment -> AutoPaths.rotateBlueToRed(
                    segment,
                    Constants.FIELD_LENGTH_METERS,
                    Constants.FIELD_WIDTH_METERS
                ))
                .toList();

            startPose = AutoPaths.rotateBlueToRed(
                startPose,
                Constants.FIELD_LENGTH_METERS,
                Constants.FIELD_WIDTH_METERS
            );
        }

        Pose2d finalStartPose = startPose;

        return def.builder().apply(paths)
            .beforeStarting(() -> drivetrain.resetPose(finalStartPose));
    }
    public record AutoDefinition(
        Function<List<AutoWaypoint[]>, Command> builder,
        List<AutoWaypoint[]> bluePaths,
        Pose2d blueStartPose
    ) {}
    public record BLineAutoDefinition(
        Function<List<Path>, Command> builder,
        String[] pathNames,
        Pose2d blueStartPose,
        boolean mirrorForLeft
    ) {}
    public static Pose2d[] extractPosesFromJson(String fileName) {
        List<Pose2d> poses = new ArrayList<>();

        try {
            java.nio.file.Path path = Filesystem.getDeployDirectory().toPath()
                .resolve("autos/paths")
                .resolve(fileName);

            String content = Files.readString(path);

            JSONParser parser = new JSONParser();
            JSONObject json = (JSONObject) parser.parse(content);

            JSONArray elements = (JSONArray) json.get("path_elements");

            Rotation2d lastRotation = new Rotation2d(); // default = 0 rad

            for (Object objRaw : elements) {
                JSONObject obj = (JSONObject) objRaw;

                String type = (String) obj.get("type");

                switch (type) {

                    case "waypoint": {
                        JSONObject translation = (JSONObject) obj.get("translation_target");
                        JSONObject rotation = (JSONObject) obj.get("rotation_target");

                        double x = (double) translation.get("x_meters");
                        double y = (double) translation.get("y_meters");

                        double headingRad = (double) rotation.get("rotation_radians");
                        lastRotation = new Rotation2d(headingRad);

                        poses.add(new Pose2d(x, y, lastRotation));
                        break;
                    }

                    case "translation": {
                        double x = (double) obj.get("x_meters");
                        double y = (double) obj.get("y_meters");

                        // reuse last rotation so preview stays smooth
                        poses.add(new Pose2d(x, y, lastRotation));
                        break;
                    }

                    case "rotation": {
                        double headingRad = (double) obj.get("rotation_radians");
                        lastRotation = new Rotation2d(headingRad);
                        break; // no pose added
                    }

                    default:
                        // ignore unknown types safely
                        break;
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        return poses.toArray(new Pose2d[0]);
    }
    private List<Path> buildBLinePaths(BLineAutoDefinition def) {
        List<Path> paths = new ArrayList<>();

        for (String name : def.pathNames()) {
            Path p = new Path(name);

            if (def.mirrorForLeft()) {
                p.mirror(); // mirror for LEFT side autos
            }

            paths.add(p);
        }

        return paths;
    }

    public boolean isShooterMotorsReady() {
        // return false;
        //return true; //nothing to check just return true
        return S_Shooter.isShooterMotorsReady();
    }
    public boolean isShooterHoodReady() {
        // return false;
        //return true; //nothing to do just return true
        return S_Shooter.isShooterHoodReady();
    }
    public boolean isClimberReady() {
        //return true; // no diagnostoic test for this
        return S_Climber.isClimberReady();
    }
    public boolean isGyroReady(){
        return drivetrain.isGyroReady();
        // return false;
    }
    public boolean isAprilTagCameraReady(){
        // return false;
        return drivetrain.isAprilTagCameraReady();
    }   
    public boolean isFrontLeftSwerveReady() {
        return drivetrain.isModuleReady(0);
        // return false;
    }
    public boolean isFrontRightSwerveReady() {
        return drivetrain.isModuleReady(1);
        //return false;
    }
    public boolean isBackLeftSwerveReady() {
        return drivetrain.isModuleReady(2);
        // return false;
    }
    public boolean isBackRightSwerveReady() {
        return drivetrain.isModuleReady(3);
       // return false;
    }
    public boolean isFloorReady(){
        return S_Hopper.isHopperReady();
        //return false;
    }
    public boolean isIntakeDeployReady(){
        return S_Intake.isIntakeDeployReady();
        //return true;
       }
    public boolean isIntakeWheelsReady(){
        return S_Intake.isIntakeWheelsReady();
        //return true;
       } 
}
