// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.autos.LeftSideHubSweepCorralClimb;
import frc.robot.autos.LeftSideMiddleCorralClimb;
import frc.robot.autos.LeftSideSweepWallCorralClimb;
import frc.robot.autos.LeftSideToNeutralTwice;
import frc.robot.autos.RightSideNeutralTwiceWithLoop;
import frc.robot.autos.RightSideToNeutralTwice;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveOverBump;
import frc.robot.commands.PassFuel;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
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

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    public final Hopper S_Hopper = new Hopper();
    public final Climber S_Climber = new Climber();
    public final Shooter S_Shooter = new Shooter();
    public final Intake S_Intake = new Intake();
    private final Map<String, AutoDefinition> autos = Map.of(
        "RightSideToNeutralTwice",
        new AutoDefinition(
            paths -> new RightSideToNeutralTwice(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
            AutoPaths.RightSideGatherFuel1,
            new Pose2d(3.573, 2.579, Rotation2d.fromDegrees(90))
        ),

         "LeftSideToNeutralTwice",
        new AutoDefinition(
            paths -> new LeftSideToNeutralTwice(drivetrain, paths),
            AutoPaths.LeftSideGatherFuel1,
            new Pose2d(3.573, 5.3633, Rotation2d.fromDegrees(-90))
        ),
        "RightSideToNeutralTwiceWithLoop",
        new AutoDefinition(
            paths -> new RightSideNeutralTwiceWithLoop(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
            AutoPaths.RightSideGatherFuelWithLoop,
            new Pose2d(3.573, 2.579, Rotation2d.fromDegrees(90))
        ),
        "LeftSideMiddleOnceCorralClimb",
        new AutoDefinition(
            paths -> new LeftSideMiddleCorralClimb(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
            AutoPaths.LeftSideMiddleCorralClimbPaths,
            new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
        ),
        "LeftSideHubSweepCorralClimb",
        new AutoDefinition(
            paths -> new LeftSideHubSweepCorralClimb(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
            AutoPaths.LeftSideHubSweepCorralClimb,
            new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
        ),
         "LeftSideSweepWallCorralClimb",
        new AutoDefinition(
            paths -> new LeftSideSweepWallCorralClimb(drivetrain, S_Intake, S_Hopper, S_Shooter, paths),
            AutoPaths.LeftSideSweepWallCorral,
            new Pose2d(3.599, 5.620, Rotation2d.fromDegrees(-90))
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
        // joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setFeederSpeed(80)));
        // joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopFeeder()));
        // joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Shooter.setShooterSpeedRPS(45)));
        // joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
        joystick.rightTrigger().whileTrue(new Shoot(drivetrain,S_Shooter, S_Hopper));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> S_Intake.setIntakeWheelSpeed(60)));
        joystick.rightTrigger().onFalse(new InstantCommand(() -> S_Intake.stopIntakeWheels()));

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
        joystick.a().whileTrue(new AutoAim(drivetrain));
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

        joystick.start().onTrue(new InstantCommand(() -> S_Shooter.stopShooterMotor()));
       // joystick.rightTrigger().whileTrue(new AutoAim(drivetrain));

        // Reset the field-centric heading on left bumper press.
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

       // joystick.x().whileTrue(new DriveOverBump(drivetrain, 2));
       // joystick.y().whileTrue(new DriveOverBump(drivetrain, 3));
       joystick.y().whileTrue(new AutoDrive(drivetrain,AutoPaths.rotateBlueToRed(AutoPaths.LeftSideTestCorral.get(0), Constants.FIELD_LENGTH_METERS, Constants.FIELD_WIDTH_METERS) ));
       joystick.y().onTrue(new InstantCommand(() -> S_Intake.deployIntake()));
       //joystick.y().whileTrue(new AutoDrive(drivetrain, AutoPaths.DriveToClimberLeftSide.get(0)));
       // joystick.povUp().whileTrue(new DriveOverBump(drivetrain, 2));
       // joystick.povDown().whileTrue(new DriveOverBump(drivetrain, 3));
        //joystick.a().whileTrue(new AutoDrive(drivetrain));
        //joystick.y().whileTrue(drivetrain.getAutonomousCommand());
        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.rightBumper().onTrue(new InstantCommand(() -> S_Intake.deployIntake()));
        joystick.rightBumper().onTrue(new InstantCommand(() -> S_Hopper.setFloorRPS(45)));
        joystick.leftBumper().onTrue(new InstantCommand(() -> S_Intake.retractIntake()));
        joystick.leftBumper().onTrue(new InstantCommand(() -> S_Hopper.stopFloor()));

        joystick.leftTrigger().whileTrue(new PassFuel(drivetrain,S_Shooter,S_Hopper,joystick));
    }

    public Command getAutonomousCommand() {
        String key = autoChooser.getSelected();
        if (key == null) return Commands.none();

        AutoDefinition auto = autos.get(key);
        return buildAuto(auto);
         //return autoChooser.getSelected();
        //return new RightSideToNeutralTwice(drivetrain, S_Intake, AutoPaths.RightSideGatherFuel2, new Pose2d(3.573, 2.579, new Rotation2d().fromDegrees(-90)));
       
    }
    private void configureAutoChooser() {
        boolean first = true;

        for (String key : autos.keySet()) {
            if (first) {
                autoChooser.setDefaultOption(key, key);
                first = false;
            } else {
                autoChooser.addOption(key, key);
            }
        }

        SmartDashboard.putData("Autonomous Mode", autoChooser);
    }
    public void updateAutoPreview() {
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
