// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterLeadMotor;
  TalonFX shooterFollowerMotor;

  TalonFX feederMotor;

  SparkMax hoodMotor;

  private SparkMaxConfig hoodMotorConfig;
  private SparkClosedLoopController hoodMotorController;

  public static enum HoodPosition {
    LOW,
    MEDIUM,
    HIGH
  }

  String currentHoodPosition;

  int slot = 0;
  DigitalInput leftFuelSensor;
  DigitalInput centreFuelSensor;
  DigitalInput rightFuelSensor;

  public Shooter() {
    final CANBus canbus = new CANBus("DriveTrain");

    leftFuelSensor = new DigitalInput(8); //needs valid channel ???
    leftFuelSensor = new DigitalInput(9); //needs valid channel ???
    leftFuelSensor = new DigitalInput(10); //needs valid channel ???

    shooterLeadMotor = new TalonFX(20,canbus); //needs valid device id ???
    shooterFollowerMotor = new TalonFX(21,canbus);
    
    shooterFollowerMotor.setControl(new Follower(shooterLeadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
      
    var shooterConfigs = new TalonFXConfiguration();
    var feederConfigs = new TalonFXConfiguration();

    //For Velocity Voltage
    var shooterMotionPIDConfigs = shooterConfigs.Slot0;
    shooterMotionPIDConfigs.kS = 0.2; // Add 0.25 V output to overcome static friction
    shooterMotionPIDConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    shooterMotionPIDConfigs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    shooterMotionPIDConfigs.kP = 0.175; // A position error of 2.5 rotations results in 12 V output
    shooterMotionPIDConfigs.kI = 0; // no output for integrated error
    shooterMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    // For VelocityTorqueCurrentFOC
    var shooterMotionPIDConfigs2 = shooterConfigs.Slot1;
    shooterMotionPIDConfigs2.kS = 0.0; // Add 0.25 V output to overcome static friction
    shooterMotionPIDConfigs2.kV = 0.057; // A velocity target of 1 rps results in 0.12 V output
    shooterMotionPIDConfigs2.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    shooterMotionPIDConfigs2.kP = 6.0; // A position error of 2.5 rotations results in 12 V output
    shooterMotionPIDConfigs2.kI = 0; // no output for integrated error
    shooterMotionPIDConfigs2.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output


    var shooterMotionConfigs = shooterConfigs.MotionMagic;
    shooterMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    shooterMotionConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    shooterMotionConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    shooterLeadMotor.getConfigurator().apply(shooterConfigs);

    feederMotor = new TalonFX(22,canbus); //needs valid device id ???

    var feederMotionPIDConfigs = feederConfigs.Slot0;
    feederMotionPIDConfigs.kS = 0.0; // Add 0.25 V output to overcome static friction
    feederMotionPIDConfigs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    feederMotionPIDConfigs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    feederMotionPIDConfigs.kP = 1; // A position error of 2.5 rotations results in 12 V output
    feederMotionPIDConfigs.kI = 0; // no output for integrated error
    feederMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    var feederMotionConfigs = feederConfigs.MotionMagic;
    feederMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    feederMotionConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    feederMotionConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    feederMotor.getConfigurator().apply(feederConfigs);
    
     hoodMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless); // needs valid device id ???
     hoodMotorController = hoodMotor.getClosedLoopController();

     hoodMotorConfig = new SparkMaxConfig();

     hoodMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-1, 1);

      hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // hoodPositions = new Dictionary<String,Double>() {
      
    // };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter RPS", getShooterSpeedRPS());
  }

  public void setShooterSpeedRPS(double rps) {
    // create a velocity closed-loop request, voltage output, slot 0 configs
    
    // if (Math.abs(getShooterSpeedRPS() - rps) < 1.0) {
    //   slot = 1;
    // }
    final VelocityVoltage m_requestFOC = new VelocityVoltage(0).withSlot(0);
   
    m_requestFOC.EnableFOC = true;

    shooterLeadMotor.setControl(m_requestFOC.withVelocity(rps));
  }
  public void setShooterRPSTorqueFOC(double rps) {
    final VelocityTorqueCurrentFOC m_requestFOC = new VelocityTorqueCurrentFOC(0).withSlot(0);

    shooterLeadMotor.setControl(m_requestFOC.withVelocity(rps));
  }

  public double getShooterSpeedRPS() {
    return shooterLeadMotor.getVelocity().getValueAsDouble();
  }

  public void stopShooterMotor(){
    shooterLeadMotor.disable();
  }

  // public void findShooterTargetSpeed(double distance) {}

  public void setFeederSpeed(double rps) {
    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // set velocity to 8 rps, add 0.5 V to overcome gravity
    m_request.EnableFOC = true;
    feederMotor.setControl(m_request.withVelocity(rps));
  }

  public double getFeederSpeed() {
    return feederMotor.getVelocity().getValueAsDouble();
  }

  public void stopFeeder() {
    feederMotor.disable();
  }

  public void setFeederVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  public void setShooterPIDSlot(int slot) {
  
  }

  // public void findFeederTargetSpeed(double distance) {}

  public void setHoodPosition(HoodPosition position) {

    double pos;

    switch(position) {
      case LOW:
        pos = 0;
        break;
      case MEDIUM:
        pos = 0;
        break;
      case HIGH:
        pos = 0;
       break;
      default:
        pos = 0;
        break;
    }

    hoodMotorController.setSetpoint(pos, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);

  }

  public double getHoodPosition() {
    return hoodMotorController.getSetpoint();
  }

  // public void findTargetHoodPosition(double position) {}

  // public void detectFuel(int lane, double lastReading) {}

  // public int getFuelInLane(int lane) {}

  // public int getTotalFuel() {}

  // public bool isFuelInLane(int lane) {}

  // public bool IsFuelInShooter() {}
}
