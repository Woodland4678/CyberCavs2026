// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterLeadMotor;
  TalonFX shooterFollowerMotor1;
  TalonFX shooterFollowerMotor2;
  TalonFX shooterFollowerMotor3;

  InterpolatingDoubleTreeMap shooterRPSLookup = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap shooterRPSLookupStage1 = new InterpolatingDoubleTreeMap();

   InterpolatingDoubleTreeMap passingRPSLookup = new InterpolatingDoubleTreeMap();


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
    shooterRPSLookup.put(1.89, 42.0);
    shooterRPSLookup.put(2.21, 44.0);
    shooterRPSLookup.put(2.71, 48.0);
    shooterRPSLookup.put(3.1, 52.0);
    shooterRPSLookup.put(3.42, 55.2);
    shooterRPSLookup.put(3.83, 59.8);

    shooterRPSLookupStage1.put(2.34, 40.2);
    shooterRPSLookupStage1.put(2.64, 43.2);
    shooterRPSLookupStage1.put(3.08, 45.8);
    shooterRPSLookupStage1.put(3.42, 47.2);
    shooterRPSLookupStage1.put(3.90, 50.4);
    shooterRPSLookupStage1.put(4.26, 52.4);
    shooterRPSLookupStage1.put(4.58, 55.0);
    
    // shooterRPSLookupStage1.put(2.21, 44.0);
    // shooterRPSLookupStage1.put(2.71, 48.0);
    // shooterRPSLookupStage1.put(3.1, 52.0);
    // shooterRPSLookupStage1.put(3.42, 55.2);
    // shooterRPSLookupStage1.put(3.83, 59.8);

    passingRPSLookup.put(3.83, 30.8);
    passingRPSLookup.put(6.14, 40.0);
    passingRPSLookup.put(8.0, 57.8);
    passingRPSLookup.put(9.52, 65.0);
    passingRPSLookup.put(9.52, 65.0);
    passingRPSLookup.put(12.5, 85.8);
    passingRPSLookup.put(14.95, 95.0);
    final CANBus canbus = new CANBus("rio");

    leftFuelSensor = new DigitalInput(8); //needs valid channel ???
    leftFuelSensor = new DigitalInput(9); //needs valid channel ???
    leftFuelSensor = new DigitalInput(10); //needs valid channel ???

    shooterLeadMotor = new TalonFX(5,canbus); //needs valid device id ???
    shooterFollowerMotor1 = new TalonFX(6,canbus);
    shooterFollowerMotor2= new TalonFX(7,canbus);
    shooterFollowerMotor3 = new TalonFX(8,canbus);
    
    //Aligned vs opposed needs to be determined 
    shooterFollowerMotor1.setControl(new Follower(shooterLeadMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    shooterFollowerMotor2.setControl(new Follower(shooterLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    shooterFollowerMotor3.setControl(new Follower(shooterLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
      
    var shooterConfigs = new TalonFXConfiguration();
    var feederConfigs = new TalonFXConfiguration();

    //For Velocity Voltage
    var shooterMotionPIDConfigs = shooterConfigs.Slot0;
    shooterMotionPIDConfigs.kS = 0.1; // Add 0.25 V output to overcome static friction
    shooterMotionPIDConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    shooterMotionPIDConfigs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    shooterMotionPIDConfigs.kP = 0.3; // A position error of 2.5 rotations results in 12 V output
    shooterMotionPIDConfigs.kI = 0; // no output for integrated error
    shooterMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    // For VelocityTorqueCurrentFOC
    var shooterMotionPIDConfigs2 = shooterConfigs.Slot1;
    shooterMotionPIDConfigs2.kS = 1.4; // Add 0.25 V output to overcome static friction
    shooterMotionPIDConfigs2.kV = 0.009; // A velocity target of 1 rps results in 0.12 V output
    shooterMotionPIDConfigs2.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    shooterMotionPIDConfigs2.kP = 5.0; // A position error of 2.5 rotations results in 12 V output
    shooterMotionPIDConfigs2.kI = 0; // no output for integrated error
    shooterMotionPIDConfigs2.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output


    var shooterMotionConfigs = shooterConfigs.MotionMagic;
    shooterMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    shooterMotionConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    shooterMotionConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    currentConfigs.StatorCurrentLimit = 100;
    currentConfigs.StatorCurrentLimitEnable = true;
    shooterConfigs.withCurrentLimits(currentConfigs);

    shooterLeadMotor.getConfigurator().apply(shooterConfigs);
    shooterFollowerMotor1.getConfigurator().apply(shooterConfigs);
    shooterFollowerMotor2.getConfigurator().apply(shooterConfigs);
    shooterFollowerMotor3.getConfigurator().apply(shooterConfigs);

    feederMotor = new TalonFX(9,canbus); //needs valid device id ???

    var feederMotionPIDConfigs = feederConfigs.Slot0;
    feederMotionPIDConfigs.kS = 0.1; // Add 0.25 V output to overcome static friction
    feederMotionPIDConfigs.kV = 0.102; // A velocity target of 1 rps results in 0.12 V output
    feederMotionPIDConfigs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    feederMotionPIDConfigs.kP = 0.2; // A position error of 2.5 rotations results in 12 V output
    feederMotionPIDConfigs.kI = 0; // no output for integrated error
    feederMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    var feederMotionConfigs = feederConfigs.MotionMagic;
    feederMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    feederMotionConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    feederMotionConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)
    feederConfigs.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(60));
    feederMotor.getConfigurator().apply(feederConfigs);
    
     hoodMotor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless); // needs valid device id ???
     hoodMotorController = hoodMotor.getClosedLoopController();

     hoodMotorConfig = new SparkMaxConfig();

     hoodMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.2)
      .i(0)
      .d(0)
      .outputRange(-1, 1);

      hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      feederMotor.setNeutralMode(NeutralModeValue.Brake);
    // hoodPositions = new Dictionary<String,Double>() {
      
    // };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter RPS", getShooterSpeedRPS());
    SmartDashboard.putNumber("Hood Encoder Position", getHoodPosition());
    SmartDashboard.putNumber("Feeder RPS", getFeederSpeed());
    SmartDashboard.putNumber("Feeder torque current", feederMotor.getTorqueCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Feeder supply current", feederMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Feeder torque current", feederMotor.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter lead motor supply current", shooterLeadMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter follower1 motor supply current", shooterFollowerMotor1.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter follower2 motor supply current", shooterFollowerMotor2.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter follower3 motor supply current", shooterFollowerMotor3.getSupplyCurrent().getValueAsDouble());
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
    final VelocityTorqueCurrentFOC m_requestFOC = new VelocityTorqueCurrentFOC(0).withSlot(1);

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
  public void setShooterVoltage(double voltage) {
    shooterLeadMotor.setVoltage(voltage);
  }
  // public void findFeederTargetSpeed(double distance) {}
  public double getDesiredShooterRPS(double distance, int hoodPos) {
    if (hoodPos == 0) {
      setHoodPosition(Constants.ShooterConstants.hoodRetractPosition);
      return shooterRPSLookup.get(distance);
    } 
    else if (hoodPos == 1) {
      setHoodPosition(Constants.ShooterConstants.hoodStage1Position);
      return shooterRPSLookupStage1.get(distance);
    }
    return shooterRPSLookup.get(distance);
  }
  public void setHoodPosition(double position) {

   

    // switch(position) {
    //   case LOW:
    //     pos = 0;
    //     break;
    //   case MEDIUM:
    //     pos = 0;
    //     break;
    //   case HIGH:
    //     pos = 0;
    //    break;
    //   default:
    //     pos = 0;
    //     break;
    // }

    hoodMotorController.setSetpoint(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);

  }

  public double getHoodPosition() {
    return hoodMotor.getEncoder().getPosition();
  }
  public boolean isShooterMotorsReady() {
    return (shooterLeadMotor.isConnected() && shooterFollowerMotor1.isConnected() && shooterFollowerMotor2.isConnected() && shooterFollowerMotor3.isConnected());
  }
  public boolean isShooterHoodReady() {
    return !hoodMotor.hasActiveFault();
  }
  public double getShooterPassRPS(double dist) {
    return passingRPSLookup.get(dist);
  }
  // public void findTargetHoodPosition(double position) {}

  // public void detectFuel(int lane, double lastReading) {}

  // public int getFuelInLane(int lane) {}

  // public int getTotalFuel() {}

  // public bool isFuelInLane(int lane) {}

  // public bool IsFuelInShooter() {}
}
