// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX shooterLeadMotor;
  TalonFX shooterFollowerMotor;

  TalonFX feederMotor;

  SparkMax hoodMotor;

  Dictionary<String, Double> hoodPositions;

  String currentHoodPosition;

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

    var shooterMotionPIDConfigs = shooterConfigs.Slot0;
    shooterMotionPIDConfigs.kS = 0.19; // Add 0.25 V output to overcome static friction
    shooterMotionPIDConfigs.kV = 0.1; // A velocity target of 1 rps results in 0.12 V output
    shooterMotionPIDConfigs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    shooterMotionPIDConfigs.kP = 0.0; // A position error of 2.5 rotations results in 12 V output
    shooterMotionPIDConfigs.kI = 0; // no output for integrated error
    shooterMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output


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

    // hoodMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless); // needs valid device id ???

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
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    final TorqueCurrentFOC m_requestFOC = new TorqueCurrentFOC(0);
    // set velocity to 8 rps, add 0.5 V to overcome gravity
    //shooterLeadMotor.setControl(m_request.withVelocity(rps));
    shooterLeadMotor.setControl(m_requestFOC);
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

  // public void findFeederTargetSpeed(double distance) {}

  // public void setHoodPosition(string position) {}

  // public string getHoodPosition() {}

  // public void findTargetHoodPosition(double position) {}

  // public void detectFuel(int lane, double lastReading) {}

  // public int getFuelInLane(int lane) {}

  // public int getTotalFuel() {}

  // public bool isFuelInLane(int lane) {}

  // public bool IsFuelInShooter() {}
}
