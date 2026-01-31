// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  
  TalonFX climberMotor;
  private DigitalInput atMaxExtention;
  boolean isLocked = false;
  Relay lockSolenoid;




  public Climber() {
    final CANBus canbus = new CANBus("rio");
    climberMotor = new TalonFX(10,canbus); //needs valid device id
    atMaxExtention = new DigitalInput(7); //needs valid channel
    var climberConfigs = new TalonFXConfiguration();
    lockSolenoid = new Relay(3);//needs valid channel

    // set slot 0 gains
    var climberMotionPIDConfigs = climberConfigs.Slot0;
    climberMotionPIDConfigs.kS = 0.0; // Add 0.25 V output to overcome static friction
    climberMotionPIDConfigs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    climberMotionPIDConfigs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    climberMotionPIDConfigs.kP = 1; // A position error of 2.5 rotations results in 12 V output
    climberMotionPIDConfigs.kI = 0; // no output for integrated error
    climberMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    var climberMotionConfigs = climberConfigs.MotionMagic;
    climberMotionConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    climberMotionConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    climberMotionConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    climberMotor.getConfigurator().apply(climberConfigs);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", getClimberPosition());
    SmartDashboard.putBoolean("Climber is at max extention", getAtMaxEtention());
  }
  public boolean getAtMaxEtention() {
    return atMaxExtention.get();
  }
  public void moveClimberToPosition(double pos){
    
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    
   // if (!isLocked) {
      climberMotor.setControl(m_request.withPosition(pos));
    //}
  }
  public double getClimberPosition(){

    return climberMotor.getPosition().getValueAsDouble();
  
  }
  public void setClimberVoltage(double voltage) {
    //if (!isLocked) {
      climberMotor.setVoltage(voltage);
    //}
  }
  public boolean getIsLocked() {
    return isLocked;
  }
  public boolean isClimberReady() {
    return climberMotor.isConnected();
  }
  public void stopClimber() {
    climberMotor.disable();
  }
}
