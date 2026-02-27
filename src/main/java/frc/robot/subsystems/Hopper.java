// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  TalonFX floorMotor;
  

  
  /** Creates a new Hopper. */
  public Hopper() {

    final CANBus canbus = new CANBus("rio");
    floorMotor = new TalonFX(3,canbus);

    // in init function
    var floorConfigs = new TalonFXConfiguration();
    
    floorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // set slot 0 gains
    var floorMotionPIDConfigs = floorConfigs.Slot0;
    floorMotionPIDConfigs.kS = 0.4; // Add 0.25 V output to overcome static friction
    floorMotionPIDConfigs.kV = 0.1; // A velocity target of 1 rps results in 0.12 V output
    floorMotionPIDConfigs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    floorMotionPIDConfigs.kP = 0.2; // A position error of 2.5 rotations results in 12 V output
    floorMotionPIDConfigs.kI = 0; // no output for integrated error
    floorMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output


    floorMotor.getConfigurator().apply(floorConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Floor Velocity", getFloorVelocity());
  }

  public double getFloorVelocity(){

    return floorMotor.getVelocity().getValueAsDouble();
  }

  public void setFloorRPS(double rps){

    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // set velocity to 8 rps, add 0.5 V to overcome gravity
    m_request.EnableFOC = true;
    floorMotor.setControl(m_request.withVelocity(rps));
    //floorMotor.setControl(m_request.withVelocity(rpm).withFeedForward(0.5)); 
  }

  public void setFloorVoltage(double voltage) {
    floorMotor.setVoltage(voltage);
  }

  public void stopFloor(){
    floorMotor.disable();
  }
}
