// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  
  SparkMax climberMotor;

  private SparkMaxConfig climberMotorConfig;
  private SparkClosedLoopController climberMotorController;


  private DigitalInput atMaxExtention;

  public Climber() {
    //final CANBus canbus = new CANBus("rio");

    climberMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushless); // needs valid device id ???
    climberMotorController = climberMotor.getClosedLoopController();

    climberMotorConfig = new SparkMaxConfig();

     climberMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-1, 1);

      climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
    climberMotorController.setSetpoint(pos, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public double getClimberPosition(){

    return climberMotor.getEncoder().getPosition();
  
  }
  // public boolean isClimberReady() {
  //   return climberMotor.isConnected();
  // }
  public void stopClimber() {
    climberMotor.disable();
  }
  public void extendClimber() {
    moveClimberToPosition(Constants.ClimberConstants.extendPosition);
  }
  public void retractClimber() {
    moveClimberToPosition(Constants.ClimberConstants.retractPosition);
  }
}
