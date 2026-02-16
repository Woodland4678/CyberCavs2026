// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  SparkMax deployMotor;
  private SparkMaxConfig deployMotorConfig;
  private SparkClosedLoopController deployMotorController;

  TalonFX intakeWheels;
  private boolean isIntakeDeployed;

  /** Creates a new Intake. */
  public Intake() {

    final CANBus canbus = new CANBus("rio");
    intakeWheels = new TalonFX(2,canbus);

    deployMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless); // needs valid device id ???
    deployMotorController = deployMotor.getClosedLoopController();

    deployMotorConfig = new SparkMaxConfig();

     deployMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      // Set PID values for position control. We don't need to pass a closed
      // loop slot, as it will default to slot 0.
      .p(0.1)
      .i(0)
      .d(0)
      .outputRange(-1, 1);

      deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // in init function
    var intakeWheelsConfigs = new TalonFXConfiguration();
    
    intakeWheelsConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // set slot 0 gains
    var intakeWheelsMotionPIDConfigs = intakeWheelsConfigs.Slot0;
    intakeWheelsMotionPIDConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    intakeWheelsMotionPIDConfigs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    intakeWheelsMotionPIDConfigs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    intakeWheelsMotionPIDConfigs.kP = 50; // A position error of 2.5 rotations results in 12 V output
    intakeWheelsMotionPIDConfigs.kI = 0; // no output for integrated error
    intakeWheelsMotionPIDConfigs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output


    intakeWheels.getConfigurator().apply(intakeWheelsConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Deploy Status = ", isIntakeDeployed());
    SmartDashboard.putNumber("IntakeWheel Speed", getIntakeWheelSpeed());


  }

  public void moveIntake(double pos){
    
    deployMotorController.setSetpoint(pos, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void deployIntake(){
    isIntakeDeployed = true;
    moveIntake(1);
  }

  
  public void retractIntake(){
    isIntakeDeployed = false;
    moveIntake(0);
  }

  public boolean isIntakeDeployed(){
    return isIntakeDeployed;

  }

  public void setIntakeWheelSpeed(double rps){
    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    m_request.EnableFOC = true;
    intakeWheels.setControl(m_request.withVelocity(rps));
    //floorMotor.setControl(m_request.withVelocity(rpm).withFeedForward(0.5));

  }

  public double getIntakeWheelSpeed(){
    return intakeWheels.getVelocity().getValueAsDouble();
  }

  public void stopIntakeWheels(){
    intakeWheels.disable();
  }
}
