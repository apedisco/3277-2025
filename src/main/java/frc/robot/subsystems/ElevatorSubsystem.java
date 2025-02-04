// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  public TalonFX m_TestMotor1 = new TalonFX(11);
  public TalonFX m_TestMotor2 = new TalonFX(12);
  TalonFXConfiguration toConfigure = new TalonFXConfiguration();
  public int MasterID = 9;
  public boolean OpposeMasterDirection = false;
  public Follower motor2 = new Follower(MasterID, OpposeMasterDirection);
  public double TestMotor1Position;
  public double TestMotor2Position;
  //public StatusCode setControl(Follower request);
  
  
  

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // m_TestMotor1.setInverted(true);
    // m_TestMotor2.setInverted(true);
    m_TestMotor1.setPosition(0);
    m_TestMotor2.setPosition(0);
    

  

  toConfigure.Slot0.kS = 0.00; // Add 0.1 V output to overcome static friction
  toConfigure.Slot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
  toConfigure.Slot0.kP = .11; // An error of 1 rps results in 0.11 V output
  toConfigure.Slot0.kI = 0; // no output for integrated error
  toConfigure.Slot0.kD = 0.1; // no output for error derivative

  // m_ElevatorMotor1.getConfigurator().apply(toConfigure);

  TestMotor1Position = m_TestMotor1.getRotorPosition().getValueAsDouble();
  TestMotor2Position = m_TestMotor2.getRotorPosition().getValueAsDouble();
  SmartDashboard.putNumber("Test Motor 1 Position", TestMotor1Position);
  SmartDashboard.putNumber("Test Motor 2 Position", TestMotor2Position);
  }
  public void elevatorVoltageOut(){
    m_TestMotor1.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
