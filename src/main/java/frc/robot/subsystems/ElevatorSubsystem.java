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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  
  public TalonFX m_ElevatorMotor1 = new TalonFX(9);
  public TalonFX m_ElevatorMotor2 = new TalonFX(10);
  TalonFXConfiguration toConfigure = new TalonFXConfiguration();
  public int MasterID = 9;
  public boolean OpposeMasterDirection = false;
  private double followerUpdateFreq = 1000;
  public Follower mootor2 = new Follower(MasterID, OpposeMasterDirection);
  //public StatusCode setControl(Follower request);
  
  
  

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

  

  toConfigure.Slot0.kS = 0.00; // Add 0.1 V output to overcome static friction
  toConfigure.Slot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
  toConfigure.Slot0.kP = .11; // An error of 1 rps results in 0.11 V output
  toConfigure.Slot0.kI = 0; // no output for integrated error
  toConfigure.Slot0.kD = 0.1; // no output for error derivative

  // m_ElevatorMotor1.getConfigurator().apply(toConfigure);
  m_ElevatorMotor2.getConfigurator().apply(toConfigure);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
