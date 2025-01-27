// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestElevator extends Command {
  ElevatorSubsystem m_ElevatorSubsystem;
  Joystick m_ElevatorJoystick;
  /** Creates a new TestElevator. */
  public TestElevator(ElevatorSubsystem elevatorSubsystem, Joystick joystick) {
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
    m_ElevatorJoystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.m_TestMotor1.setNeutralMode(NeutralModeValue.Brake);
    m_ElevatorSubsystem.m_TestMotor2.setNeutralMode(NeutralModeValue.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ElevatorSubsystem.m_TestMotor1.set((((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.3));
    m_ElevatorSubsystem.m_TestMotor2.set((((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.3));
    SmartDashboard.putNumber("TestMotor1 Position", m_ElevatorSubsystem.m_TestMotor1.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("TestMotor2 Positon", m_ElevatorSubsystem.m_TestMotor2.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Joystick out", -(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.m_TestMotor1.set(0);
    m_ElevatorSubsystem.m_TestMotor2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
