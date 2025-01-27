// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorCommand extends Command {

  ElevatorSubsystem m_ElevatorSubsystem;
  private double motor1position;
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  Joystick m_ElevatorJoystick;
  TalonFXConfiguration toConfigure = new TalonFXConfiguration();
  private double engagetime;
  // final TrapezoidProfile m_profile = new TrapezoidProfile(
  //  new TrapezoidProfile.Constraints(1, .025));
  // TrapezoidProfile.State m_goal = new TrapezoidProfile.State(10, 0);
  // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  // final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  /** Creates a new elevatorCommand. */
  public elevatorCommand(ElevatorSubsystem elevatorSubsystem, double motor1position, Joystick elevatorJoystick) {
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);

    m_ElevatorJoystick = elevatorJoystick;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

// calculate the next profile setpoint
// m_setpoint = m_profile.calculate(5, m_setpoint, m_goal);

// // send the request to the device
// m_request.Position = m_setpoint.position;
// m_request.Velocity = m_setpoint.velocity;

    engagetime = System.currentTimeMillis();
    m_ElevatorSubsystem.m_ElevatorMotor1.setPosition(0);//   setSelectedSensorPosition(0);
    motor1position = m_ElevatorSubsystem.m_ElevatorMotor1.getRotorPosition().getValueAsDouble();
    System.out.println("Elevator Initilized");
    //toConfigure.Slot0.kS = 0.009; // Add 0.1 V output to overcome static friction
    //toConfigure.Slot0.kV = .1825; // A velocity target of 1 rps results in 0.12 V output
    toConfigure.Slot0.kP = 4; // An error of 1 rps results in 0.11 V output
    toConfigure.Slot0.kI = 0; // no output for integrated error
    toConfigure.Slot0.kD = .52; // no output for error derivative
  
    m_ElevatorSubsystem.m_ElevatorMotor1.getConfigurator().apply(toConfigure);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor1position = m_ElevatorSubsystem.m_ElevatorMotor1.getRotorPosition().getValueAsDouble();

    // m_ElevatorSubsystem.m_ElevatorMotor1.setControl(m_request.withPosition(10));

    //m_ElevatorSubsystem.m_ElevatorMotor1.set((((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));
    //toConfigure.Slot0.kP = (((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5);

    //m_ElevatorSubsystem.m_ElevatorMotor1.set(.1);

    SmartDashboard.putNumber("Motor 1 Ecoder Value", motor1position);
    SmartDashboard.putNumber("Joystick Vals", (((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));
    SmartDashboard.putNumber("Timer", (System.currentTimeMillis()-engagetime));
    SmartDashboard.putNumber("???", (motor1position/((System.currentTimeMillis()-engagetime)/100)));
    //m_ElevatorSubsystem.m_ElevatorMotor1.getConfigurator().apply(toConfigure);
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
