// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorCommandTest extends Command {

  ElevatorSubsystem m_ElevatorSubsystem;

  private double TestMotor1Position;
  private double TestMotor2Position;
  private double error;
  private double lastError;
  private double out;
  private double errordistance;
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  Joystick m_ElevatorJoystick;
  TalonFXConfiguration toConfigure = new TalonFXConfiguration();
  private double engagetime;
  private double slowTime;
  private boolean slowTimeCheck;
  private double elevatorEncoderValueCheck;
  private double elevatorEncoderValue;
  Joystick m_TestJoystick;

  /** Creates a new elevatorCommand. */
  public elevatorCommandTest(ElevatorSubsystem elevatorSubsystem, double motor1position, Joystick elevatorJoystick, Joystick testJoystick){
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);

    m_ElevatorJoystick = elevatorJoystick;
    m_TestJoystick = testJoystick;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    engagetime = System.currentTimeMillis();
    System.out.println("Elevator Initilized");
    // toConfigure.Slot0.kP = 87;  // An error of 1 rps results in 0.11 V output
    // toConfigure.Slot0.kI = 0; // no output for integrated error
    // toConfigure.Slot0.kD = 0.5; // no output for error derivative
    // m_ElevatorSubsystem.m_TestMotor1.getConfigurator().apply(toConfigure);
    // m_ElevatorSubsystem.m_TestMotor2.getConfigurator().apply(toConfigure);

    m_ElevatorSubsystem.m_TestMotor1.setNeutralMode(NeutralModeValue.Brake);
    m_ElevatorSubsystem.m_TestMotor2.setNeutralMode(NeutralModeValue.Brake);

    slowTimeCheck = true;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TestMotor1Position = m_ElevatorSubsystem.m_TestMotor1.getRotorPosition().getValueAsDouble();
    TestMotor2Position = m_ElevatorSubsystem.m_TestMotor2.getRotorPosition().getValueAsDouble();
    elevatorEncoderValue = Robot.elevatorEncoder.get();

    error = ((TestMotor1Position+(-4)));

    if (errordistance <= .01 && slowTimeCheck){
      slowTime = System.currentTimeMillis();
      System.out.println("Preparing");
      SmartDashboard.putNumber("What time?", (System.currentTimeMillis() - slowTime));
      slowTimeCheck = false;
    }

    if(errordistance <= .01 && (System.currentTimeMillis() - slowTime) > 20000 ){
      m_ElevatorSubsystem.m_TestMotor1.set(0.0345);
      m_ElevatorSubsystem.m_TestMotor2.set(0.0345);
      System.out.println("holding");
      slowTimeCheck = true;
      errordistance = ((error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000);   
        }
    
    else{
    System.out.println("Business as usual");

    out = -(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5);
    //out = (0) + (((((-elevatorEncoderValue)/1000+-4)))* .2); // ((0.000001 * (((-elevatorEncoderValue)/1000) - ((-elevatorEncoderValueCheck)/1000))) / ((System.currentTimeMillis()-engagetime)/1000));
    m_ElevatorSubsystem.m_TestMotor1.set(-out);
    m_ElevatorSubsystem.m_TestMotor2.set(-out);
    

    //out = (-0.042) + (-(java.lang.Math.sqrt((-TestMotor1Position+2)))* .05) - ((0.001 * (error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000))

    errordistance = ((error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000);
    SmartDashboard.putNumber("Error Difference", errordistance);

    TestMotor1Position = m_ElevatorSubsystem.m_TestMotor1.getRotorPosition().getValueAsDouble();
    
    lastError = ((TestMotor1Position+(-4)));
    elevatorEncoderValueCheck = Robot.elevatorEncoder.get();
    
    SmartDashboard.putNumber("D", (((-elevatorEncoderValue)/1000) - ((-elevatorEncoderValueCheck)/1000)) / ((System.currentTimeMillis()-engagetime)/1000));
    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoderValue);
    SmartDashboard.putNumber("Power out", out);
    SmartDashboard.putNumber("Test Motor 1 Position", TestMotor1Position);
    SmartDashboard.putNumber("Test Motor 2 Position", TestMotor2Position);
    SmartDashboard.putNumber("Joystick Vals", ((error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000));
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("lastError", lastError);
    SmartDashboard.putNumber("Power out", out);
    System.out.println(out);
    SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
    SmartDashboard.putNumber("Elvator Voltage", m_ElevatorSubsystem.m_TestMotor1.getMotorVoltage().getValueAsDouble());
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.m_TestMotor1.set(0.0);
    m_ElevatorSubsystem.m_TestMotor2.set(0.0);
    // m_ElevatorSubsystem.m_TestMotor1.set(0);
    // m_ElevatorSubsystem.m_TestMotor2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
