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
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorCommandTestBackup extends Command {

  ElevatorSubsystem m_ElevatorSubsystem;

  private double motor1position;
  private double TestMotor1PositionStart;
  private double TestMotor2PositionStart;
  private double TestMotor1Position;
  private double TestMotor2Position;
  private double motorerror;
  private double error;
  private double lastError;
  private double derivative;
  private double out;
  private double errordistance;
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  Joystick m_ElevatorJoystick;
  TalonFXConfiguration toConfigure = new TalonFXConfiguration();
  private double engagetime;
  private double slowTime;
  private boolean slowTimeCheck;
  private boolean whileBreak;
  // final TrapezoidProfile m_profile = new TrapezoidProfile(
  //  new TrapezoidProfile.Constraints(1, .025));
  // TrapezoidProfile.State m_goal = new TrapezoidProfile.State(10, 0);
  // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  // final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  /** Creates a new elevatorCommand. */
  public elevatorCommandTestBackup(ElevatorSubsystem elevatorSubsystem, double motor1position, Joystick elevatorJoystick){
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
    //m_ElevatorSubsystem.m_ElevatorMotor1.setPosition(0);//   setSelectedSensorPosition(0);
    System.out.println("Elevator Initilized");
    //toConfigure.Slot0.kG = .02539;
    //toConfigure.Slot0.kS = 0.009; // Add 0.1 V output to overcome static friction
    // toConfigure.Slot0.kV = .1825; // A velocity target of 1 rps results in 0.12 V output
    toConfigure.Slot0.kP = 87;  // An error of 1 rps results in 0.11 V output
    toConfigure.Slot0.kI = 0; // no output for integrated error
    toConfigure.Slot0.kD = 0.5; // no output for error derivative
  //-.02539
    m_ElevatorSubsystem.m_ElevatorMotor1.getConfigurator().apply(toConfigure);
    m_ElevatorSubsystem.m_TestMotor1.getConfigurator().apply(toConfigure);
    m_ElevatorSubsystem.m_TestMotor2.getConfigurator().apply(toConfigure);

    m_ElevatorSubsystem.m_TestMotor1.setNeutralMode(NeutralModeValue.Brake);
    m_ElevatorSubsystem.m_TestMotor2.setNeutralMode(NeutralModeValue.Brake);

    TestMotor1PositionStart = m_ElevatorSubsystem.m_TestMotor1.getRotorPosition().getValueAsDouble();
    TestMotor2PositionStart = m_ElevatorSubsystem.m_TestMotor2.getRotorPosition().getValueAsDouble();

    slowTimeCheck = true;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor1position = m_ElevatorSubsystem.m_ElevatorMotor1.getRotorPosition().getValueAsDouble();
    TestMotor1Position = m_ElevatorSubsystem.m_TestMotor1.getRotorPosition().getValueAsDouble();
    TestMotor2Position = m_ElevatorSubsystem.m_TestMotor2.getRotorPosition().getValueAsDouble();
    motorerror = TestMotor1Position-TestMotor1PositionStart;

    error = ((-TestMotor1Position+(-2)));

    if (errordistance <= .01 && slowTimeCheck){
      slowTime = System.currentTimeMillis();
      System.out.println("Preparing");
      SmartDashboard.putNumber("What time?", (System.currentTimeMillis() - slowTime));
      slowTimeCheck = false;
    }

    if(errordistance <= .01 && (System.currentTimeMillis() - slowTime) > 2000 ){
     // while(errordistance <= .01 && (System.currentTimeMillis() - slowTime) > 2000 ){
      m_ElevatorSubsystem.m_TestMotor1.set(-0.036328);
      m_ElevatorSubsystem.m_TestMotor2.set(-0.036328);
      System.out.println("holding");
      slowTimeCheck = true;
      errordistance = ((error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000);
      whileBreak = m_ElevatorJoystick.getRawButtonReleased(3);
      SmartDashboard.putBoolean("Joystick check", whileBreak);
      
        }
    
    else{
    System.out.println("Business as usual");
    // P = (TestMotor1Position+(-1))*.2
    // D = (error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000)

    out = (-0.042) + ((-TestMotor1Position+-2)*.08) - ((.04 * (error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000));//.115
    m_ElevatorSubsystem.m_TestMotor1.set(out);

    errordistance = ((error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000);
    SmartDashboard.putNumber("Error Difference", errordistance);
    
    lastError = ((-TestMotor1Position+(-2)));




    System.out.println();
    SmartDashboard.putNumber("Power out", out);
    // //  TestMotor1Position = m_ElevatorSubsystem.m_ElevatorMotor1.getRotorPosition().getValueAsDouble();

    // //   error = TestMotor1PositionStart - TestMotor1Position;

    // //   derivative = (error - lastError) / ((System.currentTimeMillis()-engagetime)/1000);

    // //   out = (.3) * (-TestMotor1Position+(-1)) + (0 * derivative);

    //   //m_ElevatorSubsystem.m_TestMotor1.set(out);

    //   //lastError = error;

    //   SmartDashboard.putNumber("Test Motor 1 Position", TestMotor1Position);
    //   SmartDashboard.putNumber("Test Motor 2 Position", TestMotor2Position);
    //   SmartDashboard.putNumber("Joystick Vals", (-(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5)));
    //   //System.out.println(-(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));
    //   SmartDashboard.putNumber("Power out", out);
      
    //   System.out.println(out);
    //   SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
    // }
    // else{
    //   m_ElevatorSubsystem.m_TestMotor1.set(-.042);
    // }
  
   // Static hold value == m_ElevatorSubsystem.m_TestMotor1.set(-.042);

    // if(motorerror > -1 ){
    //   m_ElevatorSubsystem.m_TestMotor1.set(((TestMotor1Position+(-1))*.2)*());
    // }
    // else{
    //   m_ElevatorSubsystem.m_TestMotor1.set(.02539);
    // }

    // m_ElevatorSubsystem.m_ElevatorMotor1.setControl(m_request.withPosition(0));

    //m_ElevatorSubsystem.m_ElevatorMotor1.set((((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));
    //toConfigure.Slot0.kP = (((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5);
    // m_ElevatorSubsystem.m_TestMotor1.set(-(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));
    // m_ElevatorSubsystem.m_TestMotor2.set(-(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));

    //m_ElevatorSubsystem.m_TestMotor1.setControl(m_request.withPosition(-2));
    //m_ElevatorSubsystem.m_TestMotor1.setControl(m_request.withPosition(-2));

    // SmartDashboard.putNumber("Motor 1 Ecoder Value", motor1position);
    SmartDashboard.putNumber("Test Motor 1 Position", TestMotor1Position);
    SmartDashboard.putNumber("Test Motor 2 Position", TestMotor2Position);
    SmartDashboard.putNumber("Joystick Vals", (-(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5)));
    //System.out.println(-(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5));
    SmartDashboard.putNumber("Power out", out);

    System.out.println(out);
    SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
    // SmartDashboard.putNumber("???", (motor1position/((System.currentTimeMillis()-engagetime)/100)));
    //m_ElevatorSubsystem.m_ElevatorMotor1.getConfigurator().apply(toConfigure);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.m_TestMotor1.set(0);
    m_ElevatorSubsystem.m_TestMotor2.set(0);
    // m_ElevatorSubsystem.m_TestMotor1.set(0);
    // m_ElevatorSubsystem.m_TestMotor2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
