// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorPIDCommand extends Command {

  ElevatorSubsystem m_ElevatorSubsystem;

  private double TestMotor1PositionStart;
  private double TestMotor2PositionStart; // Might not need if follower is properly used???
  private double engagetime;
  private double setPoint;
  private double TestMotor1Position;
  private double TestMotor2Position; // Might not need if follower is properly used???
  private double error;
  private double errordistance;
  private double slowTime;
  private double lastError;
  private double P;
  private double D;
  private double out;

  TalonFXConfiguration toConfigure = new TalonFXConfiguration();

  private boolean slowTimeCheck;

  /** Creates a new elevatorPIDCommand. */
  public elevatorPIDCommand(ElevatorSubsystem elevatorSubsystem) {
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    toConfigure.Slot0.kP = 0;  // An error of 1 rps results in 0.11 V output
    toConfigure.Slot0.kI = 0; // no output for integrated error
    toConfigure.Slot0.kD = 0;

    m_ElevatorSubsystem.m_TestMotor1.setNeutralMode(NeutralModeValue.Brake);
    m_ElevatorSubsystem.m_TestMotor2.setNeutralMode(NeutralModeValue.Brake);
    TestMotor1PositionStart = m_ElevatorSubsystem.m_TestMotor1.getRotorPosition().getValueAsDouble();
    TestMotor2PositionStart = m_ElevatorSubsystem.m_TestMotor2.getRotorPosition().getValueAsDouble();
    m_ElevatorSubsystem.m_TestMotor1.getConfigurator().apply(toConfigure);
    m_ElevatorSubsystem.m_TestMotor2.getConfigurator().apply(toConfigure);
    // Gets the value of the encoder when the command is initilized

    engagetime = System.currentTimeMillis();

    setPoint = (-2);

    slowTimeCheck = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //motor1position = m_ElevatorSubsystem.m_ElevatorMotor1.getRotorPosition().getValueAsDouble();
    TestMotor1Position = m_ElevatorSubsystem.m_TestMotor1.getRotorPosition().getValueAsDouble();
    TestMotor2Position = m_ElevatorSubsystem.m_TestMotor2.getRotorPosition().getValueAsDouble();
    //motorerror = TestMotor1Position-TestMotor1PositionStart;

    error = ((-TestMotor1Position+(setPoint)));

    if (errordistance <= .01 && slowTimeCheck){
      slowTime = System.currentTimeMillis();
      System.out.println("Preparing");
      SmartDashboard.putNumber("What time?", (System.currentTimeMillis() - slowTime));
      slowTimeCheck = false;
    }

    if(errordistance <= .01 && (System.currentTimeMillis() - slowTime) > 2000 ){
      m_ElevatorSubsystem.m_TestMotor1.set(-0.036328);
      m_ElevatorSubsystem.m_TestMotor2.set(-0.036328);
      System.out.println("holding");
      slowTimeCheck = true;
      errordistance = ((error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000);     
        }
    
    else{
    System.out.println("Business as usual");
    // P = (TestMotor1Position+(-1))*.2
    // D = (error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000)

    out = (-0.042) + ((-TestMotor1Position+-2)*.0) - ((.0 * (error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000));//.115
    m_ElevatorSubsystem.m_TestMotor1.set(out);

    errordistance = ((error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000);
    SmartDashboard.putNumber("Error Difference", errordistance);
    
    lastError = ((-TestMotor1Position+(-2)));




    System.out.println();
    SmartDashboard.putNumber("Power out", out);
    SmartDashboard.putNumber("Test Motor 1 Position", TestMotor1Position);
    SmartDashboard.putNumber("Test Motor 2 Position", TestMotor2Position);
    SmartDashboard.putNumber("Power out", out);
    System.out.println(out);
    SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
   }
    // TestMotor1Position = m_ElevatorSubsystem.m_TestMotor1.getRotorPosition().getValueAsDouble();
    // TestMotor2Position = m_ElevatorSubsystem.m_TestMotor2.getRotorPosition().getValueAsDouble();

    // error = ((-TestMotor1Position+(setPoint)));
    // // gets how far the encoder value is from the setpoint value

    // if (D <= .01 && slowTimeCheck){
    //   slowTime = System.currentTimeMillis();
    //   System.out.println("Preparing");
    //   SmartDashboard.putNumber("What time?", (System.currentTimeMillis() - slowTime));
    //   slowTimeCheck = false;
    // } // checks to see if the elevator is moving, This should only engage after the elevator has reached the setPoint

    // if(D <= .01 && (System.currentTimeMillis() - slowTime) > 2000 ){
    //    m_ElevatorSubsystem.m_TestMotor1.set(-0.036328);
    //    m_ElevatorSubsystem.m_TestMotor2.set(-0.036328);
    //    System.out.println("holding");
    //    slowTimeCheck = true;
    //    D = (error - lastError) / ((System.currentTimeMillis()-engagetime)/1000);
    // } // if the elevator hasnt moved in a certain amount of time set both motors to the g-value

    // else{
    //   // System.out.println("Business as usual");
    //   // P = (-TestMotor1Position+(setPoint));
    //   // D = (error - lastError) / ((System.currentTimeMillis()-engagetime)/1000);
  
    //   // //out = (-0.042) + (.08 * P) - (.04 * D); // (the g-value for one motor) + (The rate at which P is applied * P) - (The rate at which D is applied * D)
    //   // out = (-0.042) + ((-TestMotor1Position+-2)*.08) - ((.04 * (error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000));
    //   // m_ElevatorSubsystem.m_TestMotor1.set(out);

    //   // SmartDashboard.putNumber("Error Difference", errordistance);
      
    //   // lastError = ((-TestMotor1Position+(setPoint)));

    // out = (-0.042) + ((-TestMotor1Position+-2)*.08) - ((.04 * (error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000));//.115
    // m_ElevatorSubsystem.m_TestMotor1.set(out);

    // errordistance = ((error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000);
    // SmartDashboard.putNumber("Error Difference", errordistance);
    
    // lastError = ((-TestMotor1Position+(-2)));
  
    //   SmartDashboard.putNumber("Power out", out);
    //   SmartDashboard.putNumber("P value", P);
    //   SmartDashboard.putNumber("D value", D);
    //   SmartDashboard.putNumber("Test Motor 1 Position", TestMotor1Position);
    //   SmartDashboard.putNumber("Test Motor 2 Position", TestMotor2Position);
    //   System.out.println(out);
    //   SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
    //  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.m_TestMotor1.set(0); // sets the output value of the motors to 0 
    m_ElevatorSubsystem.m_TestMotor2.set(0); // This will not be the case in the final otherwise our arm will be dropping in between positions.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
