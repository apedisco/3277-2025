// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3ScoreCommand extends Command {
    ArmSubsystem m_ArmSubsystem;
    ElevatorSubsystem m_ElevatorSubsystem;
  private double armEncoderValue;
  private double armEncoderValueTop;
  private double armEncoderValueBottom;
  private double out;
  private double setpoint;
  private double P;
  private double D;
  private double engagetime;
  private double timeTop;
  private double timeBottom;
  private double elevatorEncoderValue;
  private double elevatorEncoderValueCheck;
  private boolean dropcheck;
  private boolean armOn;
  /** Creates a new L3ScoreCommand. */
  public L3ScoreCommand(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ElevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    engagetime = System.currentTimeMillis();
    setpoint = .33;
    P = 1.5;
    D = 0;
    dropcheck = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     //m_ArmSubsystem.m_GrabberMotor.set((((m_TestJoystick.getRawAxis(3) + 1) / 2)*.5));
     armEncoderValue = Robot.armEncoder.get();
  if (armEncoderValue < .30 || armEncoderValue > .36 && dropcheck == true){
    System.out.println("armPID Running");
    armEncoderValue = Robot.armEncoder.get();
    armEncoderValueTop = Robot.armEncoder.get();
    timeTop = (System.currentTimeMillis()-engagetime)/1000;

    out = ((setpoint - (((armEncoderValue)))) * P); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
    m_ArmSubsystem.m_ArmMotor.set(out);

    armEncoderValueBottom = Robot.armEncoder.get();
    timeBottom = (System.currentTimeMillis()-engagetime)/1000;
  }

  if (armEncoderValue > .29 && armEncoderValue < .33 && dropcheck == true){

    //System.out.println("armPID Running");
    armEncoderValue = Robot.armEncoder.get();
    armEncoderValueTop = Robot.armEncoder.get();
    timeTop = (System.currentTimeMillis()-engagetime)/1000;

    out = ((setpoint - (((armEncoderValue)))) * P); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
    m_ArmSubsystem.m_ArmMotor.set(out);

    armEncoderValueBottom = Robot.armEncoder.get();
    timeBottom = (System.currentTimeMillis()-engagetime)/1000;

    elevatorEncoderValue = Robot.elevatorEncoder.get();


    System.out.println("Business as usual");

    //out = -(((m_ElevatorJoystick.getRawAxis(3) + 1) / 2)*.5);
    out = (0) + (((((-elevatorEncoderValue)/1000+-3.3)))* .2); // ((0.000001 * (((-elevatorEncoderValue)/1000) - ((-elevatorEncoderValueCheck)/1000))) / ((System.currentTimeMillis()-engagetime)/1000));
    m_ElevatorSubsystem.m_ElevatorMotor1.set(-out);
    m_ElevatorSubsystem.m_ElevatorMotor2.set(-out);
    

    //out = (-0.042) + (-(java.lang.Math.sqrt((-ElevatorMotor1Position+2)))* .05) - ((0.001 * (error - lastError)) / ((System.currentTimeMillis()-engagetime)/1000))

    elevatorEncoderValueCheck = Robot.elevatorEncoder.get();
    System.out.println((elevatorEncoderValueCheck/1000));
    if((elevatorEncoderValueCheck/1000) < -2.9 & dropcheck == true){
      engagetime = System.currentTimeMillis();
      dropcheck = false;
      //(elevatorEncoderValue)/1000 > 3.30
     
    }
  }
  if(dropcheck == false && System.currentTimeMillis() - engagetime <300){
    m_ArmSubsystem.m_GrabberMotor.set(.4); 
    m_ArmSubsystem.m_ArmMotor.set(0);
    System.out.println(System.currentTimeMillis() - engagetime);
  }
  if(armOn == false && dropcheck == false && System.currentTimeMillis() - engagetime >=300){
    m_ArmSubsystem.m_GrabberMotor.set(0); //&& System.currentTimeMillis() < 600
    m_ElevatorSubsystem.m_ElevatorMotor1.set(0);
    m_ElevatorSubsystem.m_ElevatorMotor2.set(0);
    elevatorEncoderValue = Robot.elevatorEncoder.get();
  }
  if (dropcheck == false && (elevatorEncoderValue/1000) > -.3){
    System.out.println("armPID Running");
    armEncoderValue = Robot.armEncoder.get();
    armEncoderValueTop = Robot.armEncoder.get();
    timeTop = (System.currentTimeMillis()-engagetime)/1000;

    out = ((.22 - (((armEncoderValue)))) * P); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
    m_ArmSubsystem.m_ArmMotor.set(out);

    armEncoderValueBottom = Robot.armEncoder.get();
    timeBottom = (System.currentTimeMillis()-engagetime)/1000;
    armOn = false;
  }
    SmartDashboard.putNumber("P output", ((setpoint - ((armEncoderValue)/1000)) * P));
    SmartDashboard.putNumber("D output",(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop))) ;
    SmartDashboard.putNumber("Arm Encoder", armEncoderValue);
    SmartDashboard.putNumber("Power out", out);
    SmartDashboard.putNumber("Timer", ((System.currentTimeMillis()-engagetime)/1000));
    SmartDashboard.putNumber("delta Setpoint", (setpoint - (((armEncoderValue)))));
    SmartDashboard.putNumber("Elevator Encoder Value", (-elevatorEncoderValue)/1000);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
