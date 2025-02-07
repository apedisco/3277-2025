// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L2ScoreCommand extends Command {
  ArmSubsystem m_ArmSubsystem;
  private double armEncoderValue;
  private double armEncoderValueTop;
  private double armEncoderValueBottom;
  private double out;
  private double setPointScoring;
  private double setPointIntaking;
  private double P;
  private double D;
  private double engagetime;
  private double timeTop;
  private double timeBottom;
  private double elevatorEncoderValue;
  private double elevatorEncoderValueCheck;
  private boolean armStop;
  private boolean stopCheck;
  private boolean armOn;
  /** Creates a new L2ScoreCommand. */
  public L2ScoreCommand(ArmSubsystem armSubsystem) {
    m_ArmSubsystem = armSubsystem;
    addRequirements(m_ArmSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    engagetime = System.currentTimeMillis();
    setPointScoring = .33;
    setPointIntaking = .22;
    P = 1.5;
    D = 0;
    stopCheck = true;
    armStop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     armEncoderValue = Robot.armEncoder.get();
  if (armStop == false){
    System.out.println("armPID Running");
    armEncoderValue = Robot.armEncoder.get();
    armEncoderValueTop = Robot.armEncoder.get();
    timeTop = (System.currentTimeMillis()-engagetime)/1000;

    out = ((setPointScoring - (((armEncoderValue)))) * P); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
    m_ArmSubsystem.m_ArmMotor.set(out);

    armEncoderValueBottom = Robot.armEncoder.get();
    timeBottom = (System.currentTimeMillis()-engagetime)/1000;
  }

  if(armEncoderValue > .29 && armEncoderValue < .33 && stopCheck){
    engagetime = System.currentTimeMillis();
    stopCheck = false;
  }

  if(armEncoderValue > .29 && armEncoderValue < .33 && (System.currentTimeMillis() - engagetime <= 300 )){
    m_ArmSubsystem.m_GrabberMotor.set(8);
  }

  if(System.currentTimeMillis() - engagetime > 300 && stopCheck == false){
    armStop = true;
    System.out.println("armPID Running");
    armEncoderValue = Robot.armEncoder.get();
    armEncoderValueTop = Robot.armEncoder.get();
    timeTop = (System.currentTimeMillis()-engagetime)/1000;
    
    out = ((setPointIntaking - (((armEncoderValue)))) * P); //(D * ((armEncoderValueBottom)-(armEncoderValueTop))/(timeBottom - timeTop));
    m_ArmSubsystem.m_ArmMotor.set(out);

    armEncoderValueBottom = Robot.armEncoder.get();
    timeBottom = (System.currentTimeMillis()-engagetime)/1000;
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
