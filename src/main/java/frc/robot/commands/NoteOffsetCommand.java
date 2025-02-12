// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoublePredicate;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NoteOffsetCommand extends Command {

  CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
 // ShootingSubsystem m_ShootingSubsystem;

  private Timer Runtime;
  private double rotSpeed;
  private double yDist;
  private double ATag;
  private double MaxSpeed = 1;
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private double engagetime;

  private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

  /** Creates a new NoteOffsetCommand. */
  public NoteOffsetCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, double Timeout, double ATag) {

    m_CommandSwerveDrivetrain = commandSwerveDrivetrain;
    addRequirements(m_CommandSwerveDrivetrain);
    // m_ShootingSubsystem = shootingSubsystem;
    // addRequirements(m_ShootingSubsystem);
    Runtime = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    engagetime = System.currentTimeMillis();
    Runtime.reset();
    Runtime.start();
    //m_IntakeSubsystem.setspeed(1.0);

    System.out.println("Note Aim Started Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // double tx = LimelightHelpers.getTX("limelight-note");

    // boolean RunNote = LimelightHelpers.getTV("limelight-note");

    ATag  = SmartDashboard.getNumber("ATag Detector", 0);
    rotSpeed = SmartDashboard.getNumber("Limelight", 0);
    yDist = (rotSpeed+12);

    

    // RoboPose = m_CommandSwerveDrivetrain.getrobotpose();
    if ((System.currentTimeMillis()-engagetime) < 500){

      System.out.println(rotSpeed);
      //System.out.println();

    m_CommandSwerveDrivetrain.setControl( robotCentric
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withVelocityX(0) //rotSpeed*-.02
    .withVelocityY(.5)
    .withDeadband(0)

    .withRotationalRate(0)
    );
  }
  else {
    System.out.println();
    m_CommandSwerveDrivetrain.setControl( robotCentric
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage)
    .withVelocityX( 0)
    .withDeadband(0)
    .withVelocityY(0)
    
    
    .withRotationalRate(0)); 
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
