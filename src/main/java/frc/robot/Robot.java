// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.Rev2mDistanceSensor.Port;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  

  public DoubleLogEntry motor1Position;

  private final RobotContainer m_robotContainer;
  ArmSubsystem m_ArmSubsystem;
  ElevatorSubsystem m_ElevatorSubsystem;

  public double ATag;

  public static DigitalInput input0 = new DigitalInput(0);
  public static DigitalInput input1 = new DigitalInput(1);
  public static DigitalInput input2 = new DigitalInput(2);
  public static DigitalInput input3 = new DigitalInput(3);
  public static DigitalInput MasterStagingSensor = new DigitalInput(4);
  public static Encoder elevatorEncoder = new Encoder(input0, input1);
  public static DutyCycleEncoder armEncoder = new DutyCycleEncoder(input2);
  // public static Rev2m
  //public static Encoder armEncoder = new Encoder(input2, input3);

  // private Rev2mDistanceSensor distOnboard; 
  // private Rev2mDistanceSensor distMXP;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLog log = DataLogManager.getLog();

    motor1Position = new DoubleLogEntry(log, "Motor 1 Encoder position");
  }

  @Override
  public void robotPeriodic() {
  SmartDashboard.putNumber("Elevator Encoder Value", elevatorEncoder.get());
  SmartDashboard.putNumber("Arm Encoder Value", armEncoder.get());
    CommandScheduler.getInstance().run(); 
 
  NetworkTable s_table = NetworkTableInstance.getDefault().getTable("limelight-shoot"); 

  // long[] rotation_array = NetworkTableInstance.getDefault().getTable("limelight-shoot").getEntry("botpose_targetspace").getIntegerArray(new long[6]);

  // long[] rotation_array = s_table.getEntry("camerapose_targetspace").getIntegerArray(new long[6]);
  NetworkTableEntry cam3d = s_table.getEntry("camerapose_targetspace");
  NetworkTableEntry stx = s_table.getEntry("tx");
  // NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = s_table.getEntry("ta");
  NetworkTableEntry stid = s_table.getEntry("tid");
  NetworkTableEntry stv = s_table.getEntry("tv");
  //NetworkTableEntry sry = s_table.getEntry("ry");
  //NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]);

  
  
  //System.out.println((m_DriveJoystick.getRawAxis(3) + 1) / 2);
  //read values periodically
  double sx = stx.getDouble(0.0);
  // double sy = sty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double said = stid.getDouble(0);
  double[] rotation = cam3d.getDoubleArray(new double[6]);
  ATag = stv.getDouble(0);
  SmartDashboard.putNumber("ATag Detector", ATag);
  SmartDashboard.putNumber("Limelight", sx);
  SmartDashboard.putNumber("ATag Area", area);
  SmartDashboard.putNumber("rotation", rotation[4]);


  ATag  = SmartDashboard.getNumber("ATag Detector", 0);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    //  if(distOnboard.isRangeValid()) {
    //   SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
    //   SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
    // }

    // if(distMXP.isRangeValid()) {
    //   SmartDashboard.putNumber("Range MXP", distMXP.getRange());
    //   SmartDashboard.putNumber("Timestamp MXP", distMXP.getTimestamp());
    // }


  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
