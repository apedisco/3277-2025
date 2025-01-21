// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.elevatorCommand;
import frc.robot.commands.elevatorCommandTest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick m_DriveJoystick = new CommandJoystick(0);
    private final Joystick m_ElevatorJoystick = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

    public RobotContainer() {
        configureBindings();
         final JoystickButton testButton = new JoystickButton(m_ElevatorJoystick, 1);
         final JoystickButton test2Button = new JoystickButton(m_ElevatorJoystick, 3);
        testButton.whileTrue(new elevatorCommand(m_ElevatorSubsystem, MaxAngularRate, m_ElevatorJoystick));
        test2Button.whileTrue(new elevatorCommandTest(m_ElevatorSubsystem, MaxAngularRate, m_ElevatorJoystick));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_DriveJoystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_DriveJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_DriveJoystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

       /* m_DriveJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_DriveJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_DriveJoystick.getLeftY(), -m_DriveJoystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_DriveJoystick.back().and(m_DriveJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_DriveJoystick.back().and(m_DriveJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_DriveJoystick.start().and(m_DriveJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_DriveJoystick.start().and(m_DriveJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_DriveJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
      */
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
