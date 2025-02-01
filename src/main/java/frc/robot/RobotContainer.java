// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.servohub.config.ServoChannelConfig.PulseRange;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Configs.CoralSubsystem;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.IntakeIn;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPractice;
import frc.robot.subsystems.ElevatorPractice.Setpoint;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick1 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

   //public final Elevator m_Elevator = new Elevator();

     private final ElevatorPractice m_elevator2 = new ElevatorPractice();

   public final Intake m_Intake = new Intake();




    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //having elevator go to a certain position
        //joystick.y().onTrue(m_Elevator.runOnce(() -> m_Elevator.setPositionGoal(.02)));
        //joystick.a().onTrue(m_Elevator.runOnce(() -> m_Elevator.setPositionGoal(0.0)));

        //Calling ElevatorUp command
       //joystick.b().onTrue(new ElevatorUp(m_Elevator));

       //manual elevator that moves with speed
       //joystick.x().whileTrue(new InstantCommand(() -> m_Elevator.manual(1.0)));
       //joystick.x().onFalse(new InstantCommand(() -> m_Elevator.manual(0)));

         // A Button -> Elevator/Arm to level 2 position
         joystick.a().onTrue(m_elevator2.setSetpointCommand(Setpoint.kLevel2));

        // X Button -> Elevator/Arm to level 3 position
        joystick.x().onTrue(m_elevator2.setSetpointCommand(Setpoint.kLevel3));

        // Y Button -> Elevator/Arm to level 4 position
       joystick.y().onTrue(m_elevator2.setSetpointCommand(Setpoint.kLevel4));

      //Intake Controls
      //joystick.a().onTrue(new IntakeIn(m_Intake));

      joystick.b().whileTrue(new InstantCommand(() -> m_Intake.runIntakeSpeed(.5)));
      joystick.b().onFalse(new InstantCommand(() -> m_Intake.runIntakeSpeed(0)));



        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
        //set up auto selecter and commmands
    }
}
