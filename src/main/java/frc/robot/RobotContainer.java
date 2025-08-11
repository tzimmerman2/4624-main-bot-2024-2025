// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ScoringMecanisms.ElevatorSubsystem;
import frc.robot.subsystems.ScoringMecanisms.IntakeSubsystem;
import frc.robot.subsystems.ScoringMecanisms.PivotSubsystem;

public class RobotContainer {
    Joystick Reef = new Joystick(1);
    Joystick Mechanic = new Joystick(2);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private static final double JOYSTICK_DEADBAND = 0;
    private static final double SpeedMultiplier = 0.7;
    public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(3);
    public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(1, 2);
    public final Pigeon2 jamaica = new Pigeon2(18, "SwerveCanivore");
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController playerOne = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        m_pivotSubsystem.resetEncoder();
        //m_elevatorSubsystem.setPosition();
        
        //drivetrain.configureAutoBuilder();
    }
    public void setupAutoBuilder() {
        drivetrain.configureAutoBuilder();
        //drivetrain.registerTelemetry(logger::telemeterize);
    }
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        NamedCommands.registerCommand("PickupPosition", new RunCommand(() -> {
            m_elevatorSubsystem.Pickup();
            m_pivotSubsystem.pickupPosition();
        }, m_elevatorSubsystem, m_pivotSubsystem));
        NamedCommands.registerCommand("L4Position", new RunCommand(() -> {
            m_elevatorSubsystem.highReef();
            m_pivotSubsystem.highScore();
        }, m_elevatorSubsystem, m_pivotSubsystem));
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
            double leftY = playerOne.getLeftY() * SpeedMultiplier;
            double leftX = playerOne.getLeftX() * SpeedMultiplier;
            double rightX = playerOne.getRightX() * SpeedMultiplier;

            double velocityX = Math.abs(leftY) > JOYSTICK_DEADBAND ? -leftY * MaxSpeed : 0;
            double velocityY = Math.abs(leftX) > JOYSTICK_DEADBAND ? -leftX * MaxSpeed : 0;
            double rotationalRate = Math.abs(rightX) > JOYSTICK_DEADBAND ? -rightX * MaxAngularRate : 0;
            if (m_elevatorSubsystem.getEncoderPosition() < -70 || m_pivotSubsystem.getEncoderPosition() < -150) {
                return drive.withVelocityX(velocityX *.7)
                        .withVelocityY(velocityY *.7)
                        .withRotationalRate(rotationalRate *.7);
            }
            return drive.withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(rotationalRate);
        })
        );

        /*controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        // reset the field-centric heading on left bumper press

    playerOne.rightBumper()
        .whileTrue(new RunCommand(() -> {
            m_elevatorSubsystem.setSpeed(-1);
            m_pivotSubsystem.setSpeed(0); 
        }, m_elevatorSubsystem, m_pivotSubsystem))
        .onFalse(new RunCommand(() -> {
            m_elevatorSubsystem.setSpeed(0);
            m_pivotSubsystem.setSpeed(0); 
        }, m_elevatorSubsystem, m_pivotSubsystem));
    playerOne.leftBumper()
        .whileTrue(new RunCommand(() -> {
            m_elevatorSubsystem.setSpeed(1);
            m_pivotSubsystem.setSpeed(0); 
        }, m_elevatorSubsystem, m_pivotSubsystem))
        .onFalse(new RunCommand(() -> {
            m_elevatorSubsystem.setSpeed(0);
            m_pivotSubsystem.setSpeed(0); 
        }, m_elevatorSubsystem, m_pivotSubsystem));
    playerOne.rightTrigger()
        .whileTrue(new RunCommand(() -> {
            m_elevatorSubsystem.setSpeed(0);
            m_pivotSubsystem.setSpeed(-0.7); 
        }, m_elevatorSubsystem, m_pivotSubsystem))
        .onFalse(new RunCommand(() -> {
            m_elevatorSubsystem.setSpeed(0);
            m_pivotSubsystem.setSpeed(0); 
        }, m_elevatorSubsystem, m_pivotSubsystem));
    playerOne.leftTrigger()
        .whileTrue(new RunCommand(() -> {
            m_elevatorSubsystem.setSpeed(0);
            m_pivotSubsystem.setSpeed(0.7); 
        }, m_elevatorSubsystem, m_pivotSubsystem))
        .onFalse(new RunCommand(() -> {
            m_elevatorSubsystem.setSpeed(0);
            m_pivotSubsystem.setSpeed(0); 
        }, m_elevatorSubsystem, m_pivotSubsystem));
    playerOne.x() 
        .whileTrue(new RunCommand(() -> m_intakeSubsystem.Troff(), m_intakeSubsystem))
        .onFalse(new RunCommand(() -> m_intakeSubsystem.setSpeed(0), m_intakeSubsystem));
    playerOne.y()
        .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    playerOne.b()
        .whileTrue(new RunCommand(() -> m_intakeSubsystem.setSpeed(0.22), m_intakeSubsystem))
        .onFalse(new RunCommand(() -> m_intakeSubsystem.setSpeed(0), m_intakeSubsystem));
    playerOne.a()
        .whileTrue(new RunCommand(() -> m_intakeSubsystem.setSpeed(-0.22), m_intakeSubsystem))
        .onFalse(new RunCommand(() -> m_intakeSubsystem.setSpeed(0), m_intakeSubsystem));
    playerOne.start()
        .onFalse(new RunCommand(() -> {
          m_elevatorSubsystem.L2Reef();
          m_pivotSubsystem.L2Reef();
        }, m_elevatorSubsystem, m_pivotSubsystem));
    playerOne.povUp()
        .onFalse(new RunCommand(() -> {
          m_elevatorSubsystem.highReef();
          m_pivotSubsystem.highScore();
      }, m_elevatorSubsystem, m_pivotSubsystem));
    playerOne.povLeft()
        .onFalse(new RunCommand(() -> {
          m_elevatorSubsystem.Pickup();
          m_pivotSubsystem.pickupPosition();
        }, m_pivotSubsystem, m_elevatorSubsystem));
    playerOne.povDown()
        .onFalse(new RunCommand(() ->  {
          m_elevatorSubsystem.bottomPosition();
          m_pivotSubsystem.bottomPosition(); 
        }, m_elevatorSubsystem, m_pivotSubsystem));
    playerOne.povRight()
        .onFalse(new RunCommand(() -> {
          m_elevatorSubsystem.L3Reef();
          m_pivotSubsystem.L3Reef();
        }, m_elevatorSubsystem, m_pivotSubsystem));

        drivetrain.registerTelemetry(logger::telemeterize);
    new Trigger(() -> Mechanic.getRawButton(4))
        .whileTrue(drivetrain.pathfindingCommand("TopSource"));
    new Trigger(() -> Reef.getRawButton(1))
        .whileTrue(drivetrain.pathfindingCommand("yes"));
            
    new Trigger(() -> Reef.getRawButton(2))
        .whileTrue(drivetrain.pathfindingCommand("yes"));
    new Trigger(() -> Reef.getRawButton(3))
        .whileTrue(drivetrain.pathfindingCommand("Button3"));
    new Trigger(() -> Reef.getRawButton(12))
        .whileTrue(drivetrain.pathfindingCommand("yes"));
    }
    
    

    public Command getAutonomousCommand(String autoName) {
        //drivetrain.configureAutoBuilder();
        try {
            
            
            NamedCommands.registerCommand("L4Position", new RunCommand(() -> {
                m_elevatorSubsystem.highReef();
                m_pivotSubsystem.highScore();
            }, m_elevatorSubsystem, m_pivotSubsystem));
            
            NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> 
                m_intakeSubsystem.setSpeed(-0.20), m_intakeSubsystem));
            NamedCommands.registerCommand("Place", new InstantCommand(() -> 
                m_intakeSubsystem.setSpeed(-0.6), m_intakeSubsystem));
            NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> 
                m_intakeSubsystem.setSpeed(0), m_intakeSubsystem));
            
            NamedCommands.registerCommand("BottomPosition", new RunCommand(() -> {
                m_elevatorSubsystem.bottomPosition();
                m_pivotSubsystem.bottomPosition();
            }, m_elevatorSubsystem, m_pivotSubsystem));
            NamedCommands.registerCommand("PickupPosition", new RunCommand(() -> {
                m_elevatorSubsystem.Pickup();
                m_pivotSubsystem.pickupPosition();
            }, m_elevatorSubsystem, m_pivotSubsystem));

            Command autoCommand = AutoBuilder.buildAuto(autoName);
    
            System.out.println("Auto successfully loaded.");
            return autoCommand;
            
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    
    
    
}
