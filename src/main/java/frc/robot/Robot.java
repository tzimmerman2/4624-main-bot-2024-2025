// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.ejml.equation.MatrixConstructor;

import edu.wpi.first.math.Matrix;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Nat;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ScoringMecanisms.ElevatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
public class Robot extends TimedRobot {
  private final double stdDev2tags = .12;
  private final double stdDevTheta1=0;
  private Command m_autonomousCommand;
  private boolean kUseLimelight = false;
  private final DigitalInput input = new DigitalInput(0);
  private final RobotContainer m_robotContainer;
  private SendableChooser<String> autoPath = new SendableChooser<>();
  public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public Robot() {
    kUseLimelight=SmartDashboard.putBoolean("limelight", kUseLimelight);
    m_robotContainer = new RobotContainer();
    autoPath.setDefaultOption("BlueBarge", "BlueBarge");
    autoPath.addOption("BlueBargeHelp", "BlueBargeHelp");
    autoPath.addOption("RedBarge", "RedBarge");
    autoPath.addOption("RedBargeHelp", "RedBargeHelp");
    autoPath.addOption("Single Middle BB", "Single Middle BB");
    autoPath.addOption("Single Middle RB", "Single Middle RB");
    autoPath.addOption("Test", "Test");
    autoPath.addOption("BlueBarge3", "BlueBarge3");
    autoPath.addOption("RightAuto","RightAuto");
    SmartDashboard.putData("Auto Path", autoPath);
    //m_robotContainer.setupAutoBuilder();
  }


  @Override
  public void robotPeriodic() {
    kUseLimelight=SmartDashboard.getBoolean("limelight", kUseLimelight);
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putString("Auto Path Selected", autoPath.getSelected());
    if (!input.get()) {
      m_elevatorSubsystem.setPosition();
    }
    SmartDashboard.putNumber("switch", input.get() ? 1 : 0);
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      SmartDashboard.putNumber("heading", headingDeg);
      double pitchDeg = m_robotContainer.jamaica.getPitch().getValueAsDouble();
      double rollDeg = m_robotContainer.jamaica.getRoll().getValueAsDouble();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight-br", headingDeg, 0, pitchDeg, 0, rollDeg, 0);
      var llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-br");
      LimelightHelpers.SetRobotOrientation("limelight-bl", headingDeg, 0, pitchDeg, 0, rollDeg, 0);
      var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bl");
      LimelightHelpers.SetRobotOrientation("limelight-bl", headingDeg, 0, pitchDeg, 0, rollDeg, 0);
      var llMeasurement3 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
      


      
      

      
      if (llMeasurement1 != null && llMeasurement1.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        
        System.out.println("limelight backright is valid");
        Matrix<N3, N1> standardDeviationMatrix1 = new Matrix<>(Nat.N3(), Nat.N1());
        standardDeviationMatrix1.set(0, 0, stdDev2tags * llMeasurement1.pose.getTranslation().getNorm());
        standardDeviationMatrix1.set(1, 0, stdDev2tags * llMeasurement1.pose.getTranslation().getNorm());
        standardDeviationMatrix1.set(2, 0, stdDevTheta1);
        SmartDashboard.putString("standardDeviationMatrix1", standardDeviationMatrix1.toString());
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement1.pose, llMeasurement1.timestampSeconds, standardDeviationMatrix1);
      }
      
      
      if (llMeasurement2 != null && llMeasurement2.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        System.out.println("limelight backleft is valid");
        Matrix<N3, N1> standardDeviationMatrix2 = new Matrix<>(Nat.N3(), Nat.N1());
        standardDeviationMatrix2.set(0, 0, stdDev2tags * llMeasurement2.pose.getTranslation().getNorm());
        standardDeviationMatrix2.set(1, 0, stdDev2tags * llMeasurement2.pose.getTranslation().getNorm());
        standardDeviationMatrix2.set(2, 0, stdDevTheta1);
        SmartDashboard.putString("standardDeviationMatrix2", standardDeviationMatrix2.toString());
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement2.pose, llMeasurement2.timestampSeconds, standardDeviationMatrix2);
      }

      if (llMeasurement3 != null && llMeasurement3.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        System.out.println("limelight front is valid");
        Matrix<N3, N1> standardDeviationMatrix3 = new Matrix<>(Nat.N3(), Nat.N1());
        standardDeviationMatrix3.set(0, 0, stdDev2tags * llMeasurement3.pose.getTranslation().getNorm());
        standardDeviationMatrix3.set(1, 0, stdDev2tags * llMeasurement3.pose.getTranslation().getNorm());
        standardDeviationMatrix3.set(2, 0, stdDevTheta1);
        SmartDashboard.putString("standardDeviationMatrix2", standardDeviationMatrix3.toString());
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement2.pose, llMeasurement2.timestampSeconds, standardDeviationMatrix3);
      }
    }
  }
  
  


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //m_robotContainer.setupAutoBuilder();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.setupAutoBuilder();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoPath.getSelected());

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
    m_robotContainer.m_elevatorSubsystem.setSpeed(0);
    m_robotContainer.m_pivotSubsystem.setSpeed(0);
    m_robotContainer.m_intakeSubsystem.setSpeed(0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //EnableLimelight(true);
  }

  @Override
  public void teleopPeriodic() {}

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

  public void EnableLimelight(boolean useLimelight) {
    kUseLimelight = useLimelight;
  }

}
