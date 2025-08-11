package frc.robot.subsystems.ScoringMecanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_motor1;
  private final SparkMax m_motor2;
  private final RelativeEncoder m_encoder1;
  private final RelativeEncoder m_encoder2;

  public IntakeSubsystem(int canID1, int canID2) {
    m_motor1 = new SparkMax(canID1, MotorType.kBrushless);
    m_motor2 = new SparkMax(canID2, MotorType.kBrushless);
    m_encoder1 = m_motor1.getEncoder();
    m_encoder2 = m_motor2.getEncoder();
  }

  public void resetEncoders() {
    m_encoder1.setPosition(0);
    m_encoder2.setPosition(0);
  }

  public double getEncoderPosition1() {
    return m_encoder1.getPosition();
  }

  public double getEncoderPosition2() {
    return m_encoder2.getPosition();
  }

  public void setSpeed(double speed) {
    m_motor1.set(speed);
    m_motor2.set(-speed);
  }
  public void Troff(){
    m_motor1.set(0.1);
    m_motor2.set(-0.3);
  }
  public double getCurrentLeft(){
    return m_motor1.getOutputCurrent();
  }
  public double getCurrentRight(){
    return m_motor2.getOutputCurrent();
  }
  
  public boolean isCurrentTooHigh(double threshold) {
    return getCurrentLeft() > threshold || getCurrentRight() > threshold;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Current Left", getCurrentLeft());
    SmartDashboard.putNumber("Intake Current Right", getCurrentRight());
  }
}


