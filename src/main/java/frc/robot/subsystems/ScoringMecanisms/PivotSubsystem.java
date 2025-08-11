package frc.robot.subsystems.ScoringMecanisms;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSubsystem extends SubsystemBase {
  private final SparkMax m_motor;
  private final RelativeEncoder m_encoder;

  public PivotSubsystem(int canID) {
    m_motor = new SparkMax(canID, MotorType.kBrushless); // Adjust CAN ID as necessary
    m_encoder = m_motor.getEncoder();
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }

  public void setEncoderPosition(double position) {
    m_encoder.setPosition(position);
  }

  public void setSpeed(double speed) {
    if (getEncoderPosition() > -5 && speed > 0) {
      m_motor.set(0);
    }
    m_motor.set(speed);
  }

  public void stop() {
    setSpeed(0);
  }

  public void highScore() {
    if (getEncoderPosition() < -170) {
      setSpeed(1);
    } else if (getEncoderPosition() > -161){
      setSpeed(-1);
    } else {
      setSpeed(0);
    }
  }

  public void bottomPosition(){
    if (getEncoderPosition() < -11) {
      setSpeed(.7);
    } else if (getEncoderPosition() > -5){
      setSpeed(-.7);
    } else {
      setSpeed(0);
    }
  }

  public void pickupPosition(){
    if (getEncoderPosition() < -66) {
      setSpeed(0.7);
    } else if (getEncoderPosition() > -58){
      setSpeed(-0.7);
    } else {
      setSpeed(0);
    }
  }

  public void L3Reef() {
    if (getEncoderPosition() < -176) {
      setSpeed(0.9);
    } else if (getEncoderPosition() > -166){
      setSpeed(-0.9);
    } else {
      setSpeed(0);
    }
  }
  public void L2Reef() {
    if (getEncoderPosition() < -37) {
      setSpeed(0.7);
    } else if (getEncoderPosition() > -29){
      setSpeed(-0.7);
    } else {
      setSpeed(0);
    }
  }
  public boolean isInPickupPosition() {
    return getEncoderPosition() >= -75 && getEncoderPosition() <= -65;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder Position", getEncoderPosition());
  }
}
