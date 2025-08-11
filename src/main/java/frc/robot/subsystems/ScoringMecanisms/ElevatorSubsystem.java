package frc.robot.subsystems.ScoringMecanisms;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;



public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX m_motor;

  public ElevatorSubsystem() {
    m_motor = new TalonFX(20); // Adjust CAN ID as necessary
  }

  public void setPosition(){
    m_motor.setPosition(0);
  }
  public double getEncoderPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }
  public void setSpeed(double speed) {
      m_motor.set(speed);
  }
  public void setElevatorHeight(double low, double high, double speed){
      if (getEncoderPosition() > -high) {
        setSpeed(-speed);
      } else if (getEncoderPosition() < -low){
        setSpeed(speed);
      } else {
        setSpeed(0);
      }
  }
  public void highReef() {
    setElevatorHeight(160, 152.5, 1);
  }
  public void L3Reef() {
    setElevatorHeight(40, 29.2, .7);
  }
  public void Pickup() {
    setElevatorHeight(55, 45, 1);
  }
  public void bottomPosition() {
    setElevatorHeight(8, 0, 1);
  }
  public void L2Reef() {
    setElevatorHeight(69, 62, .9);
  }
  public void stop() {
    setSpeed(0);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Position", getEncoderPosition());
  }
}
