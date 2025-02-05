package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.MOTOR_ID;
import static frc.robot.Constants.DebugConstants.REV_DEBUG;
import frc.robot.util.Motors.LoggedTalon;
import java.util.OptionalInt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ClimbSubsystem extends SubsystemBase {

  private final LoggedTalon motor;
  private final TalonFXConfiguration talonConfig;


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {


    talonConfig = new TalonFXConfiguration();
    motor = new LoggedTalon(MOTOR_ID, talonConfig);
    
    
  }

  /**
   * Returns the position of the motor.
   * @return position of the motor
   */
  public double getPosition() {
    return motor.getPosition();
  }

  /**
   * Sets the speed of the motor.
   * @param speed target speed UNKNOWN USES SETVELOCITY
   */
  public void setSpeed(double speed) {
    motor.setVelocity(speed);
  }

  /*
   * Get the current speed of the motor
   * @return the current speed of the motor
   */
  public double getSpeed() {
    return motor.getVelocity();
  }

  /**
   * Stops the motor.
   */
  public void stop() {
    motor.setVelocity(0);
  }


}
