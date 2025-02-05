package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.MOTOR_ID;
import static frc.robot.Constants.DebugConstants.REV_DEBUG;
import frc.robot.util.Motors.LoggedTalon;
import java.util.OptionalInt;

public class ClimbSubsystem extends SubsystemBase {

  private final LoggedTalon motor;


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    
    
  }

  /**
   * Returns the position of the motor.
   * @return position of the motor
   */
  public double getPosition() {
    return 0.0; // Blank function
  }

  /**
   * Sets the speed of the motor.
   * @param speed target speed from -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    // Blank function
  }

  /**
   * Sets the motor to reverse at a given speed.
   * @param speed target speed from -1.0 to 1.0
   */
  public void setReverse(double speed) {
    // Blank function
  }

  /**
   * Stops the motor.
   */
  public void stop() {
    // Blank function
  }


}
