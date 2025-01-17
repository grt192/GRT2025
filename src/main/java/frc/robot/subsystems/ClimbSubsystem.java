package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.MOTOR_ID;
import static frc.robot.Constants.DebugConstants.REV_DEBUG;
import frc.robot.util.Motors.LoggedTalon;
import java.util.OptionalInt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class ClimbSubsystem extends SubsystemBase {

  // private final LoggedTalon motor;
  // private final TalonFXConfiguration talonConfig;
  private TalonFX motor;
  private double speed;


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {


    // talonConfig = new TalonFXConfiguration();
    // motor = new LoggedTalon(MOTOR_ID, talonConfig);
    motor = new TalonFX(MOTOR_ID);
    
    
  }

  // /**
  //  * Returns the position of the motor.
  //  * @return position of the motor
  //  */
  // public double getPosition() {
  //   return 
  // }

  /**
   * Sets the speed of the motor.
   * @param speed target speed UNKNOWN USES SETVELOCITY
   */
  public void setSpeed(double speed) {
    this.speed = speed;
    motor.set(speed);
  }

  /*
   * Get the current speed of the motor
   * @return the current speed of the motor
   */
  public double getSpeed() {
    return speed;
  }

  /**
   * Stops the motor.
   */
  public void stop() {
    motor.set(0);
  }


}
