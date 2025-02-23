package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.MOTOR_ID;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DebugConstants;
import frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbSubsystem extends SubsystemBase {

  private final LoggedTalon motor;
  private final TalonFXConfiguration motorConfig;


  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    motorConfig = new TalonFXConfiguration()
      .withSlot0(
        new Slot0Configs()
          .withKP(ClimbConstants.kP)
          .withKI(ClimbConstants.kI)
          .withKD(ClimbConstants.kD)
          .withKS(ClimbConstants.kS)
          .withKV(ClimbConstants.kV)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT)
      )
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
      );
      

    motor = new LoggedTalon(MOTOR_ID, "can", motorConfig);
  }

  @Override
  public void periodic(){
    motor.logStats();
    if(DebugConstants.MASTER_DEBUG || ClimbConstants.CLIMB_DEBUG){
      motor.publishStats();
    }
  }

  /**
   * Sets the motor to climb speed.
   */
  public void start() {
    motor.setVelocityReference(ClimbConstants.CLIMB_SPEED);
  }

  /**
   * Stops the motor.
   */
  public void stop() {
    motor.setVelocityReference(0);
  }

  /**
   * Gets the closed loop error of the motor.
   * @return
   */
  public double getClosedLoopError(){
    return motor.getClosedLoopError();
  }
}