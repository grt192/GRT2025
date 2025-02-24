package frc.robot.subsystems.Elevator;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTalon;

/**
* Elevator Class.
*/
public class ElevatorSubsystem extends SubsystemBase {
  
  //motor stuff and configs
  private final LoggedTalon motor;
  private final TalonFXConfiguration motorConfig;
  
  //limit switch
  private final DigitalInput zeroLimitSwitch;
  
  /** 
   * Constructs limit switch and motors.
   */
  public ElevatorSubsystem() {
    motorConfig = new TalonFXConfiguration()
      .withSlot0(
        new Slot0Configs()
          .withKP(ElevatorConstants.kP)
          .withKI(ElevatorConstants.kI)
          .withKD(ElevatorConstants.kD)
          .withKS(ElevatorConstants.kS)
      )
      .withMotorOutput(
        new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)  
      )
      .withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(ElevatorConstants.FORWARD_LIMIT)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(ElevatorConstants.REVERSE_LIMIT)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withStatorCurrentLimit(ElevatorConstants.CURRENT_LIMIT)  
      );
      
    motor = new LoggedTalon(
      ElevatorConstants.MOTOR_ID, "can", motorConfig 
    );

    zeroLimitSwitch = new DigitalInput(ElevatorConstants.LIMIT_ID); 
  }
  
  @Override
  public void periodic() {
    if(zeroLimitSwitch.get()){
      motor.setPosition(0);
    }
    motor.logStats();
    if(DebugConstants.MASTER_DEBUG || ElevatorConstants.ELEVATOR_DEBUG) {
      motor.publishStats();
    }
  }
  
  /**
   * Sets the elevator to a specific position.
   * @param positionReference target position
   */
  public void setPositionReference(double positionReference){
    motor.setPositionReference(positionReference);
  }

  public void setVelocityReference(double velocityReference){
    motor.setVelocityReference(velocityReference);
  }
  
  /**
   * Gets the closed loop error of the motor.
   * @return closed loop error
   */
  public double getClosedLoopError(){
    return motor.getClosedLoopError();
  }
}