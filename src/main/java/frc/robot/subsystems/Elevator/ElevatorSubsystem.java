package frc.robot.subsystems.Elevator;


import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.controllers.MechController;
import frc.robot.util.LoggedBooleanSensor;
import frc.robot.util.LoggedTalon;

/**
* Elevator Class.
*/
public class ElevatorSubsystem extends SubsystemBase {
  
  //motor stuff and configs
  private final LoggedTalon motor;
  private final TalonFXConfiguration motorConfig;
  private double arbFF;
  
  //limit switch
  private final LoggedBooleanSensor zeroLimitSwitch;
  
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
      )
      .withMotorOutput(
        new MotorOutputConfigs()
          // .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive)
      )
      .withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
          .withReverseSoftLimitEnable(true)
          .withForwardSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(ElevatorConstants.REVERSE_LIMIT)
          .withForwardSoftLimitThreshold(ElevatorConstants.FORWARD_LIMIT)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withStatorCurrentLimit(ElevatorConstants.CURRENT_LIMIT)  
      );
      
    motor = new LoggedTalon(
      ElevatorConstants.MOTOR_ID, "can", motorConfig 
    );

    zeroLimitSwitch = new LoggedBooleanSensor(
      "Elevator Limit Switch", ElevatorConstants.LIMIT_ID
    ); 
  }
  
  @Override
  public void periodic() {
    if(!zeroLimitSwitch.get()){
      motor.setPosition(0);
      // motor.setPower(0);
    }
    System.out.println(motor.getPosition());
    motor.logStats();
    zeroLimitSwitch.logStats();
    if(DebugConstants.MASTER_DEBUG || ElevatorConstants.ELEVATOR_DEBUG) {
      motor.publishStats();
      zeroLimitSwitch.publishStats();
    }
  }
  
  /**
   * Sets the elevator to a specific position.
   * @param positionReference target position
   */
  public void setPositionReference(double positionReference){
    if (positionReference > motor.getPosition()) {
      arbFF = ElevatorConstants.arbFF;
    }
    else {
      // arbFF = -20;
    }
    arbFF = ElevatorConstants.arbFF;
    motor.setPositionReferenceWithArbFF(positionReference, arbFF);
  }

  public void setVelocityReference(double velocityReference){
    motor.setVelocityReference(velocityReference);
  }

  public void setPower(double power) {
    // System.out.println(power);
    motor.setPower(power);
  }
  
  public void setTorqueCurrentFOC(double current){
    motor.setTorqueCurrentFOC(current);
  }

  public void setDutyCycle(double output){
    motor.setDutyCycle(output);
  }
  /**
   * Gets the closed loop error of the motor.
   * @return closed loop error
   */
  public double getClosedLoopError(){
    return motor.getClosedLoopError();
  }

  public boolean getLimitSwitch(){
    return !zeroLimitSwitch.get();
  }

  public double getPosition() {
    return motor.getPosition();
  }
}