// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.TOP_MOTOR_CONFIG;
import static frc.robot.Constants.ClimbConstants.BOT_MOTOR_CONFIG;
import static frc.robot.Constants.DebugConstants.REV_DEBUG;
import frc.robot.util.Motors.LoggedSparkMax;

public class ClimbSubsystem extends SubsystemBase {

  private final LoggedSparkMax topMotor;
  private final LoggedSparkMax botMotor;
  private double targetSpeed = 0;

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    topMotor = new LoggedSparkMax(TOP_MOTOR_CONFIG);
    botMotor = new LoggedSparkMax(BOT_MOTOR_CONFIG);
  }

  @Override
  public void periodic() {
    topMotor.set(targetSpeed);
    // topMotor.logStats();
    // botMotor.logStats();
    if(REV_DEBUG){
      // topMotor.publishStats();
      // botMotor.publishStats();
    }
  }

  /**
   * Gets the position of the top motor
   * @return position of the top motor in rotations after taking the position
   * conversion factor into account
   */
  public double getPosition() {
    return topMotor.getPosition();
  }

  /**
   * Gets the velocity of the top motor
   * @return velocity of the top motor in RPM after taking the velocity
   * conversion factor into account
   */
  public double getVelocity() {
    return topMotor.getVelocity();
  }

  /**
   * Sets the speed of the motors
   * @param speed target speed from -1.0 to 1.0
   */
  public void setSpeed(double speed){
    targetSpeed = speed;
  }
}