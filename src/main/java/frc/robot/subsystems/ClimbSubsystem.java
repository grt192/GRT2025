// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.TOP_MOTOR_CONFIG;
import static frc.robot.Constants.ClimbConstants.TOP_MOTOR_ID;
import static frc.robot.Constants.ClimbConstants.BOT_MOTOR_CONFIG;
import static frc.robot.Constants.ClimbConstants.BOT_MOTOR_ID;
import static frc.robot.Constants.DebugConstants.REV_DEBUG;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.util.Motors.LoggedSparkMax;

public class ClimbSubsystem extends SubsystemBase {

  private final SparkMax topMotor;
  private final SparkMax botMotor;
  private double targetSpeed = 0;

  private ClosedLoopConfig closedLoopConfig;
  private EncoderConfig encoderConfig;
  private SoftLimitConfig softLimitConfig;

  private RelativeEncoder encoder;
  private SparkMaxConfig topSparkMaxConfig, botSparkMaxConfig;

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    topMotor = new SparkMax(TOP_MOTOR_ID, MotorType.kBrushless);
    botMotor = new SparkMax(BOT_MOTOR_ID, MotorType.kBrushless);
    

    closedLoopConfig = new ClosedLoopConfig()
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(0, 1);
    encoderConfig = new EncoderConfig();
    encoder = topMotor.getEncoder();

    softLimitConfig = new SoftLimitConfig();
      softLimitConfig.forwardSoftLimitEnabled(true)
                     .forwardSoftLimit(53)
                     .reverseSoftLimitEnabled(true)
                     .reverseSoftLimit(0);


    topSparkMaxConfig = new SparkMaxConfig();
    topSparkMaxConfig.apply(closedLoopConfig)
                  .apply(softLimitConfig)
                  .inverted(true);

    botSparkMaxConfig = new SparkMaxConfig();
    botSparkMaxConfig.apply(closedLoopConfig)
                  .inverted(true)
                  .apply(softLimitConfig)
                  .follow(TOP_MOTOR_ID);

    topMotor.configure(topSparkMaxConfig,
    ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
    );
    botMotor.configure(botSparkMaxConfig,
    ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
    );

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
   * Set encoder position to 0
   */
  public void resetEncoderPos(){
    encoder.setPosition(0);
  }


  /**
   * Gets the position of the top motor
   * @return position of the top motor in rotations after taking the position
   * conversion factor into account
   */
  public double getPosition() {
    return encoder.getPosition();
  }
  /**
   * Gets the velocity of the top motor
   * @return velocity of the top motor in RPM after taking the velocity
   * conversion factor into account
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Sets the speed of the motors
   * @param speed target speed from -1.0 to 1.0
   */
  public void setSpeed(double speed){
    targetSpeed = speed;
  }
}