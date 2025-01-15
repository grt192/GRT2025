// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimbConstants;


public class ClimbSubsystem extends SubsystemBase {

  private final SparkMax topMotor;
  private final SparkMax botMotor;

  private RelativeEncoder encoder;
  private final ClosedLoopConfig closedLoopConfig;
  private SparkMaxConfig sparkMaxConfig;
  private final SparkClosedLoopController steerPIDController;
  private final EncoderConfig encoderConfig;

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    
    topMotor = new SparkMax(ClimbConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    botMotor = new SparkMax(ClimbConstants.BOT_MOTOR_ID, MotorType.kBrushless);
    botMotor.follow(topMotor);
    steerPIDController = topMotor.getClosedLoopController();

    encoderConfig = new EncoderConfig();
    // encoderConfig.inverted(true);

    closedLoopConfig = new ClosedLoopConfig();//code copied from swerve, untested do NOT trust it
    closedLoopConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingMinInput(0)
                    .positionWrappingMaxInput(1);
    
    
    sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.apply(encoderConfig)
                  .apply(closedLoopConfig)
                  .inverted(true);

    topMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configurePID(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I, ClimbConstants.CLIMB_D);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  //sets the speed of the rollers

  
  public void configurePID(double p, double i, double d){
    closedLoopConfig.pid(p, i, d);
    sparkMaxConfig.apply(closedLoopConfig);
    topMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void setPosition(double targetRads) {
    double targetDouble = (targetRads + Math.PI) / (2. * Math.PI);
    System.out.println("target" + targetDouble);
    System.out.println("current" + encoder.getPosition());
    steerPIDController.setReference(targetDouble, ControlType.kPosition);
}
public void driveRollers(double speed){
  topMotor.set(speed);
}
public double getMotorPosition() {
  // Returns the position in rotations
  return encoder.getPosition();
}


  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

   //sets the actuall speed of the rollers
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
