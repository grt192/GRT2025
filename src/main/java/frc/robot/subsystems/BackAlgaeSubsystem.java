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
import frc.robot.Constants.BackAlgaeConstants;

import edu.wpi.first.wpilibj.DigitalInput;


public class BackAlgaeSubsystem extends SubsystemBase {
  static double speed;
  static double angle;
  private final SparkMax pivotMotor;
  private final SparkMax rollerMotor;

  private RelativeEncoder encoder;
  private final ClosedLoopConfig closedLoopConfig;
  private SparkMaxConfig sparkMaxConfig;
  private final SparkClosedLoopController steerPIDController;
  private final EncoderConfig encoderConfig;

  private final DigitalInput algaeSensor;

  /** Creates a new ExampleSubsystem. */
  public BackAlgaeSubsystem() {
    angle = 0;
    pivotMotor = new SparkMax(BackAlgaeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    rollerMotor = new SparkMax(BackAlgaeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    steerPIDController = pivotMotor.getClosedLoopController();

    encoderConfig = new EncoderConfig();
    // encoderConfig.inverted(true);

    closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingMinInput(0)
                    .positionWrappingMaxInput(1);
    
    
    sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.apply(encoderConfig)
                  .apply(closedLoopConfig)
                  .inverted(true);

    pivotMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configurePID(BackAlgaeConstants.PIVOT_P, BackAlgaeConstants.PIVOT_I, BackAlgaeConstants.PIVOT_D);

    algaeSensor = new DigitalInput(BackAlgaeConstants.SENSOR_ID);

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
    pivotMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void setPosition(double targetRads) {
    double targetDouble = (targetRads + Math.PI) / (2. * Math.PI);
    System.out.println("target" + targetDouble);
    System.out.println("current" + encoder.getPosition());
    steerPIDController.setReference(targetDouble, ControlType.kPosition);
}
public void driveRollers(double speed){
  rollerMotor.set(speed);
}
public double getMotorPosition() {
  // Returns the position in rotations
  return encoder.getPosition();
}
public boolean SensorGet(){
  return algaeSensor.get();
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
