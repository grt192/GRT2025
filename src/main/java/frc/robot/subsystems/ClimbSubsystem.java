// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

import edu.wpi.first.math.util.Units;

public class ClimbSubsystem extends SubsystemBase {

  private final SparkMax topMotor;
  private final SparkMax botMotor;
  private final int botMotorID;

  private final ClosedLoopConfig closedLoopConfig;
  private final SparkMaxConfig sparkMaxConfig;
  private final SparkMaxConfig sparkMaxConfigFollow;
  private final EncoderConfig encoderConfig;
  private final SoftLimitConfig softLimitConfig;

  private final RelativeEncoder climbEncoder;
  //private final SparkClosedLoopController steerPIDController;

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    
    topMotor = new SparkMax(ClimbConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    botMotor = new SparkMax(ClimbConstants.BOT_MOTOR_ID, MotorType.kBrushless);
    climbEncoder = topMotor.getEncoder();

    botMotorID = 0; //have to change later

    //steerPIDController = topMotor.getClosedLoopController();

    encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(ClimbConstants.CLIMB_GEAR_RATIO);

    softLimitConfig = new SoftLimitConfig();
    softLimitConfig.forwardSoftLimitEnabled(true)
                   .forwardSoftLimit(Units.degreesToRadians(90)) //find out with mech for soft lim
                   .reverseSoftLimitEnabled(true)
                   .reverseSoftLimit(0);

    closedLoopConfig = new ClosedLoopConfig(); 
    //closedLoopConfig.pid(ClimbConstants.CLIMB_P, ClimbConstants.CLIMB_I, ClimbConstants.CLIMB_D);
    
    sparkMaxConfig = new SparkMaxConfig();
    sparkMaxConfig.apply(encoderConfig)
                  .apply(softLimitConfig)
                  .apply(closedLoopConfig);

    sparkMaxConfigFollow = new SparkMaxConfig(); //to follow the motor
    sparkMaxConfigFollow.follow(botMotorID)
                        .apply(encoderConfig)
                        .apply(softLimitConfig)
                        .apply(closedLoopConfig);
                  
    topMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    botMotor.configure(sparkMaxConfigFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
  }

  public void setPosition(double targetRads) {
    double targetDouble = (targetRads + Math.PI) / (2. * Math.PI);
    System.out.println("target" + targetDouble);
    System.out.println("current" + climbEncoder.getPosition());
  }

  public void driveRollers(double speed){
    topMotor.set(speed);
  }

  public double getMotorPosition() {
    // Returns the position in rotations
    return climbEncoder.getPosition();
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
