package frc.robot.subsystems.DiffySubsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;  
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;


public class DiffyArmSubsystem {

    private final SparkMax motor;

    private final SparkAbsoluteEncoder armEncoder;
    private SparkBaseConfig sparkMaxConfig;
    private AbsoluteEncoderConfig encoderConfig;
    private ClosedLoopConfig closedLoopConfig;
    private MAXMotionConfig maxMotionConfig;

    private SparkClosedLoopController armPIDController;

    /**
     * A Neo motor for diffy arm control
     * @param canId the motor's CAN ID
     */
    public DiffyArmSubsystem(int canId) {

        motor = new SparkMax(canId, MotorType.kBrushless);

        encoderConfig = new AbsoluteEncoderConfig();
        encoderConfig.inverted(true);

        maxMotionConfig = new MAXMotionConfig();
        maxMotionConfig.allowedClosedLoopError(.005);

        closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .positionWrappingEnabled(true)
                        .positionWrappingMinInput(0)
                        .positionWrappingMaxInput(1)
                        .apply(maxMotionConfig);
        
        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.apply(closedLoopConfig)
                      .apply(encoderConfig);

        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armEncoder = motor.getAbsoluteEncoder();
    }

    /**
     * Configures the motor's PID
     * @param p kP
     * @param i kI
     * @param d kD
     * @param ff kFF
     */
    public void configurePID(double p, double i, double d, double ff){
        closedLoopConfig.pidf(p, i, d, ff);
        sparkMaxConfig.apply(closedLoopConfig);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armPIDController = motor.getClosedLoopController();
    }

    /**
     * Using PID to move to target position
     * @param targetRads target position in radiants
     */
    public void setPosition(double targetRads) {
        double targetDouble = (targetRads + Math.PI) / (2. * Math.PI);
        armPIDController.setReference(targetDouble, ControlType.kPosition);
    }

    /**
     * Gets the motor's position through the absolute encoder
     * @return position in double from 0 to 1
     */
    public double getPosition(){
        return armEncoder.getPosition();
    }
}