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

    private final SparkMax diffymotor;

    private final SparkAbsoluteEncoder armEncoder;
    private SparkBaseConfig sparkMaxConfig;
    private AbsoluteEncoderConfig encoderConfig;
    private ClosedLoopConfig closedLoopConfig;
    private MAXMotionConfig maxMotionConfig;

    private SparkClosedLoopController armPIDController;

    private int upperLimit = 0;
    private int lowerLimit = 0;


    double p = 1;
    double i = 0;
    double d = 0;

    /**
     * A Neo motor for diffy arm control
     * @param canId the motor's CAN ID
     */
    public DiffyArmSubsystem(int canId) {

        diffymotor = new SparkMax(canId, MotorType.kBrushless);

        encoderConfig = new AbsoluteEncoderConfig();

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

        diffymotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        armEncoder = diffymotor.getAbsoluteEncoder();

        configurePID(p,i,d,0);
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
        diffymotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armPIDController = diffymotor.getClosedLoopController();
    }

    /**
     * Using PID to move to target position
     * @param targetDegrees target position in degrees
     */
    public void setPosition(double targetDegrees) {
        double targetDouble = (targetDegrees / 360.0);
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