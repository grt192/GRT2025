package frc.robot.subsystems.swerve;

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

public class NeoSteerMotor {

    private final SparkMax motor;

    private final SparkAbsoluteEncoder steerEncoder;
    private SparkBaseConfig sparkMaxConfig;
    private AbsoluteEncoderConfig encoderConfig;
    private ClosedLoopConfig closedLoopConfig;
    private MAXMotionConfig maxMotionConfig;

    private SparkClosedLoopController steerPIDController;

    public NeoSteerMotor(int canId) {

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
                    //   .inverted(true)
                      .apply(encoderConfig);
                    //   .absoluteEncoder.setSparkMaxDataPortConfig().inverted(true);

        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        steerEncoder = motor.getAbsoluteEncoder();
        // motor.setCANTimeout(250);

        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); //increase position update frequency
        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); //increase velocity update frequency

        // motor.setSmartCurrentLimit(40);
        // motor.enableVoltageCompensation(12.0);

        // motor.burnFlash();


    }

    public void configurePID(double p, double i, double d, double ff){
        closedLoopConfig.pidf(p, i, d, ff);
        sparkMaxConfig.apply(closedLoopConfig);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerPIDController = motor.getClosedLoopController();
    }

    public void setPosition(double targetRads) {
        double targetDouble = (targetRads + Math.PI) / (2. * Math.PI);
        // System.out.println("error" + (targetDouble - steerEncoder.getPosition()));
        // System.out.println("current" + steerEncoder.getPosition());
        // System.out.println("volts" + motor.getOutputCurrent());
        steerPIDController.setReference(targetDouble, ControlType.kPosition);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public double getPosition(){
        return steerEncoder.getPosition();
    }
    
}
