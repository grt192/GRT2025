package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

public class NeoSteerMotor {

    private final SparkMax motor;

    private final SparkAbsoluteEncoder steerEncoder;
    private final SparkBaseConfig sparkMaxConfig;
    private final EncoderConfig encoderConfig;
    private final ClosedLoopConfig closedLoopConfig;

    private final SparkClosedLoopController steerPIDController;

    public NeoSteerMotor(int canId) {

        motor = new SparkMax(canId, MotorType.kBrushless);
        steerPIDController = motor.getClosedLoopController();

        encoderConfig = new EncoderConfig();
        encoderConfig.inverted(true);

        closedLoopConfig = new ClosedLoopConfig();

        steerEncoder = motor.getAbsoluteEncoder();
        
        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.apply(encoderConfig);

        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // motor.setCANTimeout(250);

        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); //increase position update frequency
        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); //increase velocity update frequency

        // motor.setSmartCurrentLimit(40);
        // motor.enableVoltageCompensation(12.0);

        // motor.burnFlash();


    }

    public void configurePID(double p, double i, double d, double ff){
        closedLoopConfig.pid(p, i, d);
        sparkMaxConfig.apply(closedLoopConfig);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPosition(double targetRads) {
        double targetDouble = (targetRads + Math.PI) / (2. * Math.PI);
        System.out.println("target" + targetDouble);
        System.out.println("current" + steerEncoder.getPosition());
        steerPIDController.setReference(targetDouble, ControlType.kPosition);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public double getPosition(){
        return steerEncoder.getPosition();
    }
    
}
