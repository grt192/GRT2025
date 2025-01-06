package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

public class NeoSteerMotor {

    private final CANSparkMax motor;

    private final SparkAbsoluteEncoder steerEncoder;
    private final SparkPIDController steerPIDController;

    public NeoSteerMotor(int canId) {

        motor = new CANSparkMax(canId, MotorType.kBrushless);
        steerEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        steerEncoder.setAverageDepth(64); //tune value as needed

        steerPIDController = motor.getPIDController();

        // motor.setCANTimeout(250);

        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); //increase position update frequency
        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); //increase velocity update frequency

        // motor.setSmartCurrentLimit(40);
        // motor.enableVoltageCompensation(12.0);

        // motor.burnFlash();


    }

    public void configurePID(double p, double i, double d, double ff){
        steerPIDController.setFeedbackDevice(steerEncoder); //configure Steer to use the absolute encoder for closed loop PID feedback
        steerPIDController.setP(p);
        steerPIDController.setI(i);
        steerPIDController.setD(d);
        steerPIDController.setFF(ff);
        steerPIDController.setPositionPIDWrappingEnabled(true); //enable PID wrapping
            steerPIDController.setPositionPIDWrappingMinInput(0);
            steerPIDController.setPositionPIDWrappingMaxInput(1);
    
    }

    public void setPosition(double targetRads) {
        double targetDouble = (targetRads + Math.PI) / (2. * Math.PI);
        System.out.println("target" + targetDouble);
        System.out.println("current" + steerEncoder.getPosition());
        steerPIDController.setReference(0.5, ControlType.kPosition);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public double getPosition(){
        return steerEncoder.getPosition();
    }
    
}
