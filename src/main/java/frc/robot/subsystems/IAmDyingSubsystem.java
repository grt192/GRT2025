package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IAmDyingSubsystem extends SubsystemBase {
    public final CANSparkMax motor;
    private final SparkAbsoluteEncoder steerEncoder;
    private final SparkPIDController steerPIDController;

    public IAmDyingSubsystem() {
        motor = new CANSparkMax(2, MotorType.kBrushless);
        steerEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        steerEncoder.setAverageDepth(128); //tune value as needed

        steerPIDController = motor.getPIDController();
        // steerPIDController.();
    }
    
    public void configurePID(double p, double i, double d, double ff){
        steerPIDController.setFeedbackDevice(steerEncoder); //configure Steer to use the absolute encoder for closed loop PID feedback
        steerPIDController.setP(p);
        steerPIDController.setI(i);
        steerPIDController.setD(d);
    
    }

    public void setPosition(double tick) {
        System.out.println(steerEncoder.getPosition());
        steerPIDController.setReference(tick, ControlType.kPosition);
    }
}
