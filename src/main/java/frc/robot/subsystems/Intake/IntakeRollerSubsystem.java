package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * DifferentialRollerSubsystem controls the roller motor and manages the intake sensor.
 */
public class IntakeRollerSubsystem extends SubsystemBase { 

    private final TalonFX rollerMotor;
    private double rollerSpeed;
    private final DigitalInput intakeSensor;

    /**
     * Constructor for DifferentialRollerSubsystem.
     */
    public IntakeRollerSubsystem() {
        rollerSpeed = Constants.DifferentialRollerConstants.ROLLER_SPEED;
        intakeSensor = new DigitalInput(0);
        rollerMotor = new TalonFX(Constants.DifferentialRollerConstants.ROLLER_MOTOR_ID);
    }

    /**
     * Gets the intake sensor value.
     * 
     * @return The value of the intake sensor.
     */
    public boolean getIntakeSensorValue() {
        return intakeSensor.get();
    }

    /**
     * Sets the speed of the roller motor.
     * 
     * @param speed The speed to set the roller motor to.
     */
    public void setSpeed(double speed) {
        rollerSpeed = speed;
        rollerMotor.set(rollerSpeed );
    }

    /**
     * Stops the roller motor.
     */
    public void stop() {
        rollerMotor.setVoltage(0);
        
    }

    /**
     * Gets the current speed of the roller motor.
     * 
     * @return The current speed of the roller motor.
     */
    public double getSpeed() {
        return rollerMotor.get();
    }

    /**
     * Runs the roller motor outwards at a predefined speed.
     */
    public void runOut() {
        setSpeed(rollerSpeed);
    }

    /**
     * Runs the roller motor inwards at a predefined speed.
     */
    public void runIn() {
        setSpeed(rollerSpeed);
    }
}
