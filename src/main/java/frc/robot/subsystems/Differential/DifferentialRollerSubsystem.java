package frc.robot.subsystems.Differential;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO set current limit

public class DifferentialRollerSubsystem { 

    private SparkMax rollerMotor;
    private double speed;
    private int motorID;
    private final DigitalInput intakeSensor;


    

    public DifferentialRollerSubsystem(int motorID) {
        this.motorID = motorID;
        
        speed = 0;

        intakeSensor = new DigitalInput(0);

        rollerMotor = new SparkMax(motorID, MotorType.kBrushless);
    }

    /**
     * Gets the back sensor value.
     *
     * @return If the sensor detects changes (boolean).
     */
    public boolean getIntakeRollerSensorValue() {
        return intakeSensor.get();
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.setVoltage(motorID);

    }

    public double getSpeed() {
        return rollerMotor.get();
    }



}
