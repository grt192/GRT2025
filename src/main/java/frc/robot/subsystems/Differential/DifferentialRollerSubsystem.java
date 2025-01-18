package frc.robot.subsystems.Differential;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//TODO set current limit

public class DifferentialRollerSubsystem { 

    private SparkMax rollerMotor;
    private double speed;
    private int motorID;

    

    public DifferentialRollerSubsystem(int motorID) {
        this.motorID = motorID;
        speed = 0;

        rollerMotor = new SparkMax(motorID, MotorType.kBrushless);
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
