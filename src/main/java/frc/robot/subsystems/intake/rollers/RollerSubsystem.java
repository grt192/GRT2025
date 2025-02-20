package frc.robot.subsystems.intake.rollers;

import static frc.robot.Constants.IntakeConstants.INTAKE_SENSOR_ID;
import static frc.robot.Constants.IntakeConstants.ROLLER_ID;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase{
    private final SparkMax rollerMotor;
    private final DigitalInput intakeSensor;

    public RollerSubsystem() {
        rollerMotor = new SparkMax(ROLLER_ID, MotorType.kBrushless);
        intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);
    }

    public void setRollerPower(double speed) {
        rollerMotor.set(speed);
    }

    public boolean getIntakeSensor() {
        return !intakeSensor.get();
    }

}
