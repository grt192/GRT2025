package frc.robot.subsystems.Intake.Roller;

import static frc.robot.Constants.DebugConstants.MASTER_DEBUG;
import static frc.robot.Constants.DebugConstants.ROLLER_DEBUG;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstans.RollerConstants;
import frc.robot.util.LoggedBooleanSensor;
import frc.robot.util.LoggedTalon;

public class RollerSubsystem extends SubsystemBase{
    private LoggedTalon rollerMotor;
    private LoggedBooleanSensor intakeSensor;

    private TalonFXConfiguration rollerConfig = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(RollerConstants.ROLLER_KP)
                .withKI(RollerConstants.ROLLER_KI)
                .withKD(RollerConstants.ROLLER_KD)
                .withKS(RollerConstants.ROLLER_KS)
                .withKV(RollerConstants.ROLLER_KV)
        )
        .withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
        )
        .withClosedLoopRamps(
            new ClosedLoopRampsConfigs()
                .withTorqueClosedLoopRampPeriod(RollerConstants.ROLLER_RAMP_RATE)    
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(RollerConstants.ROLLER_CURRENT_LIMIT)
        );

    public RollerSubsystem(){
        rollerMotor = new LoggedTalon(
            RollerConstants.ROLLER_CAN_ID, RollerConstants.ROLLER_CAN_NAME, rollerConfig
        );

        intakeSensor = new LoggedBooleanSensor(
            "Intake Distance Sensor", 1
        );
    }

    @Override
    public void periodic(){
        rollerMotor.logStats();
        intakeSensor.logStats();
        if(MASTER_DEBUG || ROLLER_DEBUG){
            rollerMotor.publishStats();
            intakeSensor.publishStats();
        }
    }

    public boolean getIntakeSensor() {
        return !intakeSensor.get();
    }

    /**
     * Set the speed of the roller motor
     * @param speed target speed in RPS
     */
    public void setRollerSpeed(double speed){
        rollerMotor.setSpeed(speed);
    }

    /**
     * Send duty cycle control reuest to roller motors
     * @param output
     */
    public void setDutyCycle(double output){
        rollerMotor.setDutyCycle(output);
    }

    public double getClosedLoopError(){
        return rollerMotor.getClosedLoopError();
    }
}
