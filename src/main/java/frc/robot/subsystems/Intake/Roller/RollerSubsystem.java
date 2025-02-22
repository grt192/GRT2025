package frc.robot.subsystems.Intake.Roller;

import static frc.robot.Constants.DebugConstants.MASTER_DEBUG;
import static frc.robot.Constants.DebugConstants.ROLLER_DEBUG;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstans.RollerConstants;
import frc.robot.util.LoggedTalon;

public class RollerSubsystem extends SubsystemBase{
    private LoggedTalon rollerMotor;

    private TalonFXConfiguration rollerConfig = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(RollerConstants.ROLLER_KP)
                .withKI(RollerConstants.ROLLER_KI)
                .withKD(RollerConstants.ROLLER_KD)
                .withKS(RollerConstants.ROLLER_KS)
                .withKV(RollerConstants.ROLLER_KV)
        )
        .withClosedLoopRamps(
            new ClosedLoopRampsConfigs()
                .withTorqueClosedLoopRampPeriod(RollerConstants.ROLLER_RAMP_RATE)    
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(RollerConstants.ROLLER_CURRENT_LIMIT)
        );

    public RollerSubsystem(int canId, String canBusName){
        rollerMotor = new LoggedTalon(canId, canBusName, rollerConfig);
    }

    @Override
    public void periodic(){
        rollerMotor.logStats();
        if(MASTER_DEBUG || ROLLER_DEBUG){
            rollerMotor.publishStats();
        }
        rollerMotor.logStats();
    }
    /**
     * Set the speed of the roller motor
     * @param speed target speed in RPS
     */
    public void setRollerSpeed(double speed){
        rollerMotor.setVelocityReference(speed);
    }

    public double getClosedLoopError(){
        return rollerMotor.getClosedLoopError();
    }
}
