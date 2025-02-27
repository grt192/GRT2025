package frc.robot.subsystems.Intake.Pivot;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.IntakeConstans.PivotConstants;
import frc.robot.util.LoggedTalon;

public class PivotSubsystem extends SubsystemBase{
    
    private LoggedTalon pivotMotor;
    private double arbFF;

    ArmFeedforward feedforward = new ArmFeedforward(
        PivotConstants.PIVOT_KS,
        PivotConstants.PIVOT_KG,
        PivotConstants.PIVOT_KV);


    private TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withKP(PivotConstants.PIVOT_KP)
                .withKI(PivotConstants.PIVOT_KI)
                .withKD(PivotConstants.PIVOT_KD)
        )
        .withClosedLoopRamps(
            new ClosedLoopRampsConfigs()
                .withTorqueClosedLoopRampPeriod(PivotConstants.PIVOT_RAMP_RATE)    
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT)
        )
        .withFeedback(
            new FeedbackConfigs()
                // .withRotorToSensorRatio(PivotConstants.ROTOR_TO_SENSOR_RATIO)
                .withSensorToMechanismRatio(PivotConstants.ROTOR_TO_SENSOR_RATIO)
        )
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(PivotConstants.PIVOT_MAX_POS)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(PivotConstants.PIVOT_MIN_POS)
        );


    public PivotSubsystem(){
        pivotMotor = new LoggedTalon(
            PivotConstants.PIVOT_CAN_ID, PivotConstants.PIVOT_CAN_NAME, pivotConfig
        );
        pivotMotor.setPosition(PivotConstants.PIVOT_INIT_POS);
    }

    @Override
    public void periodic(){
        pivotMotor.logStats();
        if(DebugConstants.MASTER_DEBUG || DebugConstants.PIVOT_DEBUG){
            pivotMotor.publishStats();
        }
        System.out.println(Units.radiansToDegrees(pivotMotor.getPosition()));
    }

    public void setPositionReferenceWithVoltage(double position){
        pivotMotor.setPositionReferenceWithVoltage(position);
    }

    /**
     * Sets the elevator to a specific position.
     * @param positionReference target position
     */
    public void setPositionReference(double positionReference){
        if (positionReference > pivotMotor.getPosition()) {
            arbFF = feedforward.calculate(positionReference, 5);
        }
        else {
            arbFF = feedforward.calculate(positionReference, .1, .01);
        }
        pivotMotor.setPositionReferenceWithArbFF(positionReference, arbFF);
    }

    public void setVelocityReferenceWithVoltage(double velocity){
        pivotMotor.setVelocityReferenceWithVoltage(velocity);
    }

    public void setVelocityReference(double velocity){
        pivotMotor.setVelocityReference(velocity);
    }

    public double getClosedLoopError(){
        return pivotMotor.getClosedLoopError();
    }

    public double getPosition() {
        return pivotMotor.getPosition();
    }

    public void setPower(double speed) {
        pivotMotor.setPower(speed);
    }
}
