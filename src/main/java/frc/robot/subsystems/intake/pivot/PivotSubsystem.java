package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.Motors.LoggedSparkMax;
import frc.robot.util.Motors.LoggedSparkMaxConfig;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import static frc.robot.Constants.IntakeConstants.PIVOT_CONVERSION_FACTOR;
import static frc.robot.Constants.IntakeConstants.PIVOT_D;
import static frc.robot.Constants.IntakeConstants.PIVOT_I;
import static frc.robot.Constants.IntakeConstants.PIVOT_ID;
import static frc.robot.Constants.IntakeConstants.PIVOT_KG;
import static frc.robot.Constants.IntakeConstants.PIVOT_KS;
import static frc.robot.Constants.IntakeConstants.PIVOT_KV;
import static frc.robot.Constants.IntakeConstants.PIVOT_P;
import static frc.robot.Constants.IntakeConstants.PIVOT_TOLERANCE;

import java.util.Optional;
import java.util.OptionalLong;


public class PivotSubsystem extends SubsystemBase{

    private LoggedSparkMax pivotMotor;
    private RelativeEncoder pivotEncoder; 
    private SparkClosedLoopController pivotPID;

    private ArmFeedforward armFeedforward;

    private PivotState targetState;
    private double gravityFF;

    public PivotSubsystem(){
        pivotMotor = new LoggedSparkMax(IntakeConstants.PivotMotorLoggedSparkMaxConfig);
        targetState = PivotState.ZERO;
        pivotMotor.setPosition(Units.degreesToRadians(82));
        armFeedforward = new ArmFeedforward(PIVOT_KS, PIVOT_KG, PIVOT_KV);
        gravityFF = 0;
    }

    public double getCurrentAngle() {
        return pivotMotor.getPosition();
    }

    public PivotState getTargetState() {
        return targetState;
    }

    public void setState(PivotState targetState) {
        if (getCurrentAngle() < targetState.getTargetAngle()) {
            gravityFF = armFeedforward.calculate((targetState.getTargetAngle()), 1.5);
            // System.out.println(targetState.getTargetAngle() + Units.degreesToRadians(90));
        }
        else {
            gravityFF = 0;
        }
        pivotMotor.setReference(targetState.getTargetAngle(), ControlType.kPosition, ClosedLoopSlot.kSlot0, gravityFF, ArbFFUnits.kVoltage);
        // System.out.println("state set to : " + targetState.getTargetAngle());
        this.targetState = targetState;
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public boolean atState(PivotState state) {
        return Math.abs(getCurrentAngle() - state.getTargetAngle()) < PIVOT_TOLERANCE;
    }

    public double getPositionError() {
        return Math.abs(getCurrentAngle() - targetState.getTargetAngle());
    }

    @Override
    public void periodic() {
        pivotMotor.publishStats();
    }

    }

