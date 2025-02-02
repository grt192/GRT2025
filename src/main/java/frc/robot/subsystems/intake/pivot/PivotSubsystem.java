package frc.robot.subsystems.Intake.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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


public class PivotSubsystem extends SubsystemBase{

    private SparkMax pivotMotor;
    private RelativeEncoder pivotEncoder; 
    private SparkClosedLoopController pivotPID;

    private EncoderConfig encoderConfig;
    private ClosedLoopConfig closedLoopConfig;
    private SparkMaxConfig sparkMaxConfig;
    private SoftLimitConfig softLimitConfig;

    private ArmFeedforward armFeedforward;

    private PivotState targetState;
    private double gravityFF;

    public PivotSubsystem(){

        pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0);

        encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(1./ PIVOT_CONVERSION_FACTOR);

        softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true)
                       .forwardSoftLimit(Units.degreesToRadians(90))
                       .reverseSoftLimitEnabled(true)
                       .reverseSoftLimit(0);

        closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(PIVOT_P, PIVOT_I, PIVOT_D, ClosedLoopSlot.kSlot0);

        armFeedforward = new ArmFeedforward(PIVOT_KS, PIVOT_KG, PIVOT_KV);

        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.apply(closedLoopConfig)
                      .apply(encoderConfig);
                    //   .apply(softLimitConfig);

        pivotMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotPID = pivotMotor.getClosedLoopController();

        targetState = PivotState.ZERO;
        gravityFF = 0;

    }

    public double getCurrentAngle() {
        return pivotEncoder.getPosition();
    }

    public PivotState getTargetState() {
        return targetState;
    }

    public void setState(PivotState targetState) {
        gravityFF = armFeedforward.calculate(targetState.getTargetAngle(), 2);
        pivotPID.setReference(targetState.getTargetAngle(), ControlType.kPosition, ClosedLoopSlot.kSlot0, gravityFF, ArbFFUnits.kVoltage);
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
    }

