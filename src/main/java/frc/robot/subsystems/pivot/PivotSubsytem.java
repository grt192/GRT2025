package frc.robot.subsystems.pivot;

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
import com.revrobotics.spark.SparkClosedLoopController;

import static frc.robot.Constants.IntakeConstants.PIVOT_CONVERSION_FACTOR;
import static frc.robot.Constants.IntakeConstants.PIVOT_ID;


public class PivotSubsytem extends SubsystemBase{

    private SparkMax pivotMotor;
    private RelativeEncoder pivotEncoder; 
    private SparkClosedLoopController pivotPID;

    private EncoderConfig encoderConfig;
    private ClosedLoopConfig closedLoopConfig;
    private SparkMaxConfig sparkMaxConfig;
    private SoftLimitConfig softLimitConfig;

    private double p = 1; 
    private double i = 0; 
    private double d = 0;

    public void PivotSubsystem(){

        pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(PIVOT_CONVERSION_FACTOR);

        softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true)
                       .forwardSoftLimit(Units.degreesToRadians(90))
                       .reverseSoftLimitEnabled(true)
                       .reverseSoftLimit(0);

        closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.pid(p, i, d);

        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.apply(closedLoopConfig)
                      .apply(encoderConfig)
                      .apply(softLimitConfig);

        pivotMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotPID = pivotMotor.getClosedLoopController();

    }

    public double getPosition() {
        return pivotEncoder.getPosition();
    }

    public void setState(PivotState state) {
        pivotPID.setReference(state.getTargetAngle(), ControlType.kPosition);
    }
    }

