package frc.robot.subsystems.DiffySubsystem;



import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.diffyConst.ARM_CONVERSION_FACTOR;
import static frc.robot.Constants.diffyConst.WRIST_CONVERSION_FACTOR ;


public class DiffyArmSubsystem {

    // SparkMax controls 
    private final SparkMax leftDiffyMotor; // Left can be seen in the view with the gears visible 
    private final SparkMax rightDiffyMotor;  // Right can be seen in the view with the gears visible 

    // Encoders
    private RelativeEncoder leftDiffyEncoder; 
    private RelativeEncoder rightDiffyEncoder;

    // Encoder Configs
    private EncoderConfig leftDiffyEncoderConfig;
    private EncoderConfig rightDiffyEncoderConfig;

    // SparkMax Configs
    private SparkMaxConfig leftDiffyConfig;
    private SparkMaxConfig rightDiffyConfig;

    // PID Controls
    private SparkClosedLoopController leftDiffyPID;
    private SparkClosedLoopController rightDiffyPID;

    // PID coefficients
    private double kP = 0.1;
    private double kI = 0.0;
    private double kD = 0.0;

    // PID Configs
    private ClosedLoopConfig leftDiffyPIDConfig;
    private ClosedLoopConfig rightDiffyPIDConfig;

    // Soft limits
    private SoftLimitConfig leftDiffySoftLimit;
    private SoftLimitConfig rightDiffySoftLimit;

    private DiffyState targetState;

    // MotorPosition
    private double leftMotorPosition;
    private double rightMotorPosition;

    // DiffyPosition
    private double diffyWristPosition;
    private double diffyArmPosition;


    public DiffyArmSubsystem(int canId) {

        // Motor
        leftDiffyMotor = new SparkMax(canId, MotorType.kBrushless);
        rightDiffyMotor = new SparkMax(canId, MotorType.kBrushless);
        leftMotorPosition = leftDiffyEncoder.getPosition();
        rightMotorPosition = rightDiffyEncoder.getPosition();

        // Encoder Configs
        leftDiffyEncoderConfig = new EncoderConfig();
        rightDiffyEncoderConfig = new EncoderConfig();
            // Conversion factor for gear ratio
        leftDiffyEncoderConfig.positionConversionFactor(ARM_CONVERSION_FACTOR);
        rightDiffyEncoderConfig.positionConversionFactor(ARM_CONVERSION_FACTOR);

        // Soft Limit
        leftDiffySoftLimit = new SoftLimitConfig();
        rightDiffySoftLimit = new SoftLimitConfig();

        // Soft limit config
        leftDiffySoftLimit.forwardSoftLimitEnabled(true)
        .forwardSoftLimit(Units.degreesToRadians(90))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0);

        rightDiffySoftLimit.forwardSoftLimitEnabled(true)
        .forwardSoftLimit(Units.degreesToRadians(90))
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0);

        // PID Configs
        leftDiffyPIDConfig = new ClosedLoopConfig();
        rightDiffyPIDConfig = new ClosedLoopConfig();

        leftDiffyPIDConfig.pid(kP, kI, kD);
        rightDiffyPIDConfig.pid(kP, kI, kD);

        leftDiffyMotor.configure(leftDiffyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightDiffyMotor.configure(rightDiffyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftDiffyPID = leftDiffyMotor.getClosedLoopController();
        rightDiffyPID = rightDiffyMotor.getClosedLoopController();

        // Diffy Positions
        diffyWristPosition = getDiffyArmPosition();
        diffyArmPosition = getDiffyArmPosition();
    }

    // Positions

    /* Turn motor ticks to degrees
     * @param position the position of the wrist in ticks
     */
    public double ticksToDegrees(double ticks){
        return ticks * 360;
    }

    /* Get the position of both encoders
     * @returns two doubles, the first is left motor, the second is the right motor ticks
     */
    public double[] getDifferentialPosition(){
        leftMotorPosition = leftDiffyEncoder.getPosition();
        rightMotorPosition = rightDiffyEncoder.getPosition();

        return new double[] {leftMotorPosition, rightMotorPosition};
    }

    /*
     * @returns the position of the wrist in ticks
     */
    public double getDiffyWristPosition(){
        leftMotorPosition = leftDiffyEncoder.getPosition();
        rightMotorPosition = rightDiffyEncoder.getPosition();

        diffyWristPosition = ( Math.abs(leftMotorPosition - rightMotorPosition) / 2);

        return diffyWristPosition;
    }

    /*
     * @returns the position of the arm in ticks
     */
    public double getDiffyArmPosition(){
        leftMotorPosition = leftDiffyEncoder.getPosition();
        rightMotorPosition = rightDiffyEncoder.getPosition();

        diffyArmPosition = Math.abs( Math.max( leftMotorPosition, rightMotorPosition ) - getDiffyWristPosition() );
        return diffyArmPosition; 
    }

    // Setters

    /* 
     * @param position the position of the wrist in degrees
     */
    public void setWristPosition(double position){

        leftDiffyEncoderConfig.positionConversionFactor(WRIST_CONVERSION_FACTOR);
        rightDiffyEncoderConfig.positionConversionFactor(WRIST_CONVERSION_FACTOR);

        leftDiffyPID.setReference(( position/360 ), ControlType.kPosition);
        rightDiffyPID.setReference(( - position/360 ), ControlType.kPosition);
        
    }

    /*
     * @param position the position of the arm in degrees
     */
    public void setArmPosition(double position){


        leftDiffyEncoderConfig.positionConversionFactor(ARM_CONVERSION_FACTOR);
        rightDiffyEncoderConfig.positionConversionFactor(ARM_CONVERSION_FACTOR);

        leftDiffyPID.setReference(( position/360 ), ControlType.kPosition);
        rightDiffyPID.setReference(( position/360 ), ControlType.kPosition);
        
    }

    /*
     * Puts the diffy in the default position
     */
    public void diffydefPos(){
        setWristPosition(0);
        Timer.delay(0.5); // make sure diffy stops moving before moving arm (if play)
        setArmPosition(0);
    }
}
