package frc.robot.subsystems.DiffySubsystem;



import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.DiffyConstants.ARM_CONVERSION_FACTOR;
import static frc.robot.Constants.DiffyConstants.DIFFY_D;
import static frc.robot.Constants.DiffyConstants.DIFFY_I;
import static frc.robot.Constants.DiffyConstants.DIFFY_P;
import static frc.robot.Constants.DiffyConstants.LEFT_ID;
import static frc.robot.Constants.DiffyConstants.RIGHT_ID;
import static frc.robot.Constants.DiffyConstants.WRIST_CONVERSION_FACTOR ;


public class DiffyArmSubsystem {

    // SparkMax controls 
    private final SparkMax leftDiffyMotor; // Left can be seen in the view with the gears visible 
    private final SparkMax rightDiffyMotor;  // Right can be seen in the view with the gears visible 

    // Encoders
    private RelativeEncoder leftDiffyEncoder; 
    private RelativeEncoder rightDiffyEncoder;

    // Encoder Configs
    private EncoderConfig encoderConfig;

    // SparkMax Configs
    private SparkMaxConfig leftDiffyConfig;
    private SparkMaxConfig rightDiffyConfig;

    // PID Controls
    private SparkClosedLoopController leftDiffyPID;
    private SparkClosedLoopController rightDiffyPID;

    // PID Configs
    private ClosedLoopConfig diffyPIDConfig;

    // Soft limits
    private SoftLimitConfig leftDiffySoftLimit;
    private SoftLimitConfig rightDiffySoftLimit;

    private DiffyState targetState;
    private double leftTarget;
    private double rightTarget;

    // MotorPosition
    private double leftMotorPosition;
    private double rightMotorPosition;

    // DiffyPosition
    private double diffyWristPosition;
    private double diffyArmPosition;


    public DiffyArmSubsystem(){ {

        // Motor
        leftDiffyMotor = new SparkMax(LEFT_ID, MotorType.kBrushless);
        rightDiffyMotor = new SparkMax(RIGHT_ID, MotorType.kBrushless);
        leftDiffyEncoder = leftDiffyMotor.getEncoder();
        rightDiffyEncoder = rightDiffyMotor.getEncoder();

        // Encoder Configs
        encoderConfig = new EncoderConfig();

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
        diffyPIDConfig = new ClosedLoopConfig();

        diffyPIDConfig.pid(DIFFY_P, DIFFY_I, DIFFY_D);

        leftDiffyConfig = new SparkMaxConfig();
        leftDiffyConfig.apply(encoderConfig)
                       .apply(diffyPIDConfig)
                       .apply(leftDiffySoftLimit);

        rightDiffyConfig = new SparkMaxConfig();
        rightDiffyConfig.apply(encoderConfig)
                        .apply(diffyPIDConfig)
                        .apply(leftDiffySoftLimit);

        leftDiffyMotor.configure(leftDiffyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightDiffyMotor.configure(rightDiffyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftDiffyPID = leftDiffyMotor.getClosedLoopController();
        rightDiffyPID = rightDiffyMotor.getClosedLoopController();

    }
    }

    /* Get the RAW position of both encoders
     * @returns two doubles, the first is left motor, the second is the right motor ticks
     */
    public double[] getTickMotorPositions(){
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
     * @returns the position of the wrist in radians
     */
    public double getAngledDiffyWristPosition(){
        return getDiffyArmPosition() * WRIST_CONVERSION_FACTOR;
    }

    /*
     * @returns the position of the arm in ticks
     */
    public double getDiffyArmPosition(){
        leftMotorPosition = leftDiffyEncoder.getPosition();
        rightMotorPosition = rightDiffyEncoder.getPosition();

        diffyArmPosition = Math.abs(Math.max( leftMotorPosition, rightMotorPosition ) - getDiffyWristPosition() );
        return diffyArmPosition; 
    }

    /*
     * @returns the position of the arm in ticks
     */
    public double getAngledDiffyArmPosition(){
        return getDiffyArmPosition() * ARM_CONVERSION_FACTOR;
    }

    // Setters

    public void setArmPosition(double position) {
        setArmTickPosition(position / ARM_CONVERSION_FACTOR);
    }

    public void setWristPosition(double position) {
        setWristTickPosition(position / WRIST_CONVERSION_FACTOR);
    }

    /* 
     * @param position the position of the wrist in ticks
     */
    private void setWristTickPosition(double position){
        leftTarget = getTickMotorPositions()[0] - position;
        rightTarget = getTickMotorPositions()[1] + position;
        setMotorPositions(leftTarget, rightTarget);
    }

    /*
     * @param position the position of the arm in ticks
     */
    private void setArmTickPosition(double position){
        leftTarget = position - getDiffyWristPosition();
        rightTarget = position + getDiffyWristPosition();
        setMotorPositions(leftTarget, rightTarget);
    }

    private void setMotorPositions(double leftPosition, double rightPosition) {
        leftDiffyPID.setReference(leftPosition, ControlType.kPosition);
        rightDiffyPID.setReference(rightPosition, ControlType.kPosition);
    }

    // /*
    //  * Puts the diffy in the default position TO UPDATE
    //  */
    // public void diffydefPos(){
    //     setWristPosition(0);
    //     Timer.delay(0.5); // make sure diffy stops moving before moving arm (if play)
    //     setArmPosition(0);
    // }

}