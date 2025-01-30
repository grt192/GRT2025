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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DiffyConstants.DIFFY_D;
import static frc.robot.Constants.DiffyConstants.DIFFY_I;
import static frc.robot.Constants.DiffyConstants.DIFFY_P;
import static frc.robot.Constants.DiffyConstants.LEFT_ID;
import static frc.robot.Constants.DiffyConstants.RIGHT_ID;
import static frc.robot.Constants.DiffyConstants.DIFFY_CONVERSION_FACTOR ;
import static frc.robot.Constants.DiffyConstants.DIFFY_TOLERANCE;


public class DiffyArmSubsystem extends SubsystemBase{

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

        leftDiffyEncoder.setPosition(0);
        rightDiffyEncoder.setPosition(0);

        // Encoder Configs
        encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(DIFFY_CONVERSION_FACTOR);

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
                       .apply(diffyPIDConfig);
                    //    .apply(leftDiffySoftLimit);

        rightDiffyConfig = new SparkMaxConfig();
        rightDiffyConfig.apply(encoderConfig)
                        .apply(diffyPIDConfig);
                        // .apply(leftDiffySoftLimit);

        leftDiffyMotor.configure(leftDiffyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightDiffyMotor.configure(rightDiffyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftDiffyPID = leftDiffyMotor.getClosedLoopController();
        rightDiffyPID = rightDiffyMotor.getClosedLoopController();

    }
    }

    /* Get the radian position of both encoders
     * @returns two doubles, the first is left motor, the second is the right motor rads
     */
    public double[] getMotorPositions(){
        leftMotorPosition = leftDiffyEncoder.getPosition();
        rightMotorPosition = rightDiffyEncoder.getPosition();

        return new double[] {leftMotorPosition, rightMotorPosition};
    }

    /*
     * @returns the position of the wrist in radians
     */
    public double getDiffyWristPosition(){
        leftMotorPosition = leftDiffyEncoder.getPosition();
        rightMotorPosition = rightDiffyEncoder.getPosition();

        diffyWristPosition = ( Math.abs(leftMotorPosition - rightMotorPosition) / 2.);

        return diffyWristPosition;
    }

    /*
     * @returns the position of the arm in radians
     */
    public double getDiffyArmPosition(){
        leftMotorPosition = leftDiffyEncoder.getPosition();
        rightMotorPosition = rightDiffyEncoder.getPosition();

        diffyArmPosition = Math.abs(Math.max( leftMotorPosition, rightMotorPosition ) - getDiffyWristPosition() );
        return diffyArmPosition; 
    }

    // Setters

    public void setArmPosition(double armPosition) {
        leftTarget = armPosition - getDiffyWristPosition();
        rightTarget = armPosition + getDiffyWristPosition();
        setMotorPositions(leftTarget, rightTarget);
    }

    // public void setWristPosition(double wristPosition) {
    //     leftTarget = (getMotorPositions()[0] - wristPosition) * 2;
    //     rightTarget = (getMotorPositions()[1] + wristPosition) * 2;
    //     setMotorPositions(leftTarget / 2., rightTarget / 2.);
    // }

    public void setWristPosition(double wristPosition) {
        // Get the current positions of the left and right motors
        double leftCurrent = getMotorPositions()[0]; 
        double rightCurrent = getMotorPositions()[1];
    
        // Calculate the shortest rotation to the target position for both motors
        // The formula ensures that we always rotate the shortest way around a 360-degree system
        // 1. (wristPosition - leftCurrent): Finds the difference between target and current position
        // 2. +180 and %360: Wraps the angle to ensure it's always within -180 to +180 degrees
        // 3. -180: Ensures we get the shortest direction (clockwise or counterclockwise)
        // 4. *2: Scales the movement so both motors contribute equally to wrist rotation
        double leftTarget = ((wristPosition - leftCurrent + 180) % 360 - 180) * 2;
        double rightTarget = ((wristPosition - rightCurrent + 180) % 360 - 180) * 2;
    
        // Handle a special case where the calculated target is exactly -360 degrees
        // -360 degrees is mathematically the same as 0, but we convert it to +360 for consistency
        if (leftTarget == -360) leftTarget = 360;
        if (rightTarget == -360) rightTarget = 360;
    
        // Apply the motor targets, dividing by 2 to restore the correct scaling
        // This balances the movement, ensuring both motors move in opposite directions equally
        setMotorPositions(leftTarget / 2.0, rightTarget / 2.0);
    }

    private void setMotorPositions(double leftPosition, double rightPosition) {
        System.out.println(leftPosition);
        leftDiffyPID.setReference(leftPosition, ControlType.kPosition);
        rightDiffyPID.setReference(rightPosition, ControlType.kPosition);
    }
    
    public boolean atArmState(DiffyState state) {
        return Math.abs(getDiffyArmPosition() - state.getDiffyArmPos()) < DIFFY_TOLERANCE;
    }

    public boolean atWristState(DiffyState state) {
        return Math.abs(getDiffyWristPosition() - state.getDiffyWristPos()) < DIFFY_TOLERANCE;
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