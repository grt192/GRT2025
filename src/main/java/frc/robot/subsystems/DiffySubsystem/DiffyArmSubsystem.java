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

    /*
     * Sets the position of the arm in radians
     * @param target armPosition in radians
     */
    public void setArmPosition(double armPosition) {
        leftTarget = armPosition - getDiffyWristPosition();
        rightTarget = armPosition + getDiffyWristPosition();
        setMotorPositions(leftTarget, rightTarget);
    }

    public void setWristPosition(double wristPosition) { //this does not work  for a very abvious reason (offset does not take into account current position)
        leftTarget = (getMotorPositions()[0] + wristPosition);
        rightTarget = (getMotorPositions()[1] - wristPosition);
        setMotorPositions(leftTarget, rightTarget);
    }


    /*
     * Sets the position of the wrist in radians
     * @param target wristPosition in radians
     * basicaly how it works is it gets the diffrence then adds 180 to make it positive then %360 to make it between 0 and 360 then -180 to make it between -180 and 180 then *2 to make it between -360 and 360
     */
    // public void setWristPosition(double wristPosition) {
    //     leftMotorPosition = leftDiffyEncoder.getPosition();
    //     rightMotorPosition = rightDiffyEncoder.getPosition();

    // }




    /*
     * Sets the position of the motors in radians
     * @param leftPosition left motor position in radians
     * @param rightPosition right motor position in radians
     */
    private void setMotorPositions(double leftPosition, double rightPosition) {
        leftDiffyPID.setReference(leftPosition, ControlType.kPosition);
        rightDiffyPID.setReference(rightPosition, ControlType.kPosition);
    }
    

    /*
     * view the current state of the arm state
     */
    public boolean atArmState(DiffyState state) {
        return Math.abs(getDiffyArmPosition() - state.getDiffyArmPos()) < DIFFY_TOLERANCE;
    }

    /*
     * view the current state of the wrist state
     */
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