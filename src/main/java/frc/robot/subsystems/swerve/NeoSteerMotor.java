package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import static frc.robot.Constants.LoggingConstants.SWERVE_TABLE;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

public class NeoSteerMotor {

    private final SparkMax motor;
    private final SparkAbsoluteEncoder steerEncoder;
    
    private SparkBaseConfig sparkMaxConfig;
    private AbsoluteEncoderConfig encoderConfig;
    private ClosedLoopConfig closedLoopConfig;
    private MAXMotionConfig maxMotionConfig;

    private SparkClosedLoopController steerPIDController;

    private NetworkTableInstance ntInstance; 
    private NetworkTable swerveStatsTable; 
    private DoublePublisher neoPositionPublisher;
    private DoublePublisher neoSetPositionPublisher; 
    private double targetDouble = 0;

    /**
     * A Neo steer motor for swerve steering
     * @param canId the motor's CAN ID
     */
    public NeoSteerMotor(int canId) {

        motor = new SparkMax(canId, MotorType.kBrushless);

        encoderConfig = new AbsoluteEncoderConfig();
        encoderConfig.inverted(true);

        maxMotionConfig = new MAXMotionConfig();
        maxMotionConfig.allowedClosedLoopError(.005);

        closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .positionWrappingEnabled(true)
                        .positionWrappingMinInput(0)
                        .positionWrappingMaxInput(1)
                        .apply(maxMotionConfig);
        
        sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.apply(closedLoopConfig)
                      .apply(encoderConfig);

        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        steerEncoder = motor.getAbsoluteEncoder();
        // motor.setCANTimeout(250);

        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); //increase position update frequency
        // motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); //increase velocity update frequency

        // motor.setSmartCurrentLimit(40);
        // motor.enableVoltageCompensation(12.0);

        // motor.burnFlash();
    }

    /**
     * Configures the motor's PID
     * @param p kP
     * @param i kI
     * @param d kD
     * @param ff kFF
     */
    public void configurePID(double p, double i, double d, double ff){
        closedLoopConfig.pidf(p, i, d, ff);
        sparkMaxConfig.apply(closedLoopConfig);
        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerPIDController = motor.getClosedLoopController();
    }

    /**
     * Using PID to move to target position
     * @param targetRads target position in radiants
     */
    public void setPosition(double targetRads) {
        double targetDouble = (targetRads + Math.PI) / (2. * Math.PI);
        steerPIDController.setReference(targetDouble, ControlType.kPosition);
    }

    /**
     * Gets the motor's position through the absolute encoder
     * @return position in double from 0 to 1
     */
    public double getPosition(){
        return steerEncoder.getPosition();
    }

    /**
     * Initializes NetworkTables
     * @param canId
     */
    public void initNT (int canId){
        ntInstance = NetworkTableInstance.getDefault();
        swerveStatsTable = ntInstance.getTable(SWERVE_TABLE); 
        neoPositionPublisher = swerveStatsTable.getDoubleTopic(canId + "neoPosition").publish(); 
        neoSetPositionPublisher = swerveStatsTable.getDoubleTopic(canId + "neoSetPosition").publish();
    }

    /**
     * Publishes Neo stats to NT
     */
    public void publishStats(){
        neoPositionPublisher.set(getPosition());
        neoSetPositionPublisher.set(targetDouble);
    }
}
