package frc.robot.util.Motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.util.GRTUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

public class LoggedSparkMax {
    
    private final SparkMax motor;
    private final SparkMaxConfig sparkMaxConfig;
    private final ClosedLoopConfig closedLoopConfig;
    private final RelativeEncoder encoder;
    private SparkClosedLoopController closedLoopController;
    private double targetReference;

    private NetworkTableInstance ntInstance; 
    private NetworkTable motorStatsTable; 
    private DoublePublisher neoPositionPublisher;
    private DoublePublisher neoSetPositionPublisher; 

    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry targetPositionLogEntry;
    private DoubleLogEntry busVoltageLogEntry;
    private DoubleLogEntry outputCurrtLogEntry;
    private DoubleLogEntry temperatureLogEntry;
    private DoubleLogEntry appliedOutputLogEntry;

    public LoggedSparkMax(LoggedSparkMaxConfig config) {
        motor = new SparkMax(config.getCanId(), MotorType.kBrushless);
        sparkMaxConfig = config.getSparkMaxConfig();
        closedLoopConfig = config.getClosedLoopConfig();
        motor.configure(sparkMaxConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        );
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        initLogs(config.getCanId());
        intiNT(config.getCanId());
    }

    /**
     * Configures the PIDF values for the motor
     * @param p kP value
     * @param i kI value
     * @param d kD value
     * @param f kF value
     */
    public void configurePIDF(double p, double i, double d, double f) {
        closedLoopConfig.pidf(p, i, d, f);
        sparkMaxConfig.apply(closedLoopConfig);
        motor.configure(sparkMaxConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        );
        closedLoopController = motor.getClosedLoopController();
    }

    /**
     * Sets the target reference for the motor
     * @param value value of the reference
     * @param controlType type of control (position, velocity, etc.)
     */
    public void setReference(double value, ControlType controlType){
        targetReference = value;
        closedLoopController.setReference(value, controlType);
    }

    /**
     * Sets the target reference for the motor
     * @param value value of the reference
     * @param controlType type of control (position, velocity, etc.)
     * @param closedLoopSlot closed loop slot
     * @param arbFF arbitrary feed forward
     * @param arbFFUnits arbitrary feed forward units
     */
    public void setReference(double value, ControlType controlType,
        ClosedLoopSlot closedLoopSlot, double arbFF,
        ArbFFUnits arbFFUnits
    ){
        targetReference = value;
        closedLoopController.setReference(value, ControlType.kPosition,
            closedLoopSlot, arbFF, arbFFUnits
        );
    }

    /**
     * Sets the motor's speed to a specific value
     * @param value value of the speed from -1.0 to 1.0
     */
    public void set(double value){
        motor.set(value);
    }

    /**
     * Gets the position of the motor in rotations after taking the position
     * conversion factor into account
     * @return position of the motor in rotations
     */
    public double getPosition(){
        return encoder.getPosition();
    }

    /**
     * Initializes NetworkTables 
     * @param canId motor's CAN ID
     */
    private void intiNT(int canId) {
        ntInstance = NetworkTableInstance.getDefault();
        motorStatsTable = ntInstance.getTable("MotorStats"); 
        neoPositionPublisher = motorStatsTable.getDoubleTopic(
            canId + "neoPosition"
        ).publish(); 
        neoSetPositionPublisher = motorStatsTable.getDoubleTopic(
            canId + "neoSetPosition"
        ).publish();
    }

    /**
     * Initializes logs
     * @param canId motor's CAN ID
     */
    private void initLogs(int canId){
        positionLogEntry =
            new DoubleLogEntry(DataLogManager.getLog(), canId + "position");
        
        targetPositionLogEntry =
            new DoubleLogEntry(DataLogManager.getLog(), canId + "targetPosition");

        busVoltageLogEntry =
            new DoubleLogEntry(DataLogManager.getLog(), canId + "busVoltage");
        
        outputCurrtLogEntry =
            new DoubleLogEntry(DataLogManager.getLog(), canId + "outputCurrent");
        
        temperatureLogEntry =
            new DoubleLogEntry(DataLogManager.getLog(), canId + "temperature");
        
        appliedOutputLogEntry =
            new DoubleLogEntry(DataLogManager.getLog(), canId + "appliedOutput");
    }

    /**
     * Publishes Neo stats to NT
     */
    public void publishStats(){
        neoPositionPublisher.set(getPosition());
        neoSetPositionPublisher.set(targetReference);
    }

    /**
     * Logs Neo stats into log file
     */
    public void logStats(){
        positionLogEntry.append(getPosition(), GRTUtil.getFPGATime());
        targetPositionLogEntry.append(targetReference, GRTUtil.getFPGATime());
        busVoltageLogEntry.append(motor.getBusVoltage(), GRTUtil.getFPGATime());

        outputCurrtLogEntry.append(
            motor.getOutputCurrent(), GRTUtil.getFPGATime()
        );

        temperatureLogEntry.append(
            motor.getMotorTemperature(), GRTUtil.getFPGATime()
        );

        appliedOutputLogEntry.append(
            motor.getAppliedOutput(), GRTUtil.getFPGATime()
        );
    }
}
