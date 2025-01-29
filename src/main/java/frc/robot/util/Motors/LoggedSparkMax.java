package frc.robot.util.Motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.util.GRTUtil;

import static frc.robot.Constants.LoggingConstants.REV_TABLE;
import static frc.robot.Constants.DebugConstants.REV_DEBUG;

import java.util.EnumSet;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

public class LoggedSparkMax {
    
    private final int canId;
    private final SparkMax motor;
    private final SparkMaxConfig sparkMaxConfig;
    private final ClosedLoopConfig closedLoopConfig;
    private final RelativeEncoder encoder;
    private SparkClosedLoopController closedLoopController;
    private double targetReference;

    private NetworkTableInstance ntInstance; 
    private NetworkTable motorStatsTable; 
    private DoublePublisher positionPublisher;
    private DoublePublisher velocityPublisher;
    private DoublePublisher busVoltagePublisher;
    private DoublePublisher outputCurrentPublisher;
    private DoublePublisher appliedOutputPublisher;
    private DoublePublisher temperaturePublisher;
    private DoublePublisher targetReferencePublisher;

    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry velocityLogEntry;
    private DoubleLogEntry busVoltageLogEntry;
    private DoubleLogEntry outputCurrtLogEntry;
    private DoubleLogEntry appliedOutputLogEntry;
    private DoubleLogEntry temperatureLogEntry;
    private DoubleLogEntry targetReferenceLogEntry;

    public LoggedSparkMax(LoggedSparkMaxConfig config) {
        canId = config.getCanId();
        motor = new SparkMax(canId, MotorType.kBrushless);
        sparkMaxConfig = config.getSparkMaxConfig();
        closedLoopConfig = config.getClosedLoopConfig();
        motor.configure(sparkMaxConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        );
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        initLogs();
        intiNT();
        if(REV_DEBUG){
            enableDebug();
        }
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
     * Gets the velocity of the motor in RPM after taking the velocity
     * conversion factor into account
     */
    public double getVelocity(){
        return encoder.getVelocity();
    }

    /**
     * Initializes NetworkTables 
     * @param canId motor's CAN ID
     */
    private void intiNT() {
        ntInstance = NetworkTableInstance.getDefault();
        motorStatsTable = ntInstance.getTable(REV_TABLE); 
        positionPublisher = motorStatsTable.getDoubleTopic(
            canId + "Position"
        ).publish(); 
        velocityPublisher = motorStatsTable.getDoubleTopic(
            canId + "Velocity"
        ).publish();
        busVoltagePublisher = motorStatsTable.getDoubleTopic(
            canId + "BusVoltage"
        ).publish();
        outputCurrentPublisher = motorStatsTable.getDoubleTopic(
            canId + "OutputCurrent"
        ).publish();
        appliedOutputPublisher = motorStatsTable.getDoubleTopic(
            canId + "AppliedOutput"
        ).publish();
        temperaturePublisher = motorStatsTable.getDoubleTopic(
            canId + "Temperature"
        ).publish();
        targetReferencePublisher = motorStatsTable.getDoubleTopic(
            canId + "TargetReference"
        ).publish();
    }

    /**
     * Initializes logs
     * @param canId motor's CAN ID
     */
    private void initLogs(){
        positionLogEntry =
            new DoubleLogEntry(DataLogManager.getLog(), canId + "position");
        
        targetReferenceLogEntry=
            new DoubleLogEntry(DataLogManager.getLog(), canId + "targetReference");

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
     * Allows for PIDF tuning in NT
     */
    private void enableDebug(){
        motorStatsTable.getDoubleArrayTopic(canId + "PIDF").publish().set(
            new double[] {0., 0., 0., 0.}
        );
        motorStatsTable.addListener(canId + "PIDF", 
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table, key, event) -> {
                configurePIDF(
                    event.valueData.value.getDoubleArray()[0],
                    event.valueData.value.getDoubleArray()[1],
                    event.valueData.value.getDoubleArray()[2],
                    event.valueData.value.getDoubleArray()[3]
                );
            }
        );
    }

    /**
     * Publishes  stats to NT
     */
    public void publishStats(){
        positionPublisher.set(getPosition());
        velocityPublisher.set(getVelocity());
        busVoltagePublisher.set(motor.getBusVoltage());
        outputCurrentPublisher.set(motor.getOutputCurrent());
        appliedOutputPublisher.set(motor.getAppliedOutput());
        temperaturePublisher.set(motor.getMotorTemperature());
        targetReferencePublisher.set(targetReference);
    }

    /**
     * Logs  stats into log file
     */
    public void logStats(){
        positionLogEntry.append(getPosition(), GRTUtil.getFPGATime());
        velocityLogEntry.append(getVelocity(), GRTUtil.getFPGATime());
        busVoltageLogEntry.append(motor.getBusVoltage(), GRTUtil.getFPGATime());
        outputCurrtLogEntry.append(
            motor.getOutputCurrent(), GRTUtil.getFPGATime()
        );
        appliedOutputLogEntry.append(
            motor.getAppliedOutput(), GRTUtil.getFPGATime()
        );
        temperatureLogEntry.append(
            motor.getMotorTemperature(), GRTUtil.getFPGATime()
        );
        targetReferenceLogEntry.append(targetReference, GRTUtil.getFPGATime());
    }
}
