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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class LoggedSparkMax {
    
    private final SparkMax motor;
    private final SparkMaxConfig sparkMaxConfig;
    private final ClosedLoopConfig closedLoopConfig;
    private final RelativeEncoder encoder;
    private SparkClosedLoopController closedLoopController;
    private double targetPosition;

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

    public void configurePIDF(double p, double i, double d, double f) {
        closedLoopConfig.pidf(p, i, d, f);
        sparkMaxConfig.apply(closedLoopConfig);
        motor.configure(sparkMaxConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
        );
        closedLoopController = motor.getClosedLoopController();
    }

    public void setPosition(double position){
        targetPosition = position;
        closedLoopController.setReference(position, ControlType.kPosition);
    }

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
        neoSetPositionPublisher.set(targetPosition);
    }

    /**
     * Logs Neo stats into log file
     */
    public void logStats(){
        positionLogEntry.append(getPosition(), GRTUtil.getFPGATime());
        targetPositionLogEntry.append(targetPosition, GRTUtil.getFPGATime());
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
