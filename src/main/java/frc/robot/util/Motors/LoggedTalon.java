package frc.robot.util.Motors;

import static frc.robot.Constants.LoggingConstants.CTRE_TABLE;
import static frc.robot.Constants.DebugConstants.CTRE_DEBUG;

import java.util.EnumSet;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.util.GRTUtil;

public class LoggedTalon{

    private final TalonFX motor;

    private final int canId;
    private double[] pidsvg = new double[6];

    private NetworkTableInstance ntInstance;
    private NetworkTable motorStatsTable;

    private DoublePublisher positionPublisher;
    private DoublePublisher veloPublisher;
    private DoublePublisher appliedVlotsPublisher;
    private DoublePublisher supplyCurrentPublisher;
    private DoublePublisher statorCurrentPublisher;
    private DoublePublisher temperaturePublisher;
    private DoublePublisher targetPositionPublisher;
    private DoublePublisher targetVelocityPublisher;

    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry veloLogEntry;
    private DoubleLogEntry appliedVoltsLogEntry;
    private DoubleLogEntry supplyCurrLogEntry;
    private DoubleLogEntry statorCurrLogEntry;
    private DoubleLogEntry temperatureLogEntry;
    private DoubleLogEntry targetPositionLogEntry; 
    private DoubleLogEntry targetVelocityLogEntry;

    private double targetPosition;
    private double targetVelocity;

    public LoggedTalon(int canId, TalonFXConfiguration talonConfig){
        motor = new TalonFX(canId);
        for (int i = 0; i < 4; i++) {
            boolean error = motor.getConfigurator().apply(talonConfig, 0.1) == StatusCode.OK;
            if (!error) break;
        }
        this.canId = canId;
        pidsvg = new double[] {
            talonConfig.Slot0.kP,
            talonConfig.Slot0.kI,
            talonConfig.Slot0.kD,
            talonConfig.Slot0.kS,
            talonConfig.Slot0.kV,
            talonConfig.Slot0.kG
        };
        initNT(canId);
        initLogs(canId);
        if(CTRE_DEBUG){
            enableDebug();
        }
    }

    /**
     * Using PIDF to set the motor's position
     * @param position position reference
     */
    public void setPosition(double position){
        targetPosition = position;
        motor.setControl(new PositionVoltage(position));
    }

    /**
     * Using PIDF to set the motor's velocity
     * @param velocity velocity reference
     */
    public void setVelocity(double velocity){
        targetVelocity = velocity;
        motor.setControl(new VelocityVoltage(velocity));
    }

    /**
     * Configures drive motor's PIDSV
     * @param p kP
     * @param i kI
     * @param d kD
     * @param s kS for static friction
     * @param v kV Voltage feed forward
     */
    public void configurePIDSVG(double p, double i, double d, double s, double v,
        double g
    ) {
        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = s;
        slot0Configs.kV = v;
        slot0Configs.kG = g;

        motor.getConfigurator().apply(slot0Configs);
    }

    /**
     * initializes network table and entries
     * @param canId motor's CAN ID
     */
    private void initNT(int canId){
        ntInstance = NetworkTableInstance.getDefault();
        motorStatsTable = ntInstance.getTable(CTRE_TABLE);
        positionPublisher = motorStatsTable.getDoubleTopic(
            canId + "position"
        ).publish();
        veloPublisher = motorStatsTable.getDoubleTopic(canId + "velo").publish();
        appliedVlotsPublisher = motorStatsTable.getDoubleTopic(
            canId + "appliedVolts"
        ).publish();
        supplyCurrentPublisher = motorStatsTable.getDoubleTopic(
            canId + "supplyCurrent"
        ).publish();
        statorCurrentPublisher = motorStatsTable.getDoubleTopic(
            canId + "statorCurrent"
        ).publish();
        temperaturePublisher = motorStatsTable.getDoubleTopic(
            canId + "temperature"
        ).publish();
        targetPositionPublisher = motorStatsTable.getDoubleTopic(
            canId + "targetPosition"
        ).publish();
        targetVelocityPublisher = motorStatsTable.getDoubleTopic(
            canId + "targetVelocity"
        ).publish();
    }

    /**
     * Initializes log entries
     * @param canId drive motor's CAN ID
     */
    private void initLogs(int canId){
        positionLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "position"
        );
        veloLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "velo"
        );
        appliedVoltsLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "appliedVolts"
        );
        supplyCurrLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "supplyCurrent"
        );
        statorCurrLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "statorCurrent"
        );
        temperatureLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "temperature"
        );
        targetPositionLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "targetPosition"
        );
        targetVelocityLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "targetVelocity"
        );
    }

    /**
     * Allows changing PID through NT
     */
    private void enableDebug(){
        motorStatsTable.getDoubleArrayTopic(canId + "PIDSVG").publish().set(
            pidsvg
        );
        motorStatsTable.addListener(canId + "PIDSVG", 
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table, key, event) -> {
                configurePIDSVG(
                    event.valueData.value.getDoubleArray()[0],
                    event.valueData.value.getDoubleArray()[1],
                    event.valueData.value.getDoubleArray()[2],
                    event.valueData.value.getDoubleArray()[3],
                    event.valueData.value.getDoubleArray()[4],
                    event.valueData.value.getDoubleArray()[5]
                );
            }
        );
    }
    /**
     * Gets motor's position in rotations after taking the 
     * SensorToMechanismRatio into account 
     * @return position of the motor in rotations
     */
    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Gets motor's velocity in RPS after taking the SensorToMechanismRatio into
     * account
     * @return velocity of the motor in RPS
     */
    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Publishes motor stats to NT for logging
     */
    public void publishStats(){
        positionPublisher.set(motor.getPosition().getValueAsDouble());
        veloPublisher.set(motor.getVelocity().getValueAsDouble());
        appliedVlotsPublisher.set(motor.getMotorVoltage().getValueAsDouble());
        supplyCurrentPublisher.set(motor.getSupplyCurrent().getValueAsDouble());
        statorCurrentPublisher.set(motor.getStatorCurrent().getValueAsDouble());
        temperaturePublisher.set(motor.getDeviceTemp().getValueAsDouble());
        targetPositionPublisher.set(targetPosition);
        targetVelocityPublisher.set(targetVelocity);
    }    

    /**
     * Loggs motor stats into log file
     */
    public void logStats(){
        positionLogEntry.append(
            motor.getPosition().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        veloLogEntry.append(
            motor.getVelocity().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        appliedVoltsLogEntry.append(
            motor.getMotorVoltage().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        supplyCurrLogEntry.append(
            motor.getSupplyCurrent().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        statorCurrLogEntry.append(
            motor.getStatorCurrent().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        temperatureLogEntry.append(
            motor.getDeviceTemp().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        targetPositionLogEntry.append(targetPosition, GRTUtil.getFPGATime());
        targetVelocityLogEntry.append(targetVelocity, GRTUtil.getFPGATime());
    }
}
