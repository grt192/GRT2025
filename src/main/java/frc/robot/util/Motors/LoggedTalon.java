package frc.robot.util.Motors;

import static frc.robot.Constants.LoggingConstants.CTRE_TABLE;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.util.GRTUtil;

public class LoggedTalon{

    private final TalonFX motor;

    private NetworkTableInstance ntInstance;
    private NetworkTable swerveStatsTable;
    private DoublePublisher veloErrorPublisher;
    private DoublePublisher veloPublisher;
    private DoublePublisher appliedVlotsPublisher;
    private DoublePublisher supplyCurrentPublisher;
    private DoublePublisher statorCurrentPublisher;

    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry veloErrorLogEntry;
    private DoubleLogEntry veloLogEntry;
    private DoubleLogEntry appliedVoltsLogEntry;
    private DoubleLogEntry supplyCurrLogEntry;
    private DoubleLogEntry statorCurrLogEntry;
    private DoubleLogEntry temperatureLogEntry;

    public LoggedTalon(int canId, TalonFXConfiguration talonConfig){
        motor = new TalonFX(canId, "can");
        for (int i = 0; i < 4; i++) {
            boolean error = motor.getConfigurator().apply(talonConfig, 0.1) == StatusCode.OK;
            if (!error) break;
        }
        initNT(canId);
        initLogs(canId);
    }

    /**
     * initializes network table and entries
     * @param canId motor's CAN ID
     */
    private void initNT(int canId){
        ntInstance = NetworkTableInstance.getDefault();
        swerveStatsTable = ntInstance.getTable(CTRE_TABLE);
        veloErrorPublisher = swerveStatsTable.getDoubleTopic(
            canId + "veloError"
        ).publish();
        veloPublisher = swerveStatsTable.getDoubleTopic(canId + "velo").publish();
        appliedVlotsPublisher = swerveStatsTable.getDoubleTopic(
            canId + "appliedVolts"
        ).publish();
        supplyCurrentPublisher = swerveStatsTable.getDoubleTopic(
            canId + "supplyCurrent"
        ).publish();
        statorCurrentPublisher = swerveStatsTable.getDoubleTopic(
            canId + "statorCurrent"
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
        veloErrorLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "veloError"
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
    }

    /**
     * Configures drive motor's PIDSV
     * @param p kP
     * @param i kI
     * @param d kD
     * @param s kS for static friction
     * @param v kV Voltage feed forward
     */
    public void configPID(double p, double i, double d, double s, double v) {
        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = s;
        slot0Configs.kV = v;

        motor.getConfigurator().apply(slot0Configs);
    }

    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }

    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Publishes motor stats to NT for logging
     */
    public void publishStats(){
        veloErrorPublisher.set(motor.getClosedLoopError().getValueAsDouble());
        veloPublisher.set(motor.getVelocity().getValueAsDouble());
        appliedVlotsPublisher.set(motor.getMotorVoltage().getValueAsDouble());
        supplyCurrentPublisher.set(motor.getSupplyCurrent().getValueAsDouble());
        statorCurrentPublisher.set(motor.getStatorCurrent().getValueAsDouble());
    }    

    public void logStats(){
        positionLogEntry.append(
            motor.getPosition().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        veloErrorLogEntry.append(
            motor.getClosedLoopError().getValueAsDouble(), GRTUtil.getFPGATime()
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
    }
    
    public void setPosition(double position){
        motor.setControl(new PositionVoltage(position));
    }
}
