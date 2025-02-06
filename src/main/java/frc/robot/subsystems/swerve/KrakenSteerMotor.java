package frc.robot.subsystems.swerve;

import static frc.robot.Constants.LoggingConstants.SWERVE_TABLE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage; 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.util.GRTUtil;

public class KrakenSteerMotor {

    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    ClosedLoopGeneralConfigs closedLoopGeneralConfigs;

    private VelocityVoltage request;
    private double targetDouble = 0;

    private NetworkTableInstance ntInstance;
    private NetworkTable swerveStatsTable;
    private DoublePublisher positionPublisher;
    private DoublePublisher targetPositionPublisher;
    private DoublePublisher veloErrorPublisher;
    private DoublePublisher veloPublisher;
    private DoublePublisher appliedVoltsPublisher;
    private DoublePublisher supplyCurrentPublisher;
    private DoublePublisher statorCurrentPublisher;

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Voltage> appliedVoltsSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Current> statorCurrentSignal;

    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry targetPositionLogEntry;
    private DoubleLogEntry busVoltageLogEntry;
    private DoubleLogEntry outputCurrentLogEntry;
    private DoubleLogEntry temperatureLogEntry;
    private DoubleLogEntry appliedOutputLogEntry;

    public KrakenSteerMotor(int canId) {
        motor = new TalonFX(canId, "can");

        // Configure motor settings
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //max motor voltage is 12V
        motorConfig.Voltage.PeakForwardVoltage = 12; 
        motorConfig.Voltage.PeakReverseVoltage = -12;

        // Enable position wrapping (by default values are from 0-1)
        closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        closedLoopGeneralConfigs.ContinuousWrap = true; //basicaly turns stacking off
        motorConfig.ClosedLoopGeneral = closedLoopGeneralConfigs;
        
        // Set initial position to 0
        motor.setPosition(0);
        for (int i = 0; i < 4; i++) {
            boolean error = motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK;
            if (!error) break;
        }

        initNT(canId);
        initSignals();
        initLogs(canId);
    }

    /**
     * Initializes NetworkTables for publishing motor statistics.
     * @param canId The CAN ID of the motor.
     */
    private void initNT(int canId) {
        ntInstance = NetworkTableInstance.getDefault();
        swerveStatsTable = ntInstance.getTable(SWERVE_TABLE);
        positionPublisher = swerveStatsTable.getDoubleTopic(canId + "position").publish();
        targetPositionPublisher = swerveStatsTable.getDoubleTopic(canId + "targetPosition").publish();
        veloErrorPublisher = swerveStatsTable.getDoubleTopic(canId + "veloError").publish();
        veloPublisher = swerveStatsTable.getDoubleTopic(canId + "velo").publish();
        appliedVoltsPublisher = swerveStatsTable.getDoubleTopic(canId + "appliedVolts").publish();
        supplyCurrentPublisher = swerveStatsTable.getDoubleTopic(canId + "supplyCurrent").publish();
        statorCurrentPublisher = swerveStatsTable.getDoubleTopic(canId + "statorCurrent").publish();
    }

    /**
     * Initializes the status signals for the motor.
     */
    private void initSignals() {
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        appliedVoltsSignal = motor.getMotorVoltage();
        statorCurrentSignal = motor.getStatorCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();

        // Set update frequency for all signals
        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0, positionSignal, velocitySignal,
            appliedVoltsSignal, statorCurrentSignal, supplyCurrentSignal
        );
        motor.optimizeBusUtilization(0, 1.0);
    }

    /**
     * Initializes the data logs for the motor.
     * @param canId The CAN ID of the motor.
     */
    private void initLogs(int canId) {
        positionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "position");
        targetPositionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "targetPosition");
        busVoltageLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "busVoltage");
        outputCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "outputCurrent");
        temperatureLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "temperature");
        appliedOutputLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "appliedOutput");
    }

    /**
     * Configures the PID constants for the motor.
     * @param p Proportional gain.
     * @param i Integral gain.
     * @param d Derivative gain.
     * @param s Static gain.
     * @param v Velocity gain.
     */
    public void configurePID(double p, double i, double d, double ff) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = ff;
        motor.getConfigurator().apply(slot0Configs);
    }

    /**
     * Sets the target position for the motor.
     * @param targetRads The target position in radians.
     */
    public void setPosition(double targetRads) {
        targetDouble = targetRads / (2 * Math.PI); //turn into a value between 0 and 1
        motor.setControl(request);
    }

    /**
     * Gets the current position of the motor.
     * @return The current position in radians.
     */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * Publishes the motor statistics to NetworkTables.
     */
    public void publishStats() {
        positionPublisher.set(getPosition());
        targetPositionPublisher.set(targetDouble);
    }

}
