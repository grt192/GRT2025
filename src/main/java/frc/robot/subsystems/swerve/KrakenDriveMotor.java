package frc.robot.subsystems.swerve;

import static frc.robot.Constants.LoggingConstants.SWERVE_TABLE;
import static frc.robot.Constants.SwerveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

public class KrakenDriveMotor {
    
    private TalonFX motor;
    // private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0).withSlot(0);
     private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    
    private double targetRps = 0;

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    //logging
    //logging
    private NetworkTableInstance ntInstance;
    private NetworkTable swerveStatsTable;
    private DoublePublisher veloErrorPublisher;
    private DoublePublisher veloPublisher;
    private DoublePublisher appliedVlotsPublisher;
    private DoublePublisher supplyCurrentPublisher;
    private DoublePublisher statorCurrentPublisher;
    private DoublePublisher targetRPSPublisher;
    private DoublePublisher positionPublisher;

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Voltage> appliedVoltsSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Current> statorCurrentSignal; //torqueCurrent is Pro

    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry veloErrorLogEntry;
    private DoubleLogEntry veloLogEntry;
    private DoubleLogEntry targetVeloEntry;
    private DoubleLogEntry appliedVoltsLogEntry;
    private DoubleLogEntry supplyCurrLogEntry;
    private DoubleLogEntry statorCurrLogEntry;
    private DoubleLogEntry temperatureLogEntry;

    /** A kraken drive motor for swerve.
     *
     * @param canId The canId of the motor.
     */
    public KrakenDriveMotor(int canId) {
        motor = new TalonFX(canId, "can");

        // motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = PEAK_CURRENT;
        // motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = - PEAK_CURRENT;

        // motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = RAMP_RATE;
        // motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = 12;
        

        motor.setPosition(0);
        // Apply configs, apparently this fails a lot
        for (int i = 0; i < 4; i++) {
            boolean error = motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK;
            if (!error) break;
        }

        initNT(canId);
        initSignals();
        initLogs(canId);
    }
    
    /**
     * initializes network table and entries
     * @param canId motor's CAN ID
     */
    private void initNT(int canId){
        ntInstance = NetworkTableInstance.getDefault();
        swerveStatsTable = ntInstance.getTable(SWERVE_TABLE);
        positionPublisher = swerveStatsTable.getDoubleTopic(canId + "position").publish();
        targetRPSPublisher = swerveStatsTable.getDoubleTopic(canId + "targetRPS").publish();
        veloErrorPublisher = swerveStatsTable.getDoubleTopic(canId + "veloError").publish();
        veloPublisher = swerveStatsTable.getDoubleTopic(canId + "velo").publish();
        appliedVlotsPublisher = swerveStatsTable.getDoubleTopic(canId + "appliedVolts").publish();
        supplyCurrentPublisher = swerveStatsTable.getDoubleTopic(canId + "supplyCurrent").publish();
        statorCurrentPublisher = swerveStatsTable.getDoubleTopic(canId + "statorCurrent").publish();
    }

    /**
     * Initializes Phoenix 6's signals
     */
    private void initSignals(){
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        appliedVoltsSignal = motor.getMotorVoltage();
        statorCurrentSignal = motor.getStatorCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0, positionSignal, velocitySignal,
            appliedVoltsSignal, statorCurrentSignal, supplyCurrentSignal
        );
        motor.optimizeBusUtilization(0, 1.0);
    }

    /**
     * Initializes log entries
     * @param canId drive motor's CAN ID
     */
    private void initLogs(int canId){
        positionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "position");
        veloErrorLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "veloError"); 
        veloLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "velo");
        targetVeloEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "targetVelo");
        appliedVoltsLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "appliedVolts");
        supplyCurrLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "supplyCurrent");
        statorCurrLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "statorCurrent");
        temperatureLogEntry = new DoubleLogEntry(DataLogManager.getLog(), canId + "temperature");
    }
    /**
     * Set motor velo to target velo
     * @param metersPerSec target velo in m/s
     */
    public void setVelocity(double metersPerSec) {
        targetRps = metersPerSec / DRIVE_WHEEL_CIRCUMFERENCE * DRIVE_GEAR_REDUCTION;
        motor.setControl(request.withVelocity(targetRps));
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

        //dividing by KT to convert volts to current
        // slot0Configs.kP = p / KT;
        // slot0Configs.kI = i / KT;
        // slot0Configs.kD = d / KT;
        // slot0Configs.kS = s / KT;
        // slot0Configs.kV = v / KT;

        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
        slot0Configs.kS = s;
        slot0Configs.kV = v;

        motor.getConfigurator().apply(slot0Configs);
    }

    /**
     * Gets the distance the drive wheel has traveled.
     * @return distance the drive wheel has traveled in meters
     */
    public double getDistance() {
        return DRIVE_WHEEL_CIRCUMFERENCE / DRIVE_GEAR_REDUCTION * (motor.getPosition().getValueAsDouble());
    }

    /**
     * Get swerve wheel's velocity in m/s
     * @return swerve wheel's velocity in m/s
     */
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() / DRIVE_GEAR_REDUCTION * DRIVE_WHEEL_CIRCUMFERENCE;
    }

    /**
     * Gets the error of the closed loop controller
     * @return
     */
    public double getError() {
        return motor.getClosedLoopError().getValueAsDouble();
    }

    /**
     * Gets the tempature of the motor
     * @return temperature of the motor in double
     */
    public double getTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }



    /**
     * Publishes motor stats to NT for logging
     */
    public void publishStats() {
        // veloErrorPublisher.set(this.targetRps - motor.getVelocity().getValueAsDouble());
        positionPublisher.set(getDistance());
        targetRPSPublisher.set(targetRps);
        veloErrorPublisher.set(motor.getClosedLoopError().getValueAsDouble());
        veloPublisher.set(motor.getVelocity().getValueAsDouble());
        appliedVlotsPublisher.set(motor.getMotorVoltage().getValueAsDouble());
        supplyCurrentPublisher.set(motor.getSupplyCurrent().getValueAsDouble());
        statorCurrentPublisher.set(motor.getStatorCurrent().getValueAsDouble());
    }

    public void logStats() {
        positionLogEntry.append(
            motor.getPosition().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        veloErrorLogEntry.append(
            motor.getClosedLoopError().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        veloLogEntry.append(
            motor.getVelocity().getValueAsDouble(), GRTUtil.getFPGATime()
        );

        targetVeloEntry.append(targetRps, GRTUtil.getFPGATime());

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
}