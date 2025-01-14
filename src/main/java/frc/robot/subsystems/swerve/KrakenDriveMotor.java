package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveConstants.DRIVE_GEAR_REDUCTION;
import static frc.robot.Constants.SwerveConstants.DRIVE_WHEEL_CIRCUMFERENCE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class KrakenDriveMotor {
    
    private TalonFX motor;
    private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    private double targetRps = 0;

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private NetworkTableInstance ntInstance;
    private NetworkTable swerveStatsTable;
    private NetworkTableEntry deviceTempEntry;
    private NetworkTableEntry processorTempEntry;
    private NetworkTableEntry ampDrawEntry;
    private NetworkTableEntry statorCurrentEntry;
    private NetworkTableEntry supplyCurrentEntry;
    private NetworkTableEntry supplyVoltageEntry;

    private DoublePublisher veloErrorPublisher;

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Voltage> appliedVoltsSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Current> statorCurrentSignal; //torqueCurrent is Pro


    /** A kraken drive motor for swerve.
     *
     * @param canId The canId of the motor.
     */
    public KrakenDriveMotor(int canId) {
        motor = new TalonFX(canId, "can");

        // motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        // motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        // motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // motorConfig.Feedback.SensorToMechanismRatio = 3. * 20. / 26. * 3.; //according to Samuel

        // Apply configs, apparently this fails a lot
        for (int i = 0; i < 4; i++) {
            boolean error = motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK;
            if (!error) break;
        }

        initNT(canId);
        initSignals();
        veloErrorPublisher = swerveStatsTable.getDoubleTopic(canId + "veloError").publish();
    }
    
    /**
     * initializes network table and entries
     * @param canId motor's CAN ID
     */
    private void initNT(int canId){
        ntInstance = NetworkTableInstance.getDefault();
        swerveStatsTable = ntInstance.getTable("swerveStats");
        // deviceTempEntry = swerveStatsTable.getEntry(canId + "deviceTemp");
        // processorTempEntry = swerveStatsTable.getEntry(canId + "processorTemp");
        // ampDrawEntry = swerveStatsTable.getEntry(canId + "ampDraw");
        // statorCurrentEntry = swerveStatsTable.getEntry(canId + "statorCurrent");
        // supplyCurrentEntry = swerveStatsTable.getEntry(canId + "supplyCurrena");
        // supplyVoltageEntry = swerveStatsTable.getEntry(canId + "supplyVoltage");
    }

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
     * updates motor stats to network tables
     */
    public void updateNT(){
        // deviceTempEntry.setDouble(getDeviceTemp());
        // processorTempEntry.setDouble(getProcessorTemp());
        // ampDrawEntry.setDouble(getAmpDraw());
        // statorCurrentEntry.setDouble(getStatorCurrent());
        // supplyCurrentEntry.setDouble(getSupplyCurrent());
        // supplyVoltageEntry.seuble(getSupplyVoltage());
    }

    public void setVelocity(double metersPerSec) {
        targetRps = metersPerSec / DRIVE_WHEEL_CIRCUMFERENCE * DRIVE_GEAR_REDUCTION;
        motor.setControl(request.withVelocity(targetRps));
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void configPID(double p, double i, double d, double ff) {
        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kV = ff;
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;

        motor.getConfigurator().apply(slot0Configs);
    }

    public double getDistance() {
        return DRIVE_WHEEL_CIRCUMFERENCE / DRIVE_GEAR_REDUCTION * (motor.getPosition().getValueAsDouble());
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() / DRIVE_GEAR_REDUCTION * DRIVE_WHEEL_CIRCUMFERENCE;
    }

    public double getError() {
        return motor.getClosedLoopError().getValue();
    }

    public double getSetpoint() {
        return Units.radiansToRotations(targetRps); //not proportional to actual swerve rn
    }
    
    public double getAmpDraw() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getDeviceTemp() {
        return motor.getDeviceTemp().getValueAsDouble();
    }

    public double getProcessorTemp() {
        return motor.getProcessorTemp().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getSupplyVoltage() {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

    public void publishVeloError(){
        veloErrorPublisher.set(this.targetRps - motor.getVelocity().getValueAsDouble());
    }

}
