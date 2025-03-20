package frc.robot.util;

import java.util.EnumSet;

import javax.xml.crypto.Data;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import static frc.robot.Constants.DebugConstants.MASTER_DEBUG;
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
    private DoublePublisher targetTorqueCurrentFOCPublisher;
    private DoublePublisher targetDutyCyclePublisher; 
    private DoublePublisher closedLoopErrorPublisher;

    private DoubleLogEntry positionLogEntry;
    private DoubleLogEntry veloLogEntry;
    private DoubleLogEntry appliedVoltsLogEntry;
    private DoubleLogEntry supplyCurrLogEntry;
    private DoubleLogEntry statorCurrLogEntry;
    private DoubleLogEntry temperatureLogEntry;
    private DoubleLogEntry targetPositionLogEntry; 
    private DoubleLogEntry targetVelocityLogEntry;
    private DoubleLogEntry targetTorqueCurrentFOCLogEntry;
    private DoubleLogEntry targetDutyCycleLogEntry; 
    private DoubleLogEntry closedLoopErrorLogEntry;

    private double targetPosition;
    private double targetVelocity;
    private double targetTorqueCurrentFOC;
    private double targetDutyCycle;

    public LoggedTalon(
        int canId, String canBusName, TalonFXConfiguration talonConfig
    ){
        motor = new TalonFX(canId, canBusName);
        for (int i = 0; i < 4; i++) {
            boolean error =
                motor.getConfigurator()
                .apply(talonConfig, 0.1)
                == StatusCode.OK;
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
        if(MASTER_DEBUG){
            enableDebug();
        }
    }

    /**
     * Sets the motor's position reference
     * @param position position reference
     */
    public void setPositionReference(double position){
        targetPosition = position;
        motor.setControl(new PositionTorqueCurrentFOC(position));
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Sets the motor's position reference with voltage
     * @param position position reference
     */
    public void setPositionReferenceWithVoltage(double position, double arbFF){
        targetPosition = position;
        motor.setControl(new PositionVoltage(position).withFeedForward(arbFF));
    }

    /**
     * Sets the motor's position reference with feedforward
     * @param position position reference
     * @param arbFF arbitrary feedforward
     */
    public void setPositionReferenceWithArbFF(double position, double arbFF){
        targetPosition = position;
        motor.setControl(
            new PositionTorqueCurrentFOC(position).withSlot(0).withFeedForward(arbFF)
        );
    }

    /**
     * Set the motor's velocity reference
     * @param velocity velocity reference
     */
    public void setVelocityReference(double velocity){
        targetVelocity = velocity;
        motor.setControl(new VelocityTorqueCurrentFOC(velocity));
    }
    /**
     * Sets the motor's velocity reference with voltage
     * @param velocity velocity reference
     */
    public void setVelocityReferenceWithVoltage(double velocity){
        targetVelocity = velocity;
        motor.setControl(new VelocityVoltage(velocity));
    }

    /**
     * Set the current of the motor directly
     * @param current target current output
     */
    public void setTorqueCurrentFOC(double current){
        targetTorqueCurrentFOC = current;
        motor.setControl(new TorqueCurrentFOC(current));
    } 

    /**
     * Set duty cycle output
     * @param output duty cycle output from -1.0 to 1.0
     */
    public void setDutyCycle(double output){
        targetDutyCycle = output;
        motor.setControl(new DutyCycleOut(output));
    }

    /**
     * Sets the motor's position to a certain value
     * @param position mechanisam position after taking the position conversion
     * into account
     */
    public void setPosition(double position){
        for (int i = 0; i < 4; i++) {
            boolean error = motor.setPosition(position) == StatusCode.OK;
            if (!error) break;
        }
    }

    public void setPower(double power) {
        motor.set(power);
    }

    /**
     * Resets the encoder position to zero
     */
    public void resetEncoder() {
        motor.setPosition(0.0);
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
        motorStatsTable = ntInstance.getTable("MotorStats");

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

        targetTorqueCurrentFOCPublisher = motorStatsTable.getDoubleTopic(
            canId + "targetTorqueCurrentFOC"
        ).publish();

        targetDutyCyclePublisher = motorStatsTable.getDoubleTopic(
            canId + "targetDutyCycle"
        ).publish();

        closedLoopErrorPublisher = motorStatsTable.getDoubleTopic(
            canId + "closedLoopError"
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

        targetTorqueCurrentFOCLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "targetTorqueCurrentFOC"
        );

        targetDutyCycleLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "targetDutyCycle"
        );
        
        closedLoopErrorLogEntry = new DoubleLogEntry(
            DataLogManager.getLog(), canId + "closedLoopError"
        );
    }

    /**
     * Allows changing PID through NT
     */
    public void enableDebug(){
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
     * Gets motor's closed loop error
     * @return closed loop error
     */
    public double getClosedLoopError(){
        return motor.getClosedLoopError().getValueAsDouble();
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
        targetDutyCyclePublisher.set(targetDutyCycle);
        targetTorqueCurrentFOCPublisher.set(targetTorqueCurrentFOC);
        closedLoopErrorPublisher.set(motor.getClosedLoopError().getValueAsDouble());
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

        targetTorqueCurrentFOCLogEntry.append(
            targetTorqueCurrentFOC, GRTUtil.getFPGATime()
        );

        targetDutyCycleLogEntry.append(targetDutyCycle, GRTUtil.getFPGATime());

        closedLoopErrorLogEntry.append(
            motor.getClosedLoopError().getValueAsDouble(), GRTUtil.getFPGATime()
        );
    }
}