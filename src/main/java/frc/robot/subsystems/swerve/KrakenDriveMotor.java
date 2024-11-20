package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class KrakenDriveMotor {
    
    private TalonFX motor;
    private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    private double targetRps = 0;

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    /** A kraken drive motor for swerve.
     *
     * @param canId The canId of the motor.
     */
    public KrakenDriveMotor(int canId) {
        motor = new TalonFX(canId);

        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        motorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Feedback.SensorToMechanismRatio = 6.923; //according to Samuel

        // Apply configs, apparently this fails a lot
        for (int i = 0; i < 4; i++) {
            boolean error = motor.getConfigurator().apply(motorConfig, 0.1) == StatusCode.OK;
            if (!error) break;
    }

    }

    public void setVelocity(double metersPerSec) {
        targetRps = Units.radiansToRotations(metersPerSec);
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
        return motor.getPosition().getValue();
    }

    public double getVelocity() {
        return motor.getVelocity().getValue(); 
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
}
