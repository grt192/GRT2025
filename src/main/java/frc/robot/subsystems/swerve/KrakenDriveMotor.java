package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class KrakenDriveMotor {
    
    private TalonFX motor;
    private double positionConversionFactor = 0;
    private double driveRotPerMinPerMetersPerSec = 0;
    private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    private double targetRps = 0;

    /** A kraken drive motor for swerve.
     *
     * @param canId The canId of the motor.
     */
    public KrakenDriveMotor(int canId) {
        motor = new TalonFX(canId);
    }

    public void setVelocity(double metersPerSec) {
        targetRps = metersPerSec * driveRotPerMinPerMetersPerSec / 60; 
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
        return motor.getPosition().getValue() / positionConversionFactor;
    }

    public double getVelocity() {
        return motor.getVelocity().getValue() / driveRotPerMinPerMetersPerSec; 
    }

    public void setVelocityConversionFactor(double factor) {
        driveRotPerMinPerMetersPerSec = factor;
    }

    public void setPositionConversionFactor(double factor) {
        positionConversionFactor = factor;
    }

    public double getError() {
        return motor.getClosedLoopError().getValue();
    }

    public double getSetpoint() {
        return targetRps;
    }
    
    public double getAmpDraw() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }
}
