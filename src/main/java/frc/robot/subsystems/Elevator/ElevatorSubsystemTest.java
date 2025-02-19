package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystemTest extends SubsystemBase {
    private final TalonFX kmotor;
    private TalonFXConfiguration talonFXConfiguration;

    private final DigitalInput zeroLimitSwitch;

    public ElevatorSubsystemTest() {
        kmotor = new TalonFX(13, "can");

        talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 88;

        // Apply configs, apparently this fails a lot
        for (int i = 0; i < 4; i++) {
            boolean error = kmotor.getConfigurator().apply(talonFXConfiguration, 0.1) == StatusCode.OK;
            if (!error) break;
        }

        kmotor.setPosition(0);
        zeroLimitSwitch = new DigitalInput(0);

    }

    // Method to move the elevator in a given direction
    public void move(double speed) {
        kmotor.set(speed);
    }

    // Method to stop the elevator
    public void stop() {
        kmotor.set(0);
    }

    @Override
    public void periodic() {
        // System.out.println(kmotor.getPosition().getValueAsDouble());
        System.out.println(!zeroLimitSwitch.get());
        if (!zeroLimitSwitch.get()) {
            kmotor.setPosition(0);
        }
    }
}