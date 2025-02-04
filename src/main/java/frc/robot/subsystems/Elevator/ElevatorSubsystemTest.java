package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ElevatorSubsystemTest extends SubsystemBase {
    private final TalonFX motor;

    public ElevatorSubsystemTest() {
        motor = new TalonFX(13); // Assuming the motor controller is connected to PWM port 0
    }

    // Method to move the elevator in a given direction
    public void move(double speed) {
        motor.set(speed);
    }

    // Method to stop the elevator
    public void stop() {
        motor.set(0);
    }
}