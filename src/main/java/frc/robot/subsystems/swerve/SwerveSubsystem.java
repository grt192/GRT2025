package frc.robot.subsystems.swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

//krakens drive, neo steer

public class SwerveSubsystem extends SubsystemBase {
    
      //motors
    private final CANSparkMax krakenRight;
    private final WPI_TalonSRX rightFollower; 
    private final WPI_TalonSRX leftMotor; 
    private final WPI_TalonSRX leftFollower; 

    //constructor 
    public SwerveSubsystem() {
        krakenRight = new CANSparkMax(1, MotorType.kBrushless);
}
    }
}
