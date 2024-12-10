package frc.robot.subsystems.swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveSetpoint;
import frc.robot.util.SwerveSetpointGenerator;

import static frc.robot.Constants.SwerveConstants.BL_DRIVE;
import static frc.robot.Constants.SwerveConstants.BL_OFFSET;
import static frc.robot.Constants.SwerveConstants.BL_POS;
import static frc.robot.Constants.SwerveConstants.BL_STEER;
import static frc.robot.Constants.SwerveConstants.BR_DRIVE;
import static frc.robot.Constants.SwerveConstants.BR_OFFSET;
import static frc.robot.Constants.SwerveConstants.BR_POS;
import static frc.robot.Constants.SwerveConstants.BR_STEER;
import static frc.robot.Constants.SwerveConstants.FL_DRIVE;
import static frc.robot.Constants.SwerveConstants.FL_OFFSET;
import static frc.robot.Constants.SwerveConstants.FL_POS;
import static frc.robot.Constants.SwerveConstants.FL_STEER;
import static frc.robot.Constants.SwerveConstants.FR_DRIVE;
import static frc.robot.Constants.SwerveConstants.FR_OFFSET;
import static frc.robot.Constants.SwerveConstants.FR_POS;
import static frc.robot.Constants.SwerveConstants.FR_STEER;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

//krakens drive, neo steer

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

    private final AHRS ahrs;

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private SwerveSetpoint currentSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(),
              new SwerveModuleState(),
              new SwerveModuleState(),
              new SwerveModuleState()
            });

    private final SwerveSetpointGenerator setpointGenerator;
    
    public SwerveSubsystem() {

        ahrs = new AHRS(SPI.Port.kMXP);

        frontLeftModule = new SwerveModule(FL_DRIVE, FL_STEER, FL_OFFSET);
        frontRightModule = new SwerveModule(FR_DRIVE, FR_STEER, FR_OFFSET);
        backLeftModule = new SwerveModule(BL_DRIVE, BL_STEER, BL_OFFSET);
        backRightModule = new SwerveModule(BR_DRIVE, BR_STEER, BR_OFFSET);

        setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS))
            .moduleLocations(new Translation2d[] {
                FL_POS, FR_POS, BL_POS, BR_POS
            })
            .build();

    }
    
}
