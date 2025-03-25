package frc.robot.Commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveBackwardsCommand extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private final ChassisSpeeds speeds;

    public DriveBackwardsCommand(SwerveSubsystem swerveSubsystem, ChassisSpeeds speeds) {
        this.swerveSubsystem = swerveSubsystem;
        this.speeds = speeds;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.setDrivePowers(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
    }

}