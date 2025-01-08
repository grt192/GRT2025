package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleModuleSwerveSubsystem extends SubsystemBase{    

    SwerveModule module;
    double MAX_VEL = 1; //change this

    public SingleModuleSwerveSubsystem(SwerveModule module) {
        this.module = module;
    }

    public void setRawPowers(double drivePower, double steerPower) {
        module.setRawPowers(drivePower, steerPower);
    }

    public void setDrivePowers(double xPower, double yPower) {
        double velocity = MAX_VEL * Math.sqrt(yPower * yPower + xPower * xPower) / Math.sqrt(2);
        double angle = Math.atan2(yPower, xPower);

        module.setDesiredState(new SwerveModuleState(velocity, new Rotation2d(angle)));
    }

    public void setState(double speed, double angleRads) {
        module.setDesiredState(new SwerveModuleState(speed, new Rotation2d(angleRads)));
    }

}
