package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleModuleSwerveSubsystem extends SubsystemBase{    

    SwerveModule module;
    Rotation2d rotationVal = new Rotation2d(Math.PI);
    SwerveModuleState state_1 = new SwerveModuleState(0, rotationVal);
    SwerveModuleState state_2 = new SwerveModuleState(0, rotationVal);
    SwerveModuleState state_3 = new SwerveModuleState(0, rotationVal);
    SwerveModuleState state_4 = new SwerveModuleState(0, rotationVal);
    double MAX_VEL = 1; //change this

    public static final double STEER_POWER = .4;
    public static final double DRIVE_POWER = 1;

    private double drive;
    private double steer;

    public SingleModuleSwerveSubsystem(SwerveModule module) {
        this.module = module;

        this.steer = 0;
        this.drive = 0;
    }

    public void setRawPowers(double drivePower, double steerPower) {
        module.setRawPowers(drivePower, steerPower);
    }

    public void setDrivePowers(double xPower, double yPower) {
        double velocity = MAX_VEL * Math.sqrt(yPower * yPower + xPower * xPower) / Math.sqrt(2);
        double angle = Math.atan2(yPower, xPower);

        module.setUnoptimizedDesiredState(new SwerveModuleState(velocity, new Rotation2d(angle)));
    }

    public void setState(double speed, double angleRads) {
        module.setUnoptimizedDesiredState(new SwerveModuleState(speed, new Rotation2d(angleRads)));
    }

}
