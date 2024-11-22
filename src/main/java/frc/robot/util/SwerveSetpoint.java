package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//"inspired" by Team 6328
public record SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {}