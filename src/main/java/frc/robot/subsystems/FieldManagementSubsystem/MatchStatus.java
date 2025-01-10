package frc.robot.subsystems.FieldManagementSubsystem;

/**
 * Holds current state of match.
 */
public enum MatchStatus {
    NOTSTARTED,
    AUTON, 
    TELEOP,
    ENDGAME,
    ENDED;
}