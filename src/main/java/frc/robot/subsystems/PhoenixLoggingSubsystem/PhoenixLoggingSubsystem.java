package frc.robot.subsystems.PhoenixLoggingSubsystem;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.FieldManagementSubsystem.MatchStatus;

public class PhoenixLoggingSubsystem extends SubsystemBase{

    private FieldManagementSubsystem fieldManagementSubsystem;
    private boolean isLogging = false;

    public PhoenixLoggingSubsystem(FieldManagementSubsystem fieldManagementSubsystem){
        this.fieldManagementSubsystem = fieldManagementSubsystem;
        SignalLogger.setPath("/media/sda1/ctre-logs/");
    }

    @Override
    public void periodic(){
        MatchStatus currentMatchStatus = fieldManagementSubsystem.getMatchStatus();
        if(currentMatchStatus == MatchStatus.AUTON
            || currentMatchStatus == MatchStatus.TELEOP
            && !isLogging){
            SignalLogger.start();
        }
        else if(currentMatchStatus == MatchStatus.ENDED){
            SignalLogger.stop();
        }
    }
}
