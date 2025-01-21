package frc.robot.subsystems.PhoenixLoggingSubsystem;

import javax.xml.crypto.Data;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.FieldManagementSubsystem.MatchStatus;
import frc.robot.subsystems.FieldManagementSubsystem.RobotStatus;

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
            DataLogManager.start();
            // DriverStation.startDataLog(DataLogManager.getLog());
            isLogging = true;
        }
        else if(currentMatchStatus == MatchStatus.ENDED){
            SignalLogger.stop();
            DataLogManager.stop();
            isLogging = false;
        }
        else if(fieldManagementSubsystem.getRobotStatus() == RobotStatus.DISABLED){
            DataLogManager.stop();
            SignalLogger.stop();
            isLogging = false;
        }
        // System.out.println("Log Dir: " + DataLogManager.getLogDir());
    }
}
