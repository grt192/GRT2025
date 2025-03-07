package frc.robot.subsystems.FieldManagementSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

/** The subsystem that manages everything field related. */
public class FieldManagementSubsystem extends SubsystemBase {

    private boolean isRed;
    private boolean connectedToFMS;
    private MatchStatus matchStatus;
    private boolean matchStarted = false;
    private RobotStatus robotStatus;

    private NetworkTableInstance FMSNTInstance;
    private NetworkTable FMSNTTable;
    private NetworkTableEntry stationNumberEntry;
    private NetworkTableEntry matchNumberEntry;
    private NetworkTableEntry matchTypeEntry;
    private NetworkTableEntry timeLeftEntry;
    private NetworkTableEntry isAutonomousEntry;
    private NetworkTableEntry isEStoppedEntry;
    private NetworkTableEntry isEnabledEntry;
    private NetworkTableEntry isDSAttachedEntry;
    /**
     * Initializes subsystem to handle information related to the Field Management System (such as our alliance color).
     */
    public FieldManagementSubsystem() {

        isRed = false;
        connectedToFMS = false;
        matchStatus = MatchStatus.NOTSTARTED;

        FMSNTInstance = NetworkTableInstance.getDefault();
        FMSNTTable = FMSNTInstance.getTable("FMS");
        stationNumberEntry = FMSNTTable.getEntry("StationNumber");
        matchNumberEntry = FMSNTTable.getEntry("MatchNumber");
        matchTypeEntry = FMSNTTable.getEntry("MatchType");
        timeLeftEntry = FMSNTTable.getEntry("TimeLeft");
        isAutonomousEntry = FMSNTTable.getEntry("IsAutonomous");
        isEStoppedEntry = FMSNTTable.getEntry("IsEStopped");
        isEnabledEntry = FMSNTTable.getEntry("IsEnabled");
        isDSAttachedEntry = FMSNTTable.getEntry("IsDSAttached");
    }
  
    public void periodic() {
        boolean incomingIsRed;
        try {
            incomingIsRed = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
        } catch (Exception e) {
            incomingIsRed = isRed;
        }
        
        if (incomingIsRed != isRed) {
            if (incomingIsRed) {
                System.out.println("Alliance color switched to Red.");
            } else {
                System.out.println("Alliance color switched to Blue.");
            }
        }
        isRed = incomingIsRed;

        boolean incomingFMSstatus = DriverStation.isFMSAttached();
        if (incomingFMSstatus != connectedToFMS) {
            if (incomingFMSstatus) {
                System.out.println("Connected to FMS.");
            } else {
                System.out.println("Disconnected from FMS.");
            }
        }
        connectedToFMS = incomingFMSstatus;

        if (DriverStation.isAutonomous()) { 
            matchStatus = MatchStatus.AUTON; 
            matchStarted = true;
        }
        else if (DriverStation.isTeleop()) { 
            matchStatus = MatchStatus.TELEOP; 
        }
        else if (DriverStation.isTeleopEnabled() && (DriverStation.getMatchTime() < 30)) { 
            matchStatus = MatchStatus.ENDGAME; // without an FMS, we will be in 'endgame' for the first 30 sec.
        }
        else if(DriverStation.getMatchTime() == 0 && matchStarted){
            matchStatus = MatchStatus.ENDED;
        }

        if(DriverStation.isEnabled()){
            robotStatus = RobotStatus.ENABLED;
        }
        else if(DriverStation.isEStopped()){
            robotStatus = RobotStatus.ESTOPPED;
        }
        else if(DriverStation.isDisabled()){
            robotStatus = RobotStatus.DISABLED;
        }
        
        stationNumberEntry.setInteger(DriverStation.getLocation().orElse(-1));
        matchNumberEntry.setInteger(DriverStation.getMatchNumber());
        matchTypeEntry.setString(DriverStation.getMatchType().toString());
        timeLeftEntry.setDouble(DriverStation.getMatchTime());
        isAutonomousEntry.setBoolean(DriverStation.isAutonomous());
        isEStoppedEntry.setBoolean(DriverStation.isEStopped());
        isEnabledEntry.setBoolean(DriverStation.isEnabled());
        isDSAttachedEntry.setBoolean(DriverStation.isDSAttached());
    }

    /**
     * Identifies whether or not we are Red Alliance.

     * @return isRed boolean
     */
    public boolean isRedAlliance() {

        return isRed;
    }
    
    /**
     * Identifies whether or not we are connected to an FRC Field Management System.

     * @return connectedToFMS boolean
     */
    public boolean isConnectedToFMS() {

        return connectedToFMS;
    }

    /**
     * Returns current match status (AUTON, TELEOP, ENDGAME).

     * @return current MatchStatus
     */
    public MatchStatus getMatchStatus() {

        return matchStatus;
    }

    /**
     * Returns current robot status (ENABLED, DISABLED, ESTOPPED).
     * @return current robot status
     */
    public RobotStatus getRobotStatus(){

        return robotStatus;
    }
}