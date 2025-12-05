package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoggingConstants;

public class PIDScoreGame{
    
    private NetworkTableInstance ntInstance;
    private NetworkTable sensorStatsTable;
    private DoublePublisher timeForActionPublisher;
    private DoublePublisher currentPosPublisher;
    private DoublePublisher desiredPosPublisher;
    private DoublePublisher veloPublisher;

    private double setDesiredTimeStamp;
    private double lastPosUpdateTimeStamp;
    private double lastPos;
    private double currentPos;
    private double desiredPos;
    private double velo;
    private double posError;
    private boolean won;
    private double timeForAction;
    private double veloMax;
    private double positionTolerance;
    /**
     * @param name name of your device it can be anything. It will show up as [name] PID game;
     * @param veloMax velocity bounds for the pid finish. Overshooting pid = bad
     * @param positionTolerance positionTolerance of PID
     */
    public PIDScoreGame(String name, Double veloMax, Double positionTolerance){
        initNT(name);
        this.veloMax = veloMax;
        this.positionTolerance = positionTolerance;
        // initLogs(name);
    }

    /**
     * @param set the arbitrary current position
     */

    public void setCurrentPosition(double currentPos){
        currentPosPublisher.set(currentPos);
        posError = currentPos - desiredPos;
        velo = (currentPos-lastPos)/(Timer.getFPGATimestamp() -lastPosUpdateTimeStamp);
        lastPosUpdateTimeStamp = Timer.getFPGATimestamp();
        lastPos = currentPos;

        veloPublisher.set(velo);

        if (won == false){
            if ((Math.abs(velo) < veloMax)&&(Math.abs(posError) < positionTolerance)){
                timeForAction = Timer.getFPGATimestamp() - setDesiredTimeStamp;
                timeForActionPublisher.set(timeForAction);
                won = true;
            }
        }

    }
    /**
     * @param set the desired / target position arbitrary units (make sure to conform with current position)
     */

    public void setDesiredPosition(double desiredPos){
        setDesiredTimeStamp = RobotController.getFPGATime();
        desiredPosPublisher.set(desiredPos);
        won = false;
    }


    /**
     * Publish sensor reading to NT
     */
    public void publishStats(){
        // sensorReadingPublisher.set(sensor.get());
    }

    // /**
    //  * Logs sensor reading
    //  */
    // public void logStats(){
    //     sensorReadingLogEntry.append(sensor.get());
    // }

    /**
     * Initialize networktables
     * @param name
     */
    private void initNT(String name){
        ntInstance = NetworkTableInstance.getDefault();
        sensorStatsTable = ntInstance.getTable(name + " PID game");
        timeForActionPublisher = sensorStatsTable.getDoubleTopic("Time For Action").publish();
        currentPosPublisher = sensorStatsTable.getDoubleTopic("Current Pos").publish();
        desiredPosPublisher = sensorStatsTable.getDoubleTopic("Target Pos").publish();
        veloPublisher = sensorStatsTable.getDoubleTopic("Velocity").publish();

    }

    /**
     * Initialize logs
     * @param name
     */
    private void initLogs(String name){
        // sensorReadingLogEntry = new BooleanLogEntry(
        //     DataLogManager.getLog(), name
        // );
    }
}
