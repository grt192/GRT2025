package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LoggingConstants;

public class LoggedBooleanSensor {
    
    private final DigitalInput sensor;

    private NetworkTableInstance ntInstance;
    private NetworkTable sensorStatsTable;
    private BooleanPublisher sensorReadingPublisher;

    private BooleanLogEntry sensorReadingLogEntry;

    public LoggedBooleanSensor(String name, int port){
        sensor = new DigitalInput(port);
        initNT(name);
        initLogs(name);
    }

    /**
     * Get the reading of the sensor
     * @return
     */
    public boolean get(){
        return sensor.get();
    }

    /**
     * Publish sensor reading to NT
     */
    public void publishStats(){
        sensorReadingPublisher.set(sensor.get());
    }

    /**
     * Logs sensor reading
     */
    public void logStats(){
        sensorReadingLogEntry.append(sensor.get());
    }

    /**
     * Initialize networktables
     * @param name
     */
    private void initNT(String name){
        ntInstance = NetworkTableInstance.getDefault();
        sensorStatsTable = ntInstance.getTable(LoggingConstants.SENSOR_TABLE);
        sensorReadingPublisher = sensorStatsTable.getBooleanTopic(name).publish();
    }

    /**
     * Initialize logs
     * @param name
     */
    private void initLogs(String name){
        sensorReadingLogEntry = new BooleanLogEntry(
            DataLogManager.getLog(), name
        );
    }
}
