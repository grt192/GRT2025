package frc.robot.util.Motors;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class LoggedSparkMaxConfig {
    private final int canId;
    private final ClosedLoopConfig closedLoopConfig;
    private final SparkMaxConfig sparkMaxConfig;
    private final EncoderConfig encoderConfig;

    public LoggedSparkMaxConfig(
        int canId, ClosedLoopConfig closedLoopConfig, EncoderConfig encoderConfig){
        this.canId = canId;
        this.closedLoopConfig = closedLoopConfig;
        this.encoderConfig = encoderConfig;
        this.sparkMaxConfig = new SparkMaxConfig();
        this.sparkMaxConfig.apply(closedLoopConfig);
        this.sparkMaxConfig.apply(encoderConfig);
    }

    public int getCanId() {
        return canId;
    }

    public ClosedLoopConfig getClosedLoopConfig() {
        return closedLoopConfig;
    }

    public SparkMaxConfig getSparkMaxConfig() {
        return sparkMaxConfig;
    }

    public EncoderConfig getEncoderConfig() {
        return encoderConfig;
    }

}
