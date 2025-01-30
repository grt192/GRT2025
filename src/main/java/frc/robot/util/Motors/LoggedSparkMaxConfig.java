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

    /**
     * Follow another motor
     * @param canId the CAN ID of the motor to follow
     */
    public void follow(int canId){
        this.sparkMaxConfig.follow(canId);
    }

    /**
     * Follow another motor
     * @param canId the CAN ID of the motor to follow
     * @param invert whether to invert the motor
     */
    public void follow(int canId, boolean invert){
        this.sparkMaxConfig.follow(canId, invert);
    }

    /**
     * Get the CAN ID of the motor
     * @return the CAN ID of the motor
     */
    public int getCanId() {
        return canId;
    }

    /**
     * Get the closed loop configuration
     * @return the closed loop configuration
     */
    public ClosedLoopConfig getClosedLoopConfig() {
        return closedLoopConfig;
    }

    /**
     * Get the Spark Max configuration
     * @return the Spark Max configuration
     */
    public SparkMaxConfig getSparkMaxConfig() {
        return sparkMaxConfig;
    }

    /**
     * Get the encoder configuration
     * @return the encoder configuration
     */
    public EncoderConfig getEncoderConfig() {
        return encoderConfig;
    }
}
