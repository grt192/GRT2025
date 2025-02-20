package frc.robot.util.Motors;

import java.util.Optional;
import java.util.OptionalInt;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class LoggedSparkMaxConfig {
    private final int canId;
    private final ClosedLoopConfig closedLoopConfig;
    private final SparkMaxConfig sparkMaxConfig;
    private final EncoderConfig encoderConfig;

    public LoggedSparkMaxConfig(
        int canId, ClosedLoopConfig closedLoopConfig,
        EncoderConfig encoderConfig, OptionalInt followCanId,
        Optional<SoftLimitConfig> softLimitConfig
    ){
        this.canId = canId;
        this.closedLoopConfig = closedLoopConfig;
        this.encoderConfig = encoderConfig;
        this.sparkMaxConfig = new SparkMaxConfig();
        this.sparkMaxConfig.apply(closedLoopConfig);
        this.sparkMaxConfig.apply(encoderConfig);

        if(followCanId.isPresent()){
            this.sparkMaxConfig.follow(followCanId.getAsInt());
        }
        if(softLimitConfig.isPresent()){
            this.sparkMaxConfig.apply(softLimitConfig.get());
        }
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
