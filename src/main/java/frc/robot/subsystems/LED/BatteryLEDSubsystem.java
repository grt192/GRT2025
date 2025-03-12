package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class BatteryLEDSubsystem {
    // Hardware components
    private final AddressableLED led;          // LED controller object
    private final AddressableLEDBuffer ledBuffer;  // LED color buffer
    private final int length;                  // Number of LEDs in strip

    // Constructor: Set up LED hardware
    public BatteryLEDSubsystem(int id, int length) {
        this.length = length;
        
        // Initialize LED controller on specified PWM port
        led = new AddressableLED(id);
        
        // Create buffer for storing LED colors
        ledBuffer = new AddressableLEDBuffer(length);

        // Configure hardware
        led.setLength(length);     // Set number of LEDs
        led.setData(ledBuffer);    // Initialize with empty buffer
        led.start();               // Begin outputting data to LEDs
    }

    // Internal method to set individual LED colors
    private void setLEDColor(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            ledBuffer.setRGB(index, r, g, b);  // Update buffer without pushing to hardware
        }
    }

    // Public method to update display based on battery level
    public void updateBatteryLevel(double batteryPercentage) {
        // Calculate how many LEDs to illuminate
        int ledsToLight = (int) Math.ceil((batteryPercentage / 100.0) * length);
        
        // Get appropriate color from Constants
        int[] color = determineColor(batteryPercentage);

        // Update all LEDs in the strip
        for (int i = 0; i < length; i++) {
            if (i < ledsToLight) {
                setLEDColor(i, color[0], color[1], color[2]);  // Active segment
            } else {
                setLEDColor(i, Constants.OFF[0], Constants.OFF[1], Constants.OFF[2]);  // Off segment
            }
        }
        
        // Push all changes to hardware at once
        led.setData(ledBuffer);
    }

    // Determine color based on battery percentage
    private int[] determineColor(double batteryPercentage) {
        if (batteryPercentage > 50) {
            return Constants.GREEN;
        } else if (batteryPercentage > 20) {
            return Constants.YELLOW;
        } else {
            return Constants.RED;
        }
    }
}