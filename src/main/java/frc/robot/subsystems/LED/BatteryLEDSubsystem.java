package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import frc.robot.Constants;

public class BatteryLEDSubsystem {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int length;

    public BatteryLEDSubsystem(int id, int length) {
        this.length = length;
        led = new AddressableLED(id);
        ledBuffer = new AddressableLEDBuffer(length);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void setLEDColor(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            ledBuffer.setRGB(index, r, g, b);
            led.setData(ledBuffer);
        }
    }

    public void updateBatteryLevel(double batteryPercentage) {
        int ledsToLight = (int) Math.ceil((batteryPercentage / 100.0) * length);
        int[] color;

        if (batteryPercentage > 50) {
            color =  Constants.GREEN;
        } else if (batteryPercentage > 20) {
            color = Constants.YELLOW;
        } else {
            color = Constants.RED;
        }
        if (color == null) {
            color = Constants.OFF;
        }
        for (int i = 0; i < length; i++) {
            if (i < ledsToLight) {
                setLEDColor(i, color[0], color[1], color[2]);
            } else {
                setLEDColor(i, Constants.OFF[0], Constants.OFF[1], Constants.OFF[2]);
            }
        }
    }
}