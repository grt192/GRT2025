package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class LEDStripsSubsystem {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int length;

    public LEDStripsSubsystem(int id, int length) {
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

    public void setLEDColor(int index, int[] color) {
        if (index >= 0 && index < length) {
            ledBuffer.setRGB(index, color[0], color[1], color[2]);
            led.setData(ledBuffer);
        }
    }

    public void setAllLEDsColor(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        led.setData(ledBuffer);
    }

    public void setAllLEDsColor(int[] color) {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
        led.setData(ledBuffer);
    }

    public void setRainbow() {
        for (int i = 0; i < length; i++) {
            final int hue = (i * 180 / length) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        led.setData(ledBuffer);
    }

    public void pulseLEDs(int r, int g, int b, int delay) {
        new Thread(() -> {
            try {
                while (true) {
                    for (int i = 0; i <= 255; i++) {
                        setAllLEDsColor((r * i) / 255, (g * i) / 255, (b * i) / 255);
                        Thread.sleep(delay);
                    }
                    for (int i = 255; i >= 0; i--) {
                        setAllLEDsColor((r * i) / 255, (g * i) / 255, (b * i) / 255);
                        Thread.sleep(delay);
                    }
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    public void turnOffLEDs() {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
    }

    public int getLength() {
        return length;
    }



}

