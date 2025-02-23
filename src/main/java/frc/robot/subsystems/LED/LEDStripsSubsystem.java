package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStripsSubsystem {

private AddressableLED led;
private AddressableLEDBuffer ledBuffer;
private int length;

public LEDStripsSubsystem(int pwmPort, int length) {
    this.length = length;
    led = new AddressableLED(pwmPort);
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

public void setAllLEDsColor(int r, int g, int b) {
    for (int i = 0; i < length; i++) {
        ledBuffer.setRGB(i, r, g, b);
    }
    led.setData(ledBuffer);
}

public void setRed() {
    setAllLEDsColor(255, 0, 0);
}

public void setGreen() {
    setAllLEDsColor(0, 255, 0);
}

public void setBlue() {
    setAllLEDsColor(0, 0, 255);
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

