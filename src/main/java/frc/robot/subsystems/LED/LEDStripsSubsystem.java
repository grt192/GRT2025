package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class LEDStripsSubsystem {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final int length;
    private final ScheduledExecutorService executor;
    private volatile boolean isPulsing;

    public LEDStripsSubsystem(int id, int length) {
        this.length = length;
        led = new AddressableLED(id);
        ledBuffer = new AddressableLEDBuffer(length);
        executor = Executors.newSingleThreadScheduledExecutor();
        
        led.setLength(length);
        led.setData(ledBuffer);
        led.start();
    }

    // Synchronized methods for thread-safe buffer access
    public synchronized void setLEDColor(int index, int r, int g, int b) {
        if (index >= 0 && index < length) {
            ledBuffer.setRGB(index, r, g, b);
        }
    }

    public synchronized void setLEDColor(int index, int[] color) {
        if (color != null && color.length >= 3 && index >= 0 && index < length) {
            ledBuffer.setRGB(index, color[0], color[1], color[2]);
        }
    }

    public synchronized void setAllLEDsColor(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        updateLEDs();
    }

    public synchronized void setAllLEDsColor(int[] color) {
        if (color != null && color.length >= 3) {
            setAllLEDsColor(color[0], color[1], color[2]);
        }
    }

    public synchronized void setRainbow() {
        for (int i = 0; i < length; i++) {
            final int hue = (i * 180 / length) % 180;
            ledBuffer.setHSV(i, hue, 255, 255); // Increased brightness to 255
        }
        updateLEDs();
    }

    public void pulseLEDs(int r, int g, int b, int delayMs) {
        stopAnimation();
        isPulsing = true;
        
        executor.scheduleAtFixedRate(() -> {
            synchronized (this) {
                if (!isPulsing) return;
                
                for (int i = 0; i <= 255; i++) {
                    setAllLEDsColor((r * i) / 255, (g * i) / 255, (b * i) / 255);
                    updateLEDs();
                    sleepSafely(delayMs);
                }
                for (int i = 255; i >= 0; i--) {
                    setAllLEDsColor((r * i) / 255, (g * i) / 255, (b * i) / 255);
                    updateLEDs();
                    sleepSafely(delayMs);
                }
            }
        }, 0, 1, TimeUnit.MILLISECONDS);
    }

    public synchronized void turnOffLEDs() {
        stopAnimation();
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        updateLEDs();
    }

    public synchronized void updateLEDs() {
        led.setData(ledBuffer);
    }

    public synchronized void stopAnimation() {
        isPulsing = false;
        executor.shutdownNow();
    }

    public int getLength() {
        return length;
    }

    private void sleepSafely(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}