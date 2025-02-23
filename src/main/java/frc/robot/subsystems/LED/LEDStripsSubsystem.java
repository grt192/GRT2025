import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

package frc.robot.subsystems.LED;

public class LEDStrips {

private AddressableLED led;
private AddressableLEDBuffer ledBuffer;
private int length;

public LEDStrips(int pwmPort, int length) {
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

public void turnOffLEDs() {
    for (int i = 0; i < length; i++) {
        ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
}

public int getLength() {
    return length;
}
k
}
