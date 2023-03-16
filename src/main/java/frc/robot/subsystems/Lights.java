package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{

AddressableLED led = new AddressableLED(3);

AddressableLEDBuffer buffer = new AddressableLEDBuffer(100);

public Lights() {
    led.setLength(100);
    for (int i = 0; i< 100; i++) {
        buffer.setRGB(i, 0, 0, 255);
    }
    led.setData(buffer);
    led.start();
}




}
