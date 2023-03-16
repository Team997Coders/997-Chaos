package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final Servo gate = new Servo(Constants.Shooter.SERVO_CHANNEL);
  
    private final MotorControllerGroup flywheel = new MotorControllerGroup(new CANSparkMax(Constants.Shooter.TOP_FLYWHEEL_CAN_ID, MotorType.kBrushless), new CANSparkMax(Constants.Shooter.BOTTOM_FLYWHEEL_CAN_ID, MotorType.kBrushless));

    public Shooter() {
        flywheel.setInverted(Constants.Shooter.FLYWHEEL_IS_INVERTED);
        
    }

    public void setFlywheelOutput(double speed) {
        flywheel.set(speed);
    }

    public double getFlywheelSpeed() {
        return flywheel.get();
    }
 
    public void setServoAngle(double angle) {
        gate.setAngle(angle);
    }
    
    
}
