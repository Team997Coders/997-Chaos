/**
Copyright 2022 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
*/
package org.chsrobotics.offseasonRobot2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.IndexerConstants;
import org.chsrobotics.offseasonRobot2022.Robot;

public class Indexer extends SubsystemBase {
    private final DigitalInput lowerBreakbeam =
            new DigitalInput(IndexerConstants.LOWER_BREAKBEAM_DIO);
    private final DigitalInput upperBreakbeam =
            new DigitalInput(IndexerConstants.UPPER_BREAKBEAM_DIO);

    private final Servo lowerServo = new Servo(IndexerConstants.LOWER_SERVO_PWM);
    private final Servo upperServo = new Servo(IndexerConstants.UPPER_SERVO_PWM);

    private final CANSparkMax intakeMotor =
            new CANSparkMax(IndexerConstants.INTAKE_MOTOR_CANID, MotorType.kBrushless);

    private final String subdirString = "indexer";

    private final Logger<Boolean> upperBreakbeamStateLogger =
            new Logger<>("upperBreakbeamBallPresent", subdirString);
    private final Logger<Boolean> lowerBreakbeamStateLogger =
            new Logger<>("lowerBreakbeamBallPresent", subdirString);

    private final Logger<Double> upperServoSetPositionLogger =
            new Logger<>("upperServoSetPosition", subdirString);
    private final Logger<Double> lowerServoSetPositionLogger =
            new Logger<>("lowerServoSetPosition", subdirString);

    private final Logger<Double> motorSetVoltageLogger =
            new Logger<>("motorSetVoltage", subdirString);
    private final Logger<Double> motorCurrentLogger = new Logger<>("motorCurrent", subdirString);
    private final Logger<Double> motorTempCLogger = new Logger<>("motorTempC", subdirString);

    public Indexer() {
        intakeMotor.setSmartCurrentLimit(IndexerConstants.INTAKE_MOTOR_CURRENT_LIMIT);

        intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * @return
     */
    public boolean ballAtLowerBreakbeam() {
        return IndexerConstants.LOWER_BREAMBEAM_BALL_PRESENT_WHEN_TRUE
                ? lowerBreakbeam.get()
                : !lowerBreakbeam.get();
    }

    /**
     * Returns whether a ball is sensed at the upper breakbeam of the indexer.
     *
     * @return The corrected value of the upper breakbeam.
     */
    public boolean ballAtUpperBreakbeam() {
        return IndexerConstants.UPPER_BREAKBEAM_BALL_PRESENT_WHEN_TRUE
                ? upperBreakbeam.get()
                : !upperBreakbeam.get();
    }

    public void setLowerServo(double position) {
        lowerServo.set(position);

        lowerServoSetPositionLogger.update(position);
    }

    public void setUpperServo(double position) {
        upperServo.set(position);

        upperServoSetPositionLogger.update(position);
    }

    public boolean hasBall() {
        return (ballAtLowerBreakbeam() || ballAtUpperBreakbeam());
    }

    public void feed() {}

    public void setIntakeVoltage(double volts) {
        double pVolts = IndexerConstants.INTAKE_MOTOR_INVERTED ? -volts : volts;

        intakeMotor.setVoltage(pVolts);

        motorSetVoltageLogger.update(pVolts);
    }

    @Override
    public void periodic() {
        upperBreakbeamStateLogger.update(ballAtUpperBreakbeam());
        lowerBreakbeamStateLogger.update(ballAtLowerBreakbeam());

        motorCurrentLogger.update(Robot.getCurrentAmps(IndexerConstants.INTAKE_MOTOR_PDP_CHANNEL));
        motorTempCLogger.update(intakeMotor.getMotorTemperature());
    }
}
