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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.DrivetrainConstants;

/** Drivetrain subsystem of the robot, containing drive motors and encoders. */
public class Drivetrain extends SubsystemBase {
    private final VictorSPX frontLeft;
    private final VictorSPX backLeft;

    private final VictorSPX frontRight;
    private final VictorSPX backRight;

    private final Encoder rightEncoder;
    private final Encoder leftEncoder;

    private final String subdirString = "drivetrain";

    private final Logger<Double> rightPositionLogger =
            new Logger<>("rightPositionMeters", subdirString);
    private final Logger<Double> leftPositionLogger =
            new Logger<>("leftPositionMeters", subdirString);
    private final Logger<Double> rightVelocityLogger =
            new Logger<>("rightVelocityMetersPerSecond", subdirString);
    private final Logger<Double> leftVelocityLogger =
            new Logger<>("leftVelocityMetersPerSecond", subdirString);

    /** Constructs a Drivetrain. */
    public Drivetrain() {
        frontLeft = new VictorSPX(DrivetrainConstants.FRONT_LEFT_CANID);
        backLeft = new VictorSPX(DrivetrainConstants.BACK_LEFT_CANID);

        frontRight = new VictorSPX(DrivetrainConstants.FRONT_RIGHT_CANID);
        backRight = new VictorSPX(DrivetrainConstants.BACK_RIGHT_CANID);

        rightEncoder =
                new Encoder(
                        DrivetrainConstants.RIGHT_ENCODER_CHANNEL_A,
                        DrivetrainConstants.RIGHT_ENCODER_CHANNEL_B);
        leftEncoder =
                new Encoder(
                        DrivetrainConstants.LEFT_ENCODER_CHANNEL_A,
                        DrivetrainConstants.LEFT_ENCODER_CHANNEL_B);
    }

    /**
     * Sets the voltages of each pair of drivetrain motors.
     *
     * @param leftSideVoltage The voltage (in volts) to apply to the front and back left motors.
     * @param rightSideVoltage The voltage (in volts) to apply to the front and back right motors.
     */
    public void setVoltages(double leftSideVoltage, double rightSideVoltage) {
        frontLeft.set(ControlMode.PercentOutput, leftSideVoltage / frontLeft.getBusVoltage());
        backLeft.set(ControlMode.PercentOutput, leftSideVoltage / backLeft.getBusVoltage());

        frontRight.set(ControlMode.PercentOutput, rightSideVoltage / frontRight.getBusVoltage());
        backRight.set(ControlMode.PercentOutput, rightSideVoltage / backRight.getBusVoltage());
    }

    /**
     * Returns the distance travelled by the left-side encoder since the last reset.
     *
     * @return The distance, in meters.
     */
    public double getLeftSideDistanceMeters() {
        return DrivetrainConstants.ENCODER_TO_GROUND_HELPER.outputFromInput(
                leftEncoder.getDistance() / DrivetrainConstants.ENCODER_CPR);
    }

    /**
     * Returns the distance travelled by the right-side encoder since the last reset.
     *
     * @return The distance, in meters.
     */
    public double getRightSideDistanceMeters() {
        return DrivetrainConstants.ENCODER_TO_GROUND_HELPER.outputFromInput(
                rightEncoder.getDistance() / DrivetrainConstants.ENCODER_CPR);
    }

    /**
     * Returns the smoothed velocity of the left-side encoder.
     *
     * @return The filtered velocity, in meters/second.
     */
    public double getLeftSideVelocityMetersPerSecond() {
        return DrivetrainConstants.LEFT_ENCODER_VELOCITY_SMOOTHING.calculate(
                DrivetrainConstants.ENCODER_TO_GROUND_HELPER.outputFromInput(
                        leftEncoder.getRate() / DrivetrainConstants.ENCODER_CPR));
    }

    /**
     * Returns the smoothed velocity of the right-side encoder.
     *
     * @return The filtered velocity, in meters/second.
     */
    public double getRightSideVelocityMetersPerSecond() {
        return DrivetrainConstants.RIGHT_ENCODER_VELOCITY_SMOOTHING.calculate(
                DrivetrainConstants.ENCODER_TO_GROUND_HELPER.outputFromInput(
                        rightEncoder.getRate() / DrivetrainConstants.ENCODER_CPR));
    }

    /** Resets the position of right- and left- side encoders. */
    public void resetEncoders() {
        rightEncoder.reset();
        leftEncoder.reset();
    }

    @Override
    public void periodic() {
        leftPositionLogger.update(getLeftSideDistanceMeters());
        rightPositionLogger.update(getRightSideDistanceMeters());
        leftVelocityLogger.update(getLeftSideVelocityMetersPerSecond());
        rightVelocityLogger.update(getRightSideVelocityMetersPerSecond());
    }

    @Override
    public void simulationPeriodic() {}
}
