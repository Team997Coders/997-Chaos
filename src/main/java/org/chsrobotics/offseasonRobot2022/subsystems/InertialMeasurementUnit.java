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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.InertialMeasurementUnitConstants;
import org.chsrobotics.offseasonRobot2022.Robot;

/** Inertial Measurement Unit (gyroscope, magnetometer, accelerometer) subsystem of the robot. */
public class InertialMeasurementUnit extends SubsystemBase {
    private AHRS IMU;
    private boolean calibrationStarted = false;

    private final String subdirString = "IMU";

    private final Logger<Double> gyroAngleLogger = new Logger<>("gyroAngleRads", subdirString);
    private final Logger<Boolean> calibrationStateLogger =
            new Logger<>("isCalibrating", subdirString);

    private int navXSimHandle;
    private SimDouble navXGyroAngle;
    private final Timer simCalibrationTimer = new Timer();

    public InertialMeasurementUnit() {
        if (Robot.isReal()) {
            IMU = new AHRS(InertialMeasurementUnitConstants.IMU_PORT);
        } else {
            navXSimHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            navXGyroAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSimHandle, "Yaw"));
        }
    }

    /**
     * Returns the current relative orientation of the IMU's gyroscope.
     *
     * @return The rotation as a Rotation2d.
     */
    public Rotation2d getRotationGyro() {
        if (Robot.isReal()) {
            return IMU.getRotation2d();
        } else {
            return Rotation2d.fromDegrees(90 - navXGyroAngle.get());
        }
    }

    /**
     * Begins calibrating the IMU's accelerometer, gyroscope, and magnetometer.
     *
     * <p>Must be done while the robot is not moving.
     */
    public void startCalibration() {
        if (Robot.isReal()) {
            IMU.calibrate();
        } else {
            simCalibrationTimer.start();
        }
        calibrationStarted = true;
    }

    /**
     * Returns true if the IMU is currently in the calibration sequence.
     *
     * @return Whether the IMU is calibrating.
     */
    public boolean isCalibrating() {
        if (Robot.isReal()) {
            return IMU.isCalibrating();
        } else {
            if (simCalibrationTimer.get() < 2) {
                return true;
            } else {
                simCalibrationTimer.stop();
                return false;
            }
        }
    }

    /**
     * Returns true if the IMU has finished the calibration sequence.
     *
     * @return Whether the IMU has finished self-calibrating.
     */
    public boolean isCalibrated() {
        return (calibrationStarted && !isCalibrating());
    }

    @Override
    public void periodic() {
        gyroAngleLogger.update(getRotationGyro().getRadians());
        calibrationStateLogger.update(isCalibrating());
    }
}
