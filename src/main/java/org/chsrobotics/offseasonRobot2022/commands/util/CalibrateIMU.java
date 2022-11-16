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
package org.chsrobotics.offseasonRobot2022.commands.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.offseasonRobot2022.subsystems.InertialMeasurementUnit;

/** Command to trigger the calibration sequence of the robot's IMU. */
public class CalibrateIMU extends CommandBase {
    private final Timer timer = new Timer();
    private final InertialMeasurementUnit IMU;

    /**
     * Constructs a CalibrateIMU command.
     *
     * @param IMU The IMU to calibrate.
     */
    public CalibrateIMU(InertialMeasurementUnit IMU) {
        addRequirements(IMU);
        this.IMU = IMU;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        IMU.startCalibration();
        timer.reset();
        timer.start();
        HighLevelLogger.logMessage("CalibrateIMU started");
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        HighLevelLogger.logMessage("CalibrateIMU ended: " + timer.get() + " seconds");
    }

    @Override
    public boolean isFinished() {
        return IMU.isCalibrated();
    }
}
