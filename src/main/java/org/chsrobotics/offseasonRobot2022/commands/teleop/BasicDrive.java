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
package org.chsrobotics.offseasonRobot2022.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.offseasonRobot2022.Constants;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;

/** Command to move the drivetrain with a basic arcade drive and no input processing. */
public class BasicDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final Supplier<Double> linInput;
    private final Supplier<Double> rotInput;

    /**
     * Constructs a BasicDrive.
     *
     * @param drivetrain The Drivetrain subsystem for this command to use.
     * @param linInput Lambda of the desired linear input, in [-1,1] (inclusive).
     * @param rotInput Lambda of a desired rotation input, in [-1,1] (inclusive).
     */
    public BasicDrive(Drivetrain drivetrain, Supplier<Double> linInput, Supplier<Double> rotInput) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.linInput = linInput;
        this.rotInput = rotInput;
    }

    @Override
    public void initialize() {
        HighLevelLogger.logMessage("BasicDrive command started");
    }

    @Override
    public void execute() {
        drivetrain.setVoltages(
                Constants.GlobalConstants.GLOBAL_NOMINAL_VOLTAGE
                        * (linInput.get() + rotInput.get()),
                Constants.GlobalConstants.GLOBAL_NOMINAL_VOLTAGE
                        * (linInput.get() - rotInput.get()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setVoltages(0, 0);
        HighLevelLogger.logMessage("BasicDrive command ended: " + interrupted);
    }
}
