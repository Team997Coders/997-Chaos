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
import org.chsrobotics.offseasonRobot2022.Constants.CommandConstants.TeleopDriveCommandConstants;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;

/** Command for teleoperated driving with feedforward for smoothing. */
public class FeedforwardDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final Supplier<Double> linearInput;
    private final Supplier<Double> rotationalInput;

    /**
     * Constructs a FeedforwardDrive.
     *
     * @param drivetrain The Drivetrain subsystem used by this command.
     * @param linearInput A lambda of the linear input.
     * @param rotationalInput A lambda of the rotation input.
     */
    public FeedforwardDrive(
            Drivetrain drivetrain, Supplier<Double> linearInput, Supplier<Double> rotationalInput) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.linearInput = linearInput;
        this.rotationalInput = rotationalInput;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double left = linearInput.get() + rotationalInput.get();
        double right = linearInput.get() - rotationalInput.get();

        drivetrain.setVoltages(
                TeleopDriveCommandConstants.LEFT_FEEDFORWARD.calculate(left),
                TeleopDriveCommandConstants.RIGHT_FEEDFORWARD.calculate(right));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
