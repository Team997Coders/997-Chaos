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
package org.chsrobotics.offseasonRobot2022.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.offseasonRobot2022.Constants.CommandConstants.IndexCommandConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.IndexerConstants;
import org.chsrobotics.offseasonRobot2022.subsystems.Indexer;

public class Index extends CommandBase {
    private final Indexer indexer;

    private final Supplier<Boolean> drivetrainAligned;
    private final Supplier<Boolean> launcherAtSetpoints;

    private final String subdirString = "indexCommand";

    private final Logger<String> stateLogger = new Logger<>("state", subdirString);

    /**
     * Constructor for an Index command which requires external conditions to be met before feeding
     * to the flywheel.
     *
     * @param indexer Indexer subsystem to use.
     * @param drivetrainAligned
     * @param launcherAtSetpoints
     */
    public Index(
            Indexer indexer,
            Supplier<Boolean> drivetrainAligned,
            Supplier<Boolean> launcherAtSetpoints) {
        this.indexer = indexer;
        addRequirements(indexer);

        this.drivetrainAligned = drivetrainAligned;
        this.launcherAtSetpoints = launcherAtSetpoints;
    }

    @Override
    public void initialize() {
        indexer.setIntakeVoltage(IndexCommandConstants.INTAKE_MOTOR_OPEN_LOOP_VOLTAGE);
    }

    @Override
    public void execute() {
        if (drivetrainAligned.get() && launcherAtSetpoints.get()) {
            if (indexer.ballAtLowerBreakbeam()) {
                stateLogger.update("alignedFeedToFlywheel");
                indexer.setUpperServo(IndexerConstants.UPPER_SERVO_CLOSED);
                indexer.setLowerServo(IndexerConstants.LOWER_SERVO_OPEN);
            } else if (indexer.ballAtUpperBreakbeam()) {
                stateLogger.update("alignedFeedToLowerGate");
                indexer.setUpperServo(IndexerConstants.UPPER_SERVO_OPEN);
                indexer.setLowerServo(IndexerConstants.LOWER_SERVO_CLOSED);
            } else {
                stateLogger.update("alignedNoBalls");
                indexer.setUpperServo(IndexerConstants.UPPER_SERVO_CLOSED);
                indexer.setLowerServo(IndexerConstants.LOWER_SERVO_CLOSED);
            }
        } else if (indexer.ballAtUpperBreakbeam() && !indexer.ballAtLowerBreakbeam()) {
            stateLogger.update("unalignedFeedToLowerGate");
            indexer.setUpperServo(IndexerConstants.UPPER_SERVO_OPEN);
            indexer.setLowerServo(IndexerConstants.LOWER_SERVO_CLOSED);
        } else {
            if (indexer.ballAtLowerBreakbeam()) stateLogger.update("unalignedNoFeed");
            else stateLogger.update("unalignedNoBalls");

            indexer.setUpperServo(IndexerConstants.UPPER_SERVO_CLOSED);
            indexer.setLowerServo(IndexerConstants.LOWER_SERVO_CLOSED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setUpperServo(IndexerConstants.UPPER_SERVO_CLOSED);
        indexer.setLowerServo(IndexerConstants.LOWER_SERVO_CLOSED);

        indexer.setIntakeVoltage(0);
    }
}
