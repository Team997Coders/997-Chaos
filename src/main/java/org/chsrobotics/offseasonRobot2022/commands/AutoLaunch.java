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
package org.chsrobotics.offseasonRobot2022.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.HashMap;
import java.util.List;
import org.chsrobotics.lib.commands.ConsumerCommand;
import org.chsrobotics.lib.util.Tuple2;
import org.chsrobotics.offseasonRobot2022.Constants.CommandConstants.AutoLaunchConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.DrivetrainConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.LEDDisplayConstants;
import org.chsrobotics.offseasonRobot2022.Localizer;
import org.chsrobotics.offseasonRobot2022.commands.drive.AutoPivot;
import org.chsrobotics.offseasonRobot2022.commands.index.Index;
import org.chsrobotics.offseasonRobot2022.commands.launcher.LauncherController;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;
import org.chsrobotics.offseasonRobot2022.subsystems.Indexer;
import org.chsrobotics.offseasonRobot2022.subsystems.LEDDisplay;
import org.chsrobotics.offseasonRobot2022.subsystems.Launcher;

/** */
public class AutoLaunch extends ParallelCommandGroup {

    /**
     * @param localization
     * @param drivetrain
     * @param launcher
     * @param displays
     * @param indexer
     * @param trackingSetpoints
     */
    public AutoLaunch(
            Localizer localization,
            Drivetrain drivetrain,
            Launcher launcher,
            LEDDisplay display,
            Indexer indexer,
            HashMap<Double, List<Double>> trackingSetpoints) {

        if (indexer.hasBall()) {
            ConsumerCommand<Integer> ledPreparing =
                    new ConsumerCommand<Integer>(
                            display::setAnimation,
                            LEDDisplayConstants.LAUNCHER_PREPARING_LED_ANIMATION);

            ledPreparing.setName("LEDAnimationPreparing");

            ConsumerCommand<Double> drivetrainStop =
                    new ConsumerCommand<>(
                            drivetrain::setVoltages,
                            Tuple2.of(0.0, 0.0),
                            null,
                            AutoLaunchConstants.DRIVETRAIN_INITIAL_FREEZE_DURATION_SECONDS,
                            drivetrain);
            drivetrainStop.setName("DrivetrainStop");

            ConsumerCommand<NeutralMode> drivetrainBrake =
                    new ConsumerCommand<>(
                            drivetrain::setNeutralmode,
                            NeutralMode.Brake,
                            DrivetrainConstants.DEFAULT_NEUTRAL_MODE,
                            AutoLaunchConstants.DRIVETRAIN_INITIAL_FREEZE_DURATION_SECONDS);
            drivetrainBrake.setName("DrivetrainBrake");

            ConsumerCommand<Integer> ledLaunching =
                    new ConsumerCommand<>(
                            display::setAnimation,
                            LEDDisplayConstants.LAUNCHER_LAUNCHING_LED_ANIMATION,
                            drivetrain);
            ledLaunching.setName("LEDAnimationLaunching");

            LauncherController launcherController =
                    new LauncherController(
                            launcher,
                            this::getDesiredFlywheelVelocityRadiansPerSecond,
                            this::getDesiredHoodAngleRadians);

            AutoPivot autoPivot =
                    new AutoPivot(drivetrain, this::getDesiredFieldRelRotationRadians);

            Index indexCommand =
                    new Index(indexer, autoPivot::atSetpoint, launcherController::atSetpoints);

            addCommands(
                    new SequentialCommandGroup(
                            ledPreparing,
                            new ParallelCommandGroup(drivetrainStop, drivetrainBrake),
                            autoPivot,
                            ledLaunching),
                    launcherController,
                    indexCommand);
        }
    }

    @SuppressWarnings("unused")
    private double getDistanceMetersToTarget() {
        // TODO: implement
        return 0;
    }

    private double getDesiredFieldRelRotationRadians() {
        // TODO: implement
        return 0;
    }

    private double getDesiredHoodAngleRadians() {
        return 0;
    }

    private double getDesiredFlywheelVelocityRadiansPerSecond() {
        return 0;
    }
}
