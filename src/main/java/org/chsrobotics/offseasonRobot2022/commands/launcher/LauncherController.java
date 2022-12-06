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
package org.chsrobotics.offseasonRobot2022.commands.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.chsrobotics.lib.controllers.PID;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.offseasonRobot2022.Constants.GlobalConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.LauncherConstants;
import org.chsrobotics.offseasonRobot2022.subsystems.Launcher;

/** */
public class LauncherController extends CommandBase {
    private final PID flywheelController =
            new PID(
                    LauncherConstants.FLYWHEEL_KP,
                    LauncherConstants.FLYWHEEL_KI,
                    LauncherConstants.FLYWHEEL_KD,
                    0);

    @SuppressWarnings("unused")
    private final SimpleMotorFeedforward flywheelFeedforward =
            new SimpleMotorFeedforward(
                    LauncherConstants.FLYWHEEL_KS,
                    LauncherConstants.FLYWHEEL_KV,
                    LauncherConstants.FLYWHEEL_KA);

    private final PID hoodController =
            new PID(
                    LauncherConstants.HOOD_KP,
                    LauncherConstants.HOOD_KI,
                    LauncherConstants.HOOD_KD,
                    0);

    @SuppressWarnings("unused")
    private final ArmFeedforward hoodFeedforward =
            new ArmFeedforward(
                    LauncherConstants.HOOD_KS,
                    LauncherConstants.HOOD_KG_GAIN,
                    LauncherConstants.HOOD_KV,
                    LauncherConstants.HOOD_KA);

    private final Launcher launcher;

    private final Supplier<Double> flywheelSetpointLambda;

    private final Supplier<Double> hoodSetpointLambda;

    private final String subdirString = "launcherController";

    private final Logger<Double> hoodSetpointLogger =
            new Logger<>("hoodSetpointRadians", subdirString);

    private final Logger<Double> flywheelSetpointLogger =
            new Logger<>("flywheelSetpointRadiansPerSecond", subdirString);

    /**
     * @param launcher
     * @param flywheelSetpoint
     * @param hoodSetpoint
     */
    public LauncherController(
            Launcher launcher, Supplier<Double> flywheelSetpoint, Supplier<Double> hoodSetpoint) {
        this.launcher = launcher;
        this.flywheelSetpointLambda = flywheelSetpoint;
        this.hoodSetpointLambda = hoodSetpoint;
        addRequirements(launcher);
    }

    /**
     * @param launcher
     * @param flywheelSetpoint
     * @param hoodSetpoint
     */
    public LauncherController(Launcher launcher, double flywheelSetpoint, double hoodSetpoint) {
        this(launcher, () -> flywheelSetpoint, () -> hoodSetpoint);
    }

    /**
     * @return
     */
    public boolean atSetpoints() {
        return (flywheelController.isAtSetpoint() && hoodController.isAtSetpoint());
    }

    @Override
    public void execute() {
        double flywheelSetpoint = flywheelSetpointLambda.get();
        double hoodSetpoint = hoodSetpointLambda.get();

        flywheelSetpointLogger.update(flywheelSetpoint);
        hoodSetpointLogger.update(hoodSetpoint);

        flywheelController.setSetpoint(flywheelSetpoint);
        hoodController.setSetpoint(hoodSetpoint);

        // TODO: figure out what the velocity and acceleration parameters of the FF should be
        double flywheelVolts =
                MathUtil.clamp(
                        flywheelController.calculate(
                                launcher.getFlywheelVelocityRadiansPerSecond(),
                                flywheelSetpointLambda.get()),
                        -GlobalConstants.GLOBAL_NOMINAL_VOLTAGE_VOLTS,
                        GlobalConstants.GLOBAL_NOMINAL_VOLTAGE_VOLTS);

        double hoodVolts =
                MathUtil.clamp(
                        hoodController.calculate(
                                launcher.getHoodPositionRadians(), hoodSetpointLambda.get()),
                        -GlobalConstants.GLOBAL_NOMINAL_VOLTAGE_VOLTS,
                        GlobalConstants.GLOBAL_NOMINAL_VOLTAGE_VOLTS);

        launcher.setFlywheelVoltage(flywheelVolts);
        launcher.setHoodVoltage(hoodVolts);
    }

    @Override
    public void end(boolean interrupted) {
        launcher.setFlywheelVoltage(0);
        launcher.setHoodVoltage(0);
    }
}
