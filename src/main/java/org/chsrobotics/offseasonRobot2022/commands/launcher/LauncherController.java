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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.chsrobotics.offseasonRobot2022.Constants.GlobalConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.LauncherConstants;
import org.chsrobotics.offseasonRobot2022.subsystems.Launcher;

/** */
public class LauncherController extends CommandBase {
    private final PIDController flywheelController =
            new PIDController(
                    LauncherConstants.FLYWHEEL_KP,
                    LauncherConstants.FLYWHEEL_KI,
                    LauncherConstants.FLYWHEEL_KD);

    @SuppressWarnings("unused")
    private final SimpleMotorFeedforward flywheelFeedforward =
            new SimpleMotorFeedforward(
                    LauncherConstants.FLYWHEEL_KS,
                    LauncherConstants.FLYWHEEL_KV,
                    LauncherConstants.FLYWHEEL_KA);

    private final ProfiledPIDController hoodController =
            new ProfiledPIDController(
                    LauncherConstants.HOOD_KP,
                    LauncherConstants.HOOD_KI,
                    LauncherConstants.HOOD_KD,
                    new Constraints(
                            LauncherConstants.HOOD_MAX_VEL, LauncherConstants.HOOD_MAX_ACCEL));

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
        return (flywheelController.atSetpoint() && hoodController.atSetpoint());
    }

    @Override
    public void execute() {
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
