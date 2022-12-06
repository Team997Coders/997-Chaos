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
package org.chsrobotics.offseasonRobot2022.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.offseasonRobot2022.Constants.CommandConstants.AutoPivotConstants;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;

/** */
public class AutoPivot extends CommandBase {
    private final Drivetrain drivetrain;

    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    AutoPivotConstants.KP,
                    AutoPivotConstants.KI,
                    AutoPivotConstants.kD,
                    new Constraints(
                            AutoPivotConstants.MAX_V_RADS_PER_SEC,
                            AutoPivotConstants.MAX_A_RADS_PER_SEC_SQ));

    private final Supplier<Double> desiredGyroAngleRadiansLambda;

    private final String subdirString = "autoPivot";

    private final Logger<Double> setpointAngleRadiansLogger =
            new Logger<>("setpointRadians", subdirString);

    /**
     * @param drivetrain
     * @param desiredGyroAngleRadiansLambda
     */
    public AutoPivot(Drivetrain drivetrain, Supplier<Double> desiredGyroAngleRadiansLambda) {
        this.drivetrain = drivetrain;
        this.desiredGyroAngleRadiansLambda = desiredGyroAngleRadiansLambda;

        controller.enableContinuousInput(0, 2 * Math.PI);
        controller.setTolerance(Units.degreesToRadians(1), Units.degreesToRadians(0.2));
    }

    /**
     * @param drivetrain
     * @param desiredGyroAngleRadians
     */
    public AutoPivot(Drivetrain drivetrain, double desiredGyroAngleRadians) {
        this(drivetrain, () -> desiredGyroAngleRadians);
    }

    /**
     * @return
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    @Override
    public void execute() {
        double goal = desiredGyroAngleRadiansLambda.get();

        controller.setGoal(goal);

        setpointAngleRadiansLogger.update(goal);

        double feedback = controller.calculate(drivetrain.getRotationGyro().getRadians());
        double feedforward = 0;

        drivetrain.setVoltages(-(feedback + feedforward), feedback + feedforward);
    }
}
