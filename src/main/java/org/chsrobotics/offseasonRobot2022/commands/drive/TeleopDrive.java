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

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.Map;
import org.chsrobotics.lib.drive.differential.ArcadeDrive;
import org.chsrobotics.lib.drive.differential.CurvatureDrive;
import org.chsrobotics.lib.drive.differential.DifferentialDriveMode;
import org.chsrobotics.lib.drive.differential.MixedDrive;
import org.chsrobotics.lib.drive.differential.TankDrive;
import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.telemetry.DashboardChooser.ValueUpdater;
import org.chsrobotics.offseasonRobot2022.Config;
import org.chsrobotics.offseasonRobot2022.Config.DriveMode;
import org.chsrobotics.offseasonRobot2022.Constants;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;

/** */
public class TeleopDrive extends CommandBase implements ValueUpdater<DriveMode> {
    private final Drivetrain drivetrain;
    private final JoystickAxis axisA;
    private final JoystickAxis axisB;

    private DifferentialDriveMode driveMode;

    /** */
    public TeleopDrive(Drivetrain drivetrain, JoystickAxis axisA, JoystickAxis axisB) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;

        this.axisA = axisA;
        this.axisB = axisB;

        updateDriveProfile();

        Config.driveModeChooser.registerListener(this);
    }

    private void updateDriveProfile() {
        double driveMod = Config.driveLinModifierChooser.getSelected().modifier;
        double turnMod = Config.driveRotModifierChooser.getSelected().modifier;

        double driveLim = Config.driveLinLimitingChooser.getSelected().limit;
        double turnLim = Config.driveRotLimitingChooser.getSelected().limit;

        DifferentialDriveMode arcadeDrive =
                new ArcadeDrive(axisA, axisB, driveMod, turnMod, driveLim, turnLim);
        DifferentialDriveMode curvatureDrive =
                new CurvatureDrive(axisA, axisB, driveMod, turnMod, driveLim, turnLim, false);
        DifferentialDriveMode tankDrive = new TankDrive(axisA, axisB, driveMod, driveLim);

        switch (Config.driveModeChooser.getSelected()) {
            case ARCADE:
                driveMode = arcadeDrive;
            case CURVATURE:
                driveMode = curvatureDrive;
            case TANK:
                driveMode = tankDrive;
            case EVEN_MIXED_ARCADE_CURVATURE:
                driveMode = new MixedDrive(Map.of(arcadeDrive, 0.5, curvatureDrive, 0.5));
            case BIAS_ARCADE_MIXED_ARCADE_CURVATURE:
                driveMode = new MixedDrive(Map.of(arcadeDrive, 0.8, curvatureDrive, 0.2));
            case BIAS_CURVATURE_MIXED_ARCADE_CURVATURE:
                driveMode = new MixedDrive(Map.of(arcadeDrive, 0.2, curvatureDrive, 0.8));
            default:
                driveMode = arcadeDrive;
        }
    }

    @Override
    public void execute() {
        drivetrain.setVoltages(
                Constants.GlobalConstants.GLOBAL_NOMINAL_VOLTAGE_VOLTS * driveMode.execute().left,
                Constants.GlobalConstants.GLOBAL_NOMINAL_VOLTAGE_VOLTS * driveMode.execute().right);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setVoltages(0, 0);
    }

    @Override
    public void onOptionSelected(DriveMode oldOption, DriveMode newOption) {
        updateDriveProfile();
    }
}
