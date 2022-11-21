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
package org.chsrobotics.offseasonRobot2022;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.offseasonRobot2022.Constants.InputConstants;
import org.chsrobotics.offseasonRobot2022.aprilTags.AprilTag;
import org.chsrobotics.offseasonRobot2022.aprilTags.TagLayout;
import org.chsrobotics.offseasonRobot2022.commands.teleop.BasicDrive;
import org.chsrobotics.offseasonRobot2022.commands.util.CalibrateIMU;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;
import org.chsrobotics.offseasonRobot2022.subsystems.Vision;

public class RobotContainer {
    private final Joystick driveJoystick = new Joystick(InputConstants.JOYSTICK_PORT);

    private final Drivetrain drivetrain = new Drivetrain();

    private final Vision vision = new Vision();

    private final TagLayout testLayout =
            new TagLayout(new AprilTag(0, new Pose3d(5, 5, 0, new Rotation3d())));

    private final BasicDrive basicDrive =
            new BasicDrive(
                    drivetrain,
                    () -> driveJoystick.getRawAxis(InputConstants.JOYSTICK_LEFT_VERTICAL_AXIS),
                    () -> driveJoystick.getRawAxis(InputConstants.JOYSTICK_RIGHT_HORIZONTAL_AXIS));

    private final CalibrateIMU calibrateIMU = new CalibrateIMU(drivetrain);

    private final Localizer localizer = new Localizer(drivetrain, vision, new Pose2d(), testLayout);

    public RobotContainer() {
        configureButtonBindings();

        setDefaultCommands();
    }

    private void configureButtonBindings() {}

    private void setDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(drivetrain, basicDrive);
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

    /** Schedules all commands that should run when the robot is initialized. */
    public void scheduleStartupCommands() {
        CommandScheduler.getInstance().schedule(calibrateIMU);
        HighLevelLogger.logMessage("Startup commands scheduled");
    }

    /** Method called once every robot cycle to perform RobotContainer-specific periodic tasks. */
    public void periodic() {
        SmartDashboard.putData(localizer.getFullField());
        localizer.update();
    }

    /** Method to log any data that should only be recorded at the end of a match. */
    public void logShutdownData() {
        HighLevelLogger.logMessage("*******ROBOT SHUTDOWN*******");
        HighLevelLogger.logMessage("Total energy use (joules): " + Robot.getTotalEnergyJoules());
        HighLevelLogger.logMessage(
                "Total absolute distance driven (meters): " + drivetrain.getAbsDistanceMeters());
    }
}
