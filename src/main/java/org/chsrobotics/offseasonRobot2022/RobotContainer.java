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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.nio.charset.Charset;
import java.util.HashMap;
import java.util.List;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.util.DashboardChooser;
import org.chsrobotics.offseasonRobot2022.Constants.CommandConstants.LauncherControllerConstants;
import org.chsrobotics.offseasonRobot2022.Constants.InputConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.DrivetrainConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.LauncherConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.VisionConstants;
import org.chsrobotics.offseasonRobot2022.aprilTags.AprilTag;
import org.chsrobotics.offseasonRobot2022.aprilTags.TagLayout;
import org.chsrobotics.offseasonRobot2022.commands.drive.BasicDrive;
import org.chsrobotics.offseasonRobot2022.commands.drive.TrajectoryFollower;
import org.chsrobotics.offseasonRobot2022.commands.launcher.LauncherController;
import org.chsrobotics.offseasonRobot2022.commands.util.ConditionalEndRunnableCommand;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;
import org.chsrobotics.offseasonRobot2022.subsystems.Launcher;
import org.chsrobotics.offseasonRobot2022.subsystems.Vision;

public class RobotContainer {
    private final Joystick driveJoystick = new Joystick(InputConstants.JOYSTICK_PORT);

    private final Drivetrain drivetrain = new Drivetrain();

    private final Vision vision = new Vision(VisionConstants.CAMERA_1_NAME);

    private final Launcher launcher = new Launcher();

    private final TagLayout testLayout =
            new TagLayout(new AprilTag(0, new Pose3d(5, 5, 0, new Rotation3d())));

    private final Localizer localizer = new Localizer(drivetrain, vision, new Pose2d(), testLayout);

    private long cycleCounter = 0;

    private final HashMap<Command, Timer> commandTimers = new HashMap<>();

    private final BasicDrive basicDrive =
            new BasicDrive(
                    drivetrain,
                    // () -> driveJoystick.getRawAxis(InputConstants.JOYSTICK_LEFT_VERTICAL_AXIS),
                    // () ->
                    // driveJoystick.getRawAxis(InputConstants.JOYSTICK_RIGHT_HORIZONTAL_AXIS));
                    () -> -driveJoystick.getRawAxis(1),
                    () -> -driveJoystick.getRawAxis(0));

    private final LauncherController launcherStandby =
            new LauncherController(
                    launcher,
                    LauncherControllerConstants.FLYWHEEL_STANDBY_VELOCITY_RADS_PER_SECOND,
                    LauncherControllerConstants.HOOD_DEFAULT_ANGLE_RADIANS);

    private final ConditionalEndRunnableCommand calibrateIMU =
            new ConditionalEndRunnableCommand(
                    drivetrain, drivetrain::startCalibration, drivetrain::isCalibrated);

    private final DashboardChooser<Paths> autoModeChooser =
            DashboardChooser.fromEnum(Paths.class, Paths.noneTraj, true);

    public RobotContainer() {
        calibrateIMU.setName("CalibrateIMU");
        calibrateIMU.setRunsWhileDisabled(true);

        configureButtonBindings();

        setDefaultCommands();

        SmartDashboard.putData(autoModeChooser);

        drivetrain.setNeutralmode(DrivetrainConstants.DEFAULT_NEUTRAL_MODE);

        CommandScheduler.getInstance().onCommandInitialize(this::logCommandInit);
        CommandScheduler.getInstance().onCommandFinish(this::logCommandFinished);
        CommandScheduler.getInstance().onCommandInterrupt(this::logCommandInterrupted);

        HashMap<Double, List<Double>> launcherMap = new HashMap<>();

        try {
            CSVParser launcherMapParser =
                    CSVParser.parse(
                            LauncherConstants.launcherMapCSV,
                            Charset.defaultCharset(),
                            CSVFormat.DEFAULT);

            for (CSVRecord line : launcherMapParser.getRecords()) {
                try {
                    launcherMap.put(
                            Double.valueOf(line.get(0)),
                            List.of(Double.valueOf(line.get(1)), Double.valueOf(line.get(2))));
                } catch (NumberFormatException exc) {
                }
            }

        } catch (IOException exc) {
            HighLevelLogger.logMessage("LAUNCHER MAP FILE COULD NOT BE READ");
            DriverStation.reportError("LAUNCHER MAP FILE COULD NOT BE READ", false);

            launcherMap.put(0.0, List.of(0.0, 0.0));
        }
    }

    private void configureButtonBindings() {}

    private void setDefaultCommands() {
        CommandScheduler.getInstance().setDefaultCommand(drivetrain, basicDrive);
        CommandScheduler.getInstance().setDefaultCommand(launcher, launcherStandby);
    }

    public Command getAutonomousCommand() {
        return new TrajectoryFollower(
                drivetrain, localizer, autoModeChooser.getSelected().trajectory, true);
    }

    /** Schedules all commands that should run when the robot is initialized. */
    public void scheduleStartupCommands() {
        HighLevelLogger.logMessage("Startup commands scheduled");
        CommandScheduler.getInstance().schedule(calibrateIMU);
    }

    /** Method called once every robot cycle to perform RobotContainer-specific periodic tasks. */
    public void periodic() {
        SmartDashboard.putData(localizer.getFullField());
        cycleCounter++;
        localizer.update();
    }

    /** Method to log any data that should only be recorded at the end of a match. */
    public void logShutdownData() {
        HighLevelLogger.logMessage("*******ROBOT SHUTDOWN*******");
        HighLevelLogger.logMessage("Total number of loop cycles: " + cycleCounter);
        HighLevelLogger.logMessage("Total energy use (joules): " + Robot.getTotalEnergyJoules());
        HighLevelLogger.logMessage(
                "Total absolute distance driven (meters): " + drivetrain.getAbsDistanceMeters());
        HighLevelLogger.logMessage(
                "Total absolute flywheel distance (radians): "
                        + launcher.getFlywheelAbsDistanceRadians());
        HighLevelLogger.logMessage(
                "Total absolute hood distance (radians): " + launcher.getHoodAbsDistanceRadians());
    }

    private void logCommandInit(Command command) {
        HighLevelLogger.logMessage("Command started: " + command.getName());

        Timer timer = new Timer();
        timer.start();

        commandTimers.put(command, timer);
    }

    private void logCommandFinished(Command command) {
        double timeSeconds = commandTimers.get(command).get();

        HighLevelLogger.logMessage(
                "Command finished: " + command.getName() + " after " + timeSeconds + " seconds");

        commandTimers.remove(command);
    }

    private void logCommandInterrupted(Command command) {
        double timeSeconds = commandTimers.get(command).get();

        HighLevelLogger.logMessage(
                "Command interrupted: " + command.getName() + " after " + timeSeconds + " seconds");

        commandTimers.remove(command);
    }
}
