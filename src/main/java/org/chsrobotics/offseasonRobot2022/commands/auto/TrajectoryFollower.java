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
package org.chsrobotics.offseasonRobot2022.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.ArrayList;
import java.util.List;
import org.chsrobotics.offseasonRobot2022.Constants.CommandConstants.TrajectoryFollowerConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.DrivetrainConstants;
import org.chsrobotics.offseasonRobot2022.Localizer;
import org.chsrobotics.offseasonRobot2022.Robot;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;

/**
 * A command which guides the drivetrain of the robot through a pre-generated spline-defined
 * trajectory.s
 */
public class TrajectoryFollower extends CommandBase {
    private final Drivetrain drive;
    private final Localizer localization;
    private final PathPlannerTrajectory trajectory;

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    DrivetrainConstants.MODEL_LIN_KS,
                    DrivetrainConstants.MODEL_LIN_KV,
                    DrivetrainConstants.MODEL_LIN_KA);

    private final PIDController rightSideVelocityController =
            new PIDController(
                    DrivetrainConstants.WHEEL_VELOCITY_KP,
                    DrivetrainConstants.WHEEL_VELOCITY_KI,
                    DrivetrainConstants.WHEEL_VELOCITY_KD,
                    Robot.kDefaultPeriod);

    private final PIDController leftSideVelocityController =
            new PIDController(
                    DrivetrainConstants.WHEEL_VELOCITY_KP,
                    DrivetrainConstants.WHEEL_VELOCITY_KI,
                    DrivetrainConstants.WHEEL_VELOCITY_KD,
                    Robot.kDefaultPeriod);

    private final RamseteController controller;

    private final Timer timer = new Timer();

    private final DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(DrivetrainConstants.TRACKWIDTH_EFFECTIVE_M);

    /**
     * Constructs a TrajectoryFollower command.
     *
     * @param drive The Drivetrain subsystem to use.
     * @param localization The Localizer to use for pose estimates.
     * @param trajectory The pre-generated PathPlanner trajectory to follow.
     * @param setPoseToInitial Whether the global pose of the robot should be set to the initial
     *     pose of the trajectory.
     */
    public TrajectoryFollower(
            Drivetrain drive,
            Localizer localization,
            PathPlannerTrajectory trajectory,
            boolean setPoseToInitial) {
        addRequirements(drive);
        this.drive = drive;
        this.localization = localization;
        this.trajectory = trajectory;

        if (trajectory != null && setPoseToInitial) {
            this.localization.setPose(trajectory.getInitialPose());
        }

        controller =
                new RamseteController(
                        TrajectoryFollowerConstants.RAMSETE_CONVERGENCE,
                        TrajectoryFollowerConstants.RAMSETE_DAMPING);
    }

    /**
     * Returns a list of all EventMarkers occuring at the current time since the trajectory started.
     *
     * @return
     */
    public List<EventMarker> getCurrentEvents() {
        double epsilon = 0.01;

        List<EventMarker> currentEvents = new ArrayList<>();

        if (trajectory != null) {
            for (EventMarker marker : trajectory.getMarkers()) {
                if (Math.abs(timer.get() - marker.timeSeconds) <= epsilon)
                    currentEvents.add(marker);
            }
        }

        return currentEvents;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (trajectory != null) {
            ChassisSpeeds frameRates =
                    controller.calculate(
                            localization.getFusedPose(), trajectory.sample(timer.get()));

            DifferentialDriveWheelSpeeds wheelVelocities = kinematics.toWheelSpeeds(frameRates);

            SmartDashboard.putNumber("right setpoint", wheelVelocities.rightMetersPerSecond);
            SmartDashboard.putNumber("left setpoint", wheelVelocities.leftMetersPerSecond);

            SmartDashboard.putNumber("actual right", drive.getRightSideVelocityMetersPerSecond());
            SmartDashboard.putNumber("actual left", drive.getLeftSideVelocityMetersPerSecond());

            drive.setVoltages(
                    wheelVelocities.leftMetersPerSecond, wheelVelocities.rightMetersPerSecond);

            leftSideVelocityController.setSetpoint(wheelVelocities.leftMetersPerSecond);

            double leftVoltage =
                    leftSideVelocityController.calculate(drive.getLeftSideVelocityMetersPerSecond())
                            + feedforward.calculate(
                                    drive.getLeftSideVelocityMetersPerSecond(),
                                    wheelVelocities.leftMetersPerSecond,
                                    Robot.kDefaultPeriod);

            rightSideVelocityController.setSetpoint(wheelVelocities.rightMetersPerSecond);

            double rightVoltage =
                    rightSideVelocityController.calculate(
                                    drive.getRightSideVelocityMetersPerSecond())
                            + feedforward.calculate(
                                    drive.getRightSideVelocityMetersPerSecond(),
                                    wheelVelocities.rightMetersPerSecond,
                                    Robot.kDefaultPeriod);

            drive.setVoltages(leftVoltage, rightVoltage);
        }
    }

    @Override
    public boolean isFinished() {
        if (trajectory != null) return (timer.get() >= trajectory.getTotalTimeSeconds());
        else return true;
    }
}
