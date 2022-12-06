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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.chsrobotics.lib.telemetry.HighLevelLogger;

public class Robot extends TimedRobot {

    public enum State {
        DISABLED_FMS,
        DISABLED_NOFMS,
        TELEOP_FMS,
        TELEOP_NOFMS,
        AUTO_FMS,
        AUTO_NOFMS,
        TEST_NOFMS,
        ESTOPPED_FMS,
        ESTOPPED_NOFMS
    }

    private Command autonomousCommand;

    private static final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

    private final RobotContainer robotContainer = new RobotContainer();

    private State previousRobotState = null;

    @Override
    public void robotInit() {
        HighLevelLogger.startLogging();
        HighLevelLogger.logMessage("*******ROBOT STARTUP*******");
        HighLevelLogger.logMessage("997 Offseason 2022: CHAOS");
        robotContainer.scheduleStartupCommands();

        DriverStation.startDataLog(HighLevelLogger.getLog(), true);

        SmartDashboard.putData("autoModeSelector", Config.autoModeChooser);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        robotContainer.periodic();

        previousRobotState = getRobotState();

        // HighLevelLogger.logPeriodic();
    }

    @Override
    public void disabledInit() {
        if (previousRobotState == State.TELEOP_FMS || previousRobotState == State.TELEOP_NOFMS) {
            robotContainer.logShutdownData();
        }
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}

    /**
     * Returns the current state (disabled, auto, teleop, etc...) of the robot through the driver's
     * station.
     *
     * @return The current state of the robot.
     */
    public static State getRobotState() {
        if (DriverStation.isFMSAttached()) {
            if (DriverStation.isTeleop()) return State.TELEOP_FMS;
            else if (DriverStation.isAutonomous()) return State.AUTO_FMS;
            else if (DriverStation.isEStopped()) return State.ESTOPPED_FMS;
            else return State.DISABLED_FMS;
        } else {
            if (DriverStation.isTeleop()) return State.TELEOP_NOFMS;
            else if (DriverStation.isAutonomous()) return State.AUTO_NOFMS;
            else if (DriverStation.isEStopped()) return State.ESTOPPED_NOFMS;
            else if (DriverStation.isTest()) return State.TEST_NOFMS;
            else return State.DISABLED_NOFMS;
        }
    }

    /**
     * Returns the current output by a channel of the PDP.
     *
     * @param channel The channel (in [0,15], inclusive) to query.
     * @return The current in amperes of that channel.
     */
    public static double getCurrentAmps(int channel) {
        return pdp.getCurrent(channel);
    }

    /**
     * Returns the total energy output by the Power Distribution Panel.
     *
     * @return The total energy since last robot restart, in joules.
     */
    public static double getTotalEnergyJoules() {
        return pdp.getTotalEnergy();
    }
}
