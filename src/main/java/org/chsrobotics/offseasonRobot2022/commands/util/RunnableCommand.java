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
package org.chsrobotics.offseasonRobot2022.commands.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** */
public class RunnableCommand extends CommandBase {
    private final Timer timer = new Timer();

    private final Runnable init;
    private final Runnable end;

    private final double durationSeconds;

    private boolean runsWhileDisabled = false;

    /**
     * @param init
     * @param end
     * @param durationSeconds
     */
    public RunnableCommand(Runnable init, Runnable end, double durationSeconds) {
        this.init = init;
        this.end = end;

        this.durationSeconds = durationSeconds;
    }

    /**
     * @param init
     */
    public RunnableCommand(Runnable init) {
        this(init, null, 0);
    }

    /**
     * @param runsWhileDisabled
     */
    public void setRunsWhileDisabled(boolean runsWhileDisabled) {
        this.runsWhileDisabled = runsWhileDisabled;
    }

    @Override
    public boolean runsWhenDisabled() {
        return runsWhileDisabled;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (init != null) init.run();
    }

    @Override
    public void end(boolean interrupted) {
        if (end != null) end.run();
    }

    @Override
    public boolean isFinished() {
        return (durationSeconds >= 0 && timer.get() >= durationSeconds);
    }
}
