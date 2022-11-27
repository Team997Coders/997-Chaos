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

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/** */
public class ConditionalEndRunnableCommand extends CommandBase {
    private final Supplier<Boolean> endConditional;

    private final Runnable init;
    private final Runnable end;

    private boolean runsWhileDisabled = false;

    /**
     * @param init
     * @param end
     * @param endConditional
     */
    public ConditionalEndRunnableCommand(
            Runnable init, Runnable end, Supplier<Boolean> endConditional) {
        this.endConditional = endConditional;

        this.init = init;
        this.end = end;
    }

    /**
     * @param init
     * @param endConditional
     */
    public ConditionalEndRunnableCommand(Runnable init, Supplier<Boolean> endConditional) {
        this(init, null, endConditional);
    }

    /**
     * @param toRequire
     * @param init
     * @param end
     * @param endConditional
     */
    public ConditionalEndRunnableCommand(
            Subsystem toRequire, Runnable init, Runnable end, Supplier<Boolean> endConditional) {
        this(init, end, endConditional);
        addRequirements(toRequire);
    }

    /**
     * @param toRequire
     * @param init
     * @param endConditional
     */
    public ConditionalEndRunnableCommand(
            Subsystem toRequire, Runnable init, Supplier<Boolean> endConditional) {
        this(init, init, endConditional);
        addRequirements(toRequire);
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
        if (init != null) init.run();
    }

    @Override
    public void end(boolean interrupted) {
        if (end != null) end.run();
    }

    @Override
    public boolean isFinished() {
        return endConditional.get();
    }
}
