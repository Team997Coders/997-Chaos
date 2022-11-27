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
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/** */
public class ConditionalRunnableCommand extends CommandBase {
    private final Timer timer = new Timer();

    private final Supplier<Boolean> conditional;

    private final Runnable onTrue;

    private final Runnable onFalse;

    private final double durationSeconds;

    private boolean lastConditionalValue;

    private boolean runsWhileDisabled = false;

    /**
     * @param onTrue
     * @param onFalse
     * @param conditional
     * @param durationSeconds
     */
    public ConditionalRunnableCommand(
            Runnable onTrue,
            Runnable onFalse,
            Supplier<Boolean> conditional,
            double durationSeconds) {
        this.conditional = conditional;

        this.onTrue = onTrue;
        this.onFalse = onFalse;

        this.durationSeconds = durationSeconds;

        lastConditionalValue = conditional.get();
    }

    /**
     * @param onTrue
     * @param onFalse
     * @param conditional
     */
    public ConditionalRunnableCommand(
            Runnable onTrue, Runnable onFalse, Supplier<Boolean> conditional) {
        this(onTrue, onFalse, conditional, -1);
    }

    /**
     * @param toRequire
     * @param onTrue
     * @param onFRunnable
     * @param conditional
     * @param durationSeconds
     */
    public ConditionalRunnableCommand(
            Subsystem toRequire,
            Runnable onTrue,
            Runnable onFalse,
            Supplier<Boolean> conditional,
            double durationSeconds) {
        this(onTrue, onFalse, conditional, durationSeconds);
        addRequirements(toRequire);
    }

    /**
     * @param toRequire
     * @param onTrue
     * @param onFalse
     * @param conditional
     */
    public ConditionalRunnableCommand(
            Subsystem toRequire, Runnable onTrue, Runnable onFalse, Supplier<Boolean> conditional) {
        this(onTrue, onFalse, conditional);
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
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        boolean value = conditional.get();

        if (!lastConditionalValue && value && onTrue != null) {
            onTrue.run();
        } else if (lastConditionalValue && !value && onFalse != null) {
            onFalse.run();
        }
        lastConditionalValue = value;
    }

    @Override
    public boolean isFinished() {
        return (durationSeconds >= 0 && timer.get() >= durationSeconds);
    }
}
