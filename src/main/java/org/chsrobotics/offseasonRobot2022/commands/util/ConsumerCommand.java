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
import java.security.InvalidParameterException;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/** */
public class ConsumerCommand<T> extends CommandBase {
    private final boolean isBiConsumer;

    private final Consumer<T> consumer;
    private final BiConsumer<T, T> biConsumer;

    private final double durationSeconds;
    private final Timer timer = new Timer();

    private final T init;
    private final T end;

    private final List<T> biInit;
    private final List<T> biEnd;

    private boolean runsWhileDisabled = false;

    /**
     * @param consumer
     * @param init
     * @param end
     * @param durationSeconds
     */
    public ConsumerCommand(Consumer<T> consumer, T init, T end, double durationSeconds) {
        isBiConsumer = false;

        this.consumer = consumer;
        this.biConsumer = null;

        this.init = init;
        this.end = end;

        this.biInit = null;
        this.biEnd = null;

        this.durationSeconds = durationSeconds;
    }

    /**
     * @param consumer
     * @param init
     */
    public ConsumerCommand(Consumer<T> consumer, T init) {
        this(consumer, init, null, 0);
    }

    /**
     * @param toRequire
     * @param consumer
     * @param init
     * @param end
     * @param durationSeconds
     */
    public ConsumerCommand(
            Subsystem toRequire, Consumer<T> consumer, T init, T end, double durationSeconds) {
        this(consumer, init, end, durationSeconds);
        addRequirements(toRequire);
    }

    /**
     * @param toRequire
     * @param consumer
     * @param init
     */
    public ConsumerCommand(Subsystem toRequire, Consumer<T> consumer, T init) {
        this(consumer, init);
        addRequirements(toRequire);
    }

    /**
     * @param biConsumer
     * @param init
     * @param end
     * @param durationSeconds
     * @throws InvalidParameterException
     */
    public ConsumerCommand(
            BiConsumer<T, T> biConsumer, List<T> init, List<T> end, double durationSeconds)
            throws InvalidParameterException {
        if ((init.size() != 2 && init != null) || (end.size() != 2 && end != null)) {
            throw new InvalidParameterException("Number of defined parameters must be 2!");
        }
        isBiConsumer = true;

        this.biConsumer = biConsumer;
        this.consumer = null;

        this.init = null;
        this.end = null;

        this.biInit = init;
        this.biEnd = end;

        this.durationSeconds = durationSeconds;
    }

    /**
     * @param biConsumer
     * @param init
     * @throws InvalidParameterException
     */
    public ConsumerCommand(BiConsumer<T, T> biConsumer, List<T> init)
            throws InvalidParameterException {
        this(biConsumer, init, null, 0);
    }

    /**
     * @param toRequire
     * @param biConsumer
     * @param init
     * @param end
     * @param durationSeconds
     * @throws InvalidParameterException
     */
    public ConsumerCommand(
            Subsystem toRequire,
            BiConsumer<T, T> biConsumer,
            List<T> init,
            List<T> end,
            double durationSeconds)
            throws InvalidParameterException {
        this(biConsumer, init, end, durationSeconds);
        addRequirements(toRequire);
    }

    /**
     * @param toRequire
     * @param biConsumer
     * @param init
     */
    public ConsumerCommand(Subsystem toRequire, BiConsumer<T, T> biConsumer, List<T> init) {
        this(biConsumer, init);
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

        if (isBiConsumer && biConsumer != null) {
            biConsumer.accept(biInit.get(0), biInit.get(1));
        } else if (consumer != null) {
            consumer.accept(init);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (isBiConsumer && biEnd != null && biConsumer != null) {
            biConsumer.accept(biEnd.get(0), biEnd.get(1));
        } else if (end != null && consumer != null) {
            consumer.accept(end);
        }
    }

    @Override
    public boolean isFinished() {
        return (durationSeconds >= 0 && timer.get() >= durationSeconds);
    }
}
