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

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import org.chsrobotics.lib.telemetry.DashboardChooser;
import org.chsrobotics.lib.telemetry.DashboardChooser.Option;

public class Config {
    public static enum DriveLinLimiting implements Option {
        NONE(0, "None"),
        PARTIAL(10, "Partial"),
        FULL(5, "Full");

        public final double limit;

        private final String displayName;

        private DriveLinLimiting(double limit, String displayName) {
            this.limit = limit;
            this.displayName = displayName;
        }

        @Override
        public String getDisplayName() {
            return displayName;
        }
    }

    public static final DashboardChooser<DriveLinLimiting> driveLinLimitingChooser =
            DashboardChooser.fromEnum(DriveLinLimiting.class, DriveLinLimiting.NONE);

    public static enum DriveRotLimiting implements Option {
        NONE(0, "None"),
        PARTIAL(10, "Partial"),
        FULL(5, "Full");

        public final double limit;
        private final String displayName;

        private DriveRotLimiting(double limit, String displayName) {
            this.limit = limit;
            this.displayName = displayName;
        }

        @Override
        public String getDisplayName() {
            return displayName;
        }
    }

    public static final DashboardChooser<DriveRotLimiting> driveRotLimitingChooser =
            DashboardChooser.fromEnum(DriveRotLimiting.class, DriveRotLimiting.NONE);

    public static enum DriveLinModifier implements Option {
        FULL(1, "Full"),
        HALF(0.5, "Half"),
        DEMO(0.25, "Demo");

        public final double modifier;
        private final String displayName;

        private DriveLinModifier(double modifier, String displayName) {
            this.modifier = modifier;
            this.displayName = displayName;
        }

        @Override
        public String getDisplayName() {
            return displayName;
        }
    }

    public static final DashboardChooser<DriveLinModifier> driveLinModifierChooser =
            DashboardChooser.fromEnum(DriveLinModifier.class, DriveLinModifier.FULL);

    public static enum DriveRotModifier implements Option {
        FULL(1),
        HALF(0.5),
        DEMO(0.25);

        public final double modifier;

        private DriveRotModifier(double modifier) {
            this.modifier = modifier;
        }
    }

    public static final DashboardChooser<DriveRotModifier> driveRotModifierChooser =
            DashboardChooser.fromEnum(DriveRotModifier.class, DriveRotModifier.FULL);

    public static enum DriveMode implements Option {
        ARCADE("Arcade"),
        CURVATURE("Curvature"),
        TANK("Tank"),
        EVEN_MIXED_ARCADE_CURVATURE("Even Mixed Arcade Curvature"),
        BIAS_ARCADE_MIXED_ARCADE_CURVATURE("Bias Arcade Mixed Arcade Curvature"),
        BIAS_CURVATURE_MIXED_ARCADE_CURVATURE("Bias Curvature Mixed Arcade Curvature");

        private final String displayName;

        private DriveMode(String displayName) {
            this.displayName = displayName;
        }

        @Override
        public String getDisplayName() {
            return displayName;
        }
    }

    public static final DashboardChooser<DriveMode> driveModeChooser =
            DashboardChooser.fromEnum(DriveMode.class, DriveMode.ARCADE);

    public static enum Paths implements Option {
        NONE("None"),

        TEST("testTraj", "Test");

        public final PathPlannerTrajectory trajectory;

        private final String displayName;

        private Paths(String trajectoryName, String displayName) {
            this.trajectory =
                    PathPlanner.loadPath(
                            trajectoryName, PathPlanner.getConstraintsFromPath(trajectoryName));
            this.displayName = displayName;
        }

        private Paths(String displayName) {
            this.trajectory = null;
            this.displayName = displayName;
        }

        @Override
        public String getDisplayName() {
            return displayName;
        }
    }

    public static final DashboardChooser<Paths> autoModeChooser =
            DashboardChooser.fromEnum(Paths.class, Paths.NONE);
}
