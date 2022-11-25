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
import org.chsrobotics.lib.util.DashboardChooser.Option;

/**
 * Enum holding autonomous trajectories the robot can follow in a format such that a
 * DashboardChooser can easily be created of them.
 */
public enum Paths implements Option {
    noneTraj("None"),

    testTraj("testTraj", "Test");

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
