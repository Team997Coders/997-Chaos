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
package org.chsrobotics.offseasonRobot2022.aprilTags;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilTag {
    private final int id;
    private final Pose3d placement;

    /**
     * @param id
     * @param placement
     */
    public AprilTag(int id, Pose3d placement) {
        this.id = id;
        this.placement = placement;
    }

    /**
     * @return
     */
    public int getID() {
        return id;
    }

    /**
     * s
     *
     * @return
     */
    public Pose3d getPlacement() {
        return placement;
    }
}
