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
package org.chsrobotics.offseasonRobot2022.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** */
public class Vision {
    private final PhotonCamera camera =
            new PhotonCamera(NetworkTableInstance.getDefault(), VisionConstants.CAMERA_NAME);

    /** */
    public Vision() {}

    /**
     * @return
     */
    public List<PhotonTrackedTarget> getAllTags() {
        return camera.getLatestResult().getTargets();
    }

    /**
     * @return
     */
    public int getLatencyMillis() {
        return (int) camera.getLatestResult().getLatencyMillis();
    }
}
