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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.chsrobotics.lib.telemetry.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** */
public class Vision extends SubsystemBase {
    private final PhotonCamera camera;

    private final String subdirString = "vision";

    private final Logger<Double[]> targetsCornersXLogger =
            new Logger<>("targetsCornersX", subdirString);
    private final Logger<Double[]> targetsCornersYLogger =
            new Logger<>("targetsCornersY", subdirString);

    private final Logger<Double> latencySecondsLogger =
            new Logger<>("latencySeconds", subdirString);

    private final Logger<Integer[]> tagIDsInViewLogger = new Logger<>("tagIDsInView", subdirString);

    private final Logger<Boolean> hasTargetsLogger = new Logger<>("hasTargets", subdirString);

    // private final Logger<Integer> currentPipelineLogger =
    //        new Logger<>("currentPipelineIndex", subdirString);

    /**
     * Constructs a Vision subsystem.
     *
     * @param cameraName The configured name of the camera to use in PhotonVision.
     */
    public Vision(String cameraName) {
        camera = new PhotonCamera(NetworkTableInstance.getDefault(), cameraName);
    }

    /**
     * Returns all recognized targets in the camera's field of view.
     *
     * @return A list of PhotonTrackedTargets, which contain information about AprilTag ID and pose.
     */
    public List<PhotonTrackedTarget> getAllTags() {
        return camera.getLatestResult().getTargets();
    }

    /**
     * Returns the latency of the vision pipline (i.e. how long it takes to process a frame from the
     * camera).
     *
     * @return The latency of the current pipeline in seconds.
     */
    public double getLatencySeconds() {
        return Units.millisecondsToSeconds(camera.getLatestResult().getLatencyMillis());
    }

    /**
     * Returns whether there is a recognized target in the camera's field of view.
     *
     * @return If targets are seen.
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Changes the pipeline used for vision processing.
     *
     * @param index The index, reported by PhotonVision, of the new pipeline.
     */
    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    /**
     * Returns the pipeline used for vision processing.
     *
     * @return The index, reported by PhotonVision, of the current pipeline.
     */
    public int getCurrentPipeline() {
        return camera.getPipelineIndex();
    }

    @Override
    public void periodic() {
        ArrayList<Double> xCorners = new ArrayList<>();
        ArrayList<Double> yCorners = new ArrayList<>();

        ArrayList<Integer> visibleIDs = new ArrayList<>();

        for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
            visibleIDs.add(target.getFiducialId());
            for (TargetCorner corner : target.getCorners()) {
                xCorners.add(corner.x);
                yCorners.add(corner.y);
            }
        }

        targetsCornersXLogger.update(xCorners.toArray(new Double[] {}));
        targetsCornersYLogger.update(xCorners.toArray(new Double[] {}));

        tagIDsInViewLogger.update(visibleIDs.toArray(new Integer[] {}));

        latencySecondsLogger.update(getLatencySeconds());

        //    currentPipelineLogger.update(getCurrentPipeline());

        hasTargetsLogger.update(hasTargets());
    }
}
