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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import org.chsrobotics.offseasonRobot2022.Constants.LocalizationConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.DrivetrainConstants;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.VisionConstants;
import org.chsrobotics.offseasonRobot2022.aprilTags.AprilTag;
import org.chsrobotics.offseasonRobot2022.aprilTags.TagLayout;
import org.chsrobotics.offseasonRobot2022.subsystems.Drivetrain;
import org.chsrobotics.offseasonRobot2022.subsystems.Vision;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The Localizer is used for fusing pose estimates from multiple sources into a more-accurate,
 * global, field-relative pose estimate.
 */
public class Localizer {
    private final Drivetrain drive;
    private final Vision vision;

    private final DifferentialDrivePoseEstimator poseEstimator;
    private final DifferentialDriveOdometry odometry;

    private final TagLayout layout;

    private final Field2d cleanField = new Field2d();

    private final Field2d fullField = new Field2d();

    private final SimVisionSystem simVision =
            new SimVisionSystem(
                    VisionConstants.CAMERA_1_NAME,
                    VisionConstants.DIAGONAL_FOV_DEGREES,
                    VisionConstants.CAMERA_PITCH_DEGREES,
                    VisionConstants.TRANSFORMATION_ROBOT_TO_CAMERA.inverse(),
                    VisionConstants.CAMERA_HEIGHT_METERS,
                    9001,
                    VisionConstants.CAMERA_RESOLUTION_HORIZONTAL,
                    VisionConstants.CAMERA_RESOLUTION_VERTICAL,
                    VisionConstants.CONTOUR_THRESHOLD);

    /**
     * Constructs a Localizer.
     *
     * @param drive The Drivetrain subsystem to use for odometry.
     * @param vision The Vision subsystem to use for vision-derived pose estimates.
     * @param initialPose The initial pose of the robot on the field.
     * @param layout The layout containing the AprilTags of the field environment.
     */
    public Localizer(Drivetrain drive, Vision vision, Pose2d initialPose, TagLayout layout) {
        poseEstimator =
                new DifferentialDrivePoseEstimator(
                        drive.getRotationGyro(),
                        initialPose,
                        VecBuilder.fill(
                                LocalizationConstants.ODOMETRY_ERROR_METERS_STDDEV,
                                LocalizationConstants.ODOMETRY_ERROR_METERS_STDDEV,
                                LocalizationConstants.MODEL_ERROR_THETA_RADIANS_STDDEV,
                                LocalizationConstants.MODEL_ERROR_DIST_METERS_STDDEV,
                                LocalizationConstants.MODEL_ERROR_DIST_METERS_STDDEV),
                        VecBuilder.fill(
                                DrivetrainConstants.ENCODER_ERROR_DISPLACEMENT_METERS_STDDEV,
                                DrivetrainConstants.ENCODER_ERROR_DISPLACEMENT_METERS_STDDEV,
                                DrivetrainConstants.GYRO_ERROR_ANGLE_RADIANS_STDDEV),
                        VecBuilder.fill(
                                VisionConstants.DEFAULT_ERROR_DISPLACEMENT_METERS_STDDEV,
                                VisionConstants.DEFAULT_ERROR_DISPLACEMENT_METERS_STDDEV,
                                VisionConstants.DEFAULT_ERROR_ANGLE_RADIANS_STDDEV),
                        Robot.kDefaultPeriod);

        odometry = new DifferentialDriveOdometry(drive.getRotationGyro(), initialPose);
        this.drive = drive;
        this.vision = vision;
        this.layout = layout;

        for (AprilTag tag : layout.getAllTags()) {
            fullField
                    .getObject("AprilTag: " + tag.getID())
                    // TODO: currently does not support multiple tags with the same ID
                    .setPose(tag.getPlacement().toPose2d());
            simVision.addSimVisionTarget(
                    new SimVisionTarget(
                            tag.getPlacement().toPose2d(), tag.getPlacement().getZ(), 0.1, 0.3));
        }
    }

    /**
     * Advances the pose estimator by a time increment. Note that this should be called exactly once
     * per loop.
     */
    public void update() {
        ArrayList<Pose2d> visionPoses = new ArrayList<>();
        for (PhotonTrackedTarget target : vision.getAllTags()) {
            if (target.getPoseAmbiguity() < 0.2) {
                AprilTag nearestTag = layout.getTagsAtID(target.getFiducialId()).get(0);
                Transform3d cameraToTarget3d = target.getBestCameraToTarget();
                Transform2d cameraToTarget =
                        new Transform2d(
                                new Translation2d(cameraToTarget3d.getX(), cameraToTarget3d.getY()),
                                cameraToTarget3d.getRotation().toRotation2d());
                // TODO: actually get the nearest tag at that ID
                visionPoses.add(
                        PhotonUtils.estimateFieldToRobot(
                                cameraToTarget,
                                nearestTag.getPlacement().toPose2d(),
                                VisionConstants.TRANSFORMATION_ROBOT_TO_CAMERA.inverse()));
            }
        }

        poseEstimator.update(
                drive.getRotationGyro(),
                new DifferentialDriveWheelSpeeds(
                        drive.getLeftSideVelocityMetersPerSecond(),
                        drive.getRightSideVelocityMetersPerSecond()),
                drive.getLeftSideDistanceMeters(),
                drive.getRightSideDistanceMeters());

        odometry.update(
                drive.getRotationGyro(),
                drive.getLeftSideDistanceMeters(),
                drive.getRightSideDistanceMeters());

        if (!visionPoses.isEmpty()) {
            for (Pose2d visionPose : visionPoses) {
                fullField.getObject("" + visionPose.hashCode()).setPose(visionPose);
            }

            // TODO: actually get best pose
            Pose2d bestVisionPose = visionPoses.get(0);

            poseEstimator.addVisionMeasurement(
                    bestVisionPose, Timer.getFPGATimestamp() - vision.getLatencySeconds());
        }

        Pose2d cameraPose =
                getFusedPose().transformBy(VisionConstants.TRANSFORMATION_ROBOT_TO_CAMERA);
        double partialCamFOV = Units.degreesToRadians(VisionConstants.HORIZONTAL_FOV_DEGREES / 2);
        double camFOVDisplayMagnitudeMeters = 1;
        fullField.getObject("pureOdometry").setPose(getOdometryPose());
        fullField.getObject("cam1").setPose(cameraPose);
        fullField
                .getObject("cam1RightLimit")
                .setPose(
                        cameraPose.transformBy(
                                new Transform2d(
                                        new Translation2d(
                                                camFOVDisplayMagnitudeMeters / 2,
                                                new Rotation2d(-partialCamFOV)),
                                        new Rotation2d(-partialCamFOV))));
        fullField
                .getObject("cam1LeftLimit")
                .setPose(
                        cameraPose.transformBy(
                                new Transform2d(
                                        new Translation2d(
                                                camFOVDisplayMagnitudeMeters / 2,
                                                new Rotation2d(partialCamFOV)),
                                        new Rotation2d(partialCamFOV))));
        fullField.setRobotPose(getFusedPose());

        cleanField.setRobotPose(getFusedPose());

        if (!Robot.isReal()) {
            simVision.processFrame(getFusedPose());
        }
    }

    /**
     * Sets the pose of the robot on the field.
     *
     * @param poseMeters The new pose, in meters, of the robot.
     */
    public void setPose(Pose2d poseMeters) {
        drive.resetEncoders();
        poseEstimator.resetPosition(poseMeters, drive.getRotationGyro());
        odometry.resetPosition(poseMeters, drive.getRotationGyro());
    }

    /**
     * Returns the current fused pose estimate.
     *
     * @return The estimated pose, in meters, of the robot on the field.
     */
    public Pose2d getFusedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the pose estimate from only odometry.
     *
     * @return The odometry-derived pose, in meters, of the robot on the field.
     */
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns a Field2d populated with both a representation of the robot pose, and additional
     * localization information indicators.
     *
     * @return A Field2d with fused pose, odometry pose, camera FOV indicators, and AprilTags.
     */
    public Field2d getFullField() {
        return fullField;
    }

    /**
     * Returns a Field2d solely populated with a representation of the robot pose.
     *
     * @return A Field2d with only the fused pose estimate.
     */
    public Field2d getCleanField() {
        return cleanField;
    }
}
