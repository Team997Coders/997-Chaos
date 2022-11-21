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

/** */
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
                    VisionConstants.CAMERA_NAME,
                    VisionConstants.DIAGONAL_FOV_DEGREES,
                    VisionConstants.CAMERA_PITCH_DEGREES,
                    VisionConstants.TRANSFORMATION_CAMERA_TO_ROBOT,
                    VisionConstants.CAMERA_HEIGHT_METERS,
                    9001,
                    VisionConstants.CAMERA_RESOLUTION_HORIZONTAL,
                    VisionConstants.CAMERA_RESOLUTION_VERTICAL,
                    VisionConstants.CONTOUR_THRESHOLD);

    /**
     * @param drive
     * @param vision
     * @param initialPose
     * @param layout
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
                    .getObject(((Integer) tag.hashCode()).toString())
                    .setPose(tag.getPlacement().toPose2d());
            simVision.addSimVisionTarget(
                    new SimVisionTarget(
                            tag.getPlacement().toPose2d(), tag.getPlacement().getZ(), 0.1, 0.3));
        }
    }

    /** */
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
                                VisionConstants.TRANSFORMATION_CAMERA_TO_ROBOT));
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
            // TODO: actually get best pose
            Pose2d bestVisionPose = visionPoses.get(0);

            poseEstimator.addVisionMeasurement(
                    bestVisionPose,
                    Timer.getFPGATimestamp()
                            - Units.millisecondsToSeconds(vision.getLatencyMillis()));
        }

        fullField.getObject("pureOdometry").setPose(getOdometryPose());
        fullField.setRobotPose(getFusedPose());

        cleanField.setRobotPose(getFusedPose());

        if (!Robot.isReal()) {
            simVision.processFrame(getFusedPose());
        }
    }

    /**
     * @param poseMeters
     */
    public void setPose(Pose2d poseMeters) {
        drive.resetEncoders();
        poseEstimator.resetPosition(poseMeters, drive.getRotationGyro());
    }

    /**
     * @return
     */
    public Pose2d getFusedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * @return
     */
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /**
     * @return
     */
    public Field2d getFullField() {
        return fullField;
    }

    /**
     * @return
     */
    public Field2d getCleanField() {
        return cleanField;
    }
}
