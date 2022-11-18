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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.offseasonRobot2022.Constants.SubsystemConstants.DrivetrainConstants;
import org.chsrobotics.offseasonRobot2022.Robot;

/** Drivetrain subsystem of the robot, containing drive motors and encoders. */
public class Drivetrain extends SubsystemBase {
    private final VictorSPX frontLeft;
    private final VictorSPX backLeft;

    private final VictorSPX frontRight;
    private final VictorSPX backRight;

    private double leftSideAppliedVoltage = 0;
    private double rightSideAppliedVoltage = 0;

    private final Encoder rightEncoder;
    private final Encoder leftEncoder;

    private final LinearFilter rightEncoderVelocityFilter;
    private final LinearFilter leftEncoderVelocityFilter;

    private final String subdirString = "drivetrain";

    private final Logger<Double> rightPositionLogger =
            new Logger<>("rightPositionMeters", subdirString);
    private final Logger<Double> leftPositionLogger =
            new Logger<>("leftPositionMeters", subdirString);
    private final Logger<Double> rightVelocityLogger =
            new Logger<>("rightVelocityMetersPerSecond", subdirString);
    private final Logger<Double> leftVelocityLogger =
            new Logger<>("leftVelocityMetersPerSecond", subdirString);
    private final Logger<Double> odometryXLogger = new Logger<>("odometryXMeters", subdirString);
    private final Logger<Double> odometryYLogger = new Logger<>("odometryYMeters", subdirString);

    private AHRS IMU;
    private boolean calibrationStarted = false;

    private final Logger<Double> gyroAngleLogger = new Logger<>("gyroAngleRads", subdirString);
    private final Logger<Boolean> calibrationStateLogger =
            new Logger<>("isCalibrating", subdirString);

    private SimDouble simAngleRadians;
    private final Timer simCalibrationTimer = new Timer();

    private EncoderSim rightEncoderSim;
    private EncoderSim leftEncoderSim;

    private DifferentialDrivetrainSim drivetrainSim;

    private final DifferentialDriveOdometry odometry;
    private final Field2d odometryDisplay = new Field2d();

    /** Constructs a Drivetrain. */
    public Drivetrain() {
        frontLeft = new VictorSPX(DrivetrainConstants.FRONT_LEFT_CANID);
        backLeft = new VictorSPX(DrivetrainConstants.BACK_LEFT_CANID);

        frontRight = new VictorSPX(DrivetrainConstants.FRONT_RIGHT_CANID);
        backRight = new VictorSPX(DrivetrainConstants.BACK_RIGHT_CANID);

        frontLeft.setInverted(DrivetrainConstants.LEFT_MOTORS_INVERTED);
        backLeft.setInverted(DrivetrainConstants.LEFT_MOTORS_INVERTED);

        frontRight.setInverted(DrivetrainConstants.RIGHT_MOTORS_INVERTED);
        backRight.setInverted(DrivetrainConstants.RIGHT_MOTORS_INVERTED);

        rightEncoder =
                new Encoder(
                        DrivetrainConstants.RIGHT_ENCODER_CHANNEL_A,
                        DrivetrainConstants.RIGHT_ENCODER_CHANNEL_B);
        leftEncoder =
                new Encoder(
                        DrivetrainConstants.LEFT_ENCODER_CHANNEL_A,
                        DrivetrainConstants.LEFT_ENCODER_CHANNEL_B);

        rightEncoder.setDistancePerPulse(1);
        leftEncoder.setDistancePerPulse(1);

        leftEncoder.setReverseDirection(DrivetrainConstants.LEFT_ENCODER_INVERTED);
        rightEncoder.setReverseDirection(DrivetrainConstants.RIGHT_ENCODER_INVERTED);

        rightEncoderVelocityFilter =
                LinearFilter.movingAverage(
                        DrivetrainConstants.RIGHT_ENCODER_VELOCITY_SMOOTHING_SAMPLES);
        leftEncoderVelocityFilter =
                LinearFilter.movingAverage(
                        DrivetrainConstants.LEFT_ENCODER_VELOCITY_SMOOTHING_SAMPLES);

        if (Robot.isReal()) {
            IMU = new AHRS(DrivetrainConstants.NAVX_PORT);
        } else {
            int navXSimHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            simAngleRadians =
                    new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSimHandle, "Yaw"));

            rightEncoderSim = new EncoderSim(rightEncoder);
            leftEncoderSim = new EncoderSim(leftEncoder);

            drivetrainSim =
                    new DifferentialDrivetrainSim(
                            LinearSystemId.identifyDrivetrainSystem(
                                    DrivetrainConstants.MODEL_LIN_KV,
                                    DrivetrainConstants.MODEL_LIN_KA,
                                    DrivetrainConstants.MODEL_ROT_KV,
                                    DrivetrainConstants.MODEL_ROT_KA,
                                    DrivetrainConstants.TRACKWIDTH_EFFECTIVE_M),
                            DCMotor.getMiniCIM(2),
                            (1
                                    / DrivetrainConstants.MOTOR_TO_WHEEL_HELPER
                                            .toDoubleRatioInputToOutput()),
                            DrivetrainConstants.TRACKWIDTH_EFFECTIVE_M,
                            DrivetrainConstants.WHEEL_DIAMETER_M,
                            VecBuilder.fill(
                                    DrivetrainConstants.ODOMETRY_NOISE_X_METERS_STDDEV,
                                    DrivetrainConstants.ODOMETRY_NOISE_Y_METERS_STDDEV,
                                    DrivetrainConstants.GYRO_NOISE_ANGLE_RADIANS_STDDEV,
                                    DrivetrainConstants
                                            .LEFT_ENCODER_NOISE_VELOCITY_METERSPERSECOND_STDDEV,
                                    DrivetrainConstants
                                            .RIGHT_ENCODER_NOISE_VELOCITY_METERSPERSECOND_STDDEV,
                                    DrivetrainConstants
                                            .LEFT_ENCODER_NOISE_DISPLACEMENT_METERS_STDDEV,
                                    DrivetrainConstants
                                            .RIGHT_ENCODER_NOISE_DISPLACEMENT_METERS_STDDEV));
        }
        odometry = new DifferentialDriveOdometry(getRotationGyro());
    }

    /**
     * Sets the voltages of each pair of drivetrain motors.
     *
     * @param leftSideVoltage The voltage (in volts) to apply to the front and back left motors.
     * @param rightSideVoltage The voltage (in volts) to apply to the front and back right motors.
     */
    public void setVoltages(double leftSideVoltage, double rightSideVoltage) {
        SmartDashboard.putNumber("left side set voltage", leftSideVoltage);
        SmartDashboard.putNumber("right side set voltage", rightSideVoltage);

        leftSideAppliedVoltage = leftSideVoltage;
        rightSideAppliedVoltage = rightSideVoltage;
        frontLeft.set(ControlMode.PercentOutput, leftSideVoltage / frontLeft.getBusVoltage());
        backLeft.set(ControlMode.PercentOutput, leftSideVoltage / backLeft.getBusVoltage());

        frontRight.set(ControlMode.PercentOutput, rightSideVoltage / frontRight.getBusVoltage());
        backRight.set(ControlMode.PercentOutput, rightSideVoltage / backRight.getBusVoltage());
    }

    /**
     * Returns the distance travelled by the left-side encoder since the last reset.
     *
     * @return The distance, in meters.
     */
    public double getLeftSideDistanceMeters() {
        return (leftEncoder.getDistance() / DrivetrainConstants.ENCODER_CPR)
                * 2
                * Math.PI
                * DrivetrainConstants.WHEEL_DIAMETER_M;
    }

    /**
     * Returns the distance travelled by the right-side encoder since the last reset.
     *
     * @return The distance, in meters.
     */
    public double getRightSideDistanceMeters() {
        return (rightEncoder.getDistance() / DrivetrainConstants.ENCODER_CPR)
                * 2
                * Math.PI
                * DrivetrainConstants.WHEEL_DIAMETER_M;
    }

    /**
     * Returns the smoothed velocity of the left-side encoder.
     *
     * @return The filtered velocity, in meters/second.
     */
    public double getLeftSideVelocityMetersPerSecond() {
        return leftEncoderVelocityFilter.calculate(
                (leftEncoder.getRate() / DrivetrainConstants.ENCODER_CPR)
                        * 2
                        * Math.PI
                        * DrivetrainConstants.WHEEL_DIAMETER_M);
    }

    /**
     * Returns the smoothed velocity of the right-side encoder.
     *
     * @return The filtered velocity, in meters/second.
     */
    public double getRightSideVelocityMetersPerSecond() {
        return rightEncoderVelocityFilter.calculate(
                (rightEncoder.getRate() / DrivetrainConstants.ENCODER_CPR)
                        * 2
                        * Math.PI
                        * DrivetrainConstants.WHEEL_DIAMETER_M);
    }

    /** Resets the position of right- and left- side encoders. */
    public void resetEncoders() {
        rightEncoder.reset();
        leftEncoder.reset();
    }

    /**
     * Sets the behavior of the motors when they do not recieve a signal.
     *
     * @param mode
     *     <p>{@code NeutralMode.Brake}: Motor attempts to bring all motion to a stop.
     *     <p>{@code NeutralMode.Coast}: Motor stops applying power.
     */
    public void setNeutralmode(NeutralMode mode) {
        frontLeft.setNeutralMode(mode);
        frontRight.setNeutralMode(mode);
        backLeft.setNeutralMode(mode);
        backRight.setNeutralMode(mode);
    }

    /**
     * Returns the current relative orientation of the IMU's gyroscope.
     *
     * @return The rotation as a Rotation2d.
     */
    public Rotation2d getRotationGyro() {
        if (Robot.isReal()) {
            return IMU.getRotation2d();
        } else {
            return new Rotation2d(simAngleRadians.get());
        }
    }

    /**
     * Begins calibrating the IMU's accelerometer, gyroscope, and magnetometer.
     *
     * <p>Must be done while the robot is not moving.
     */
    public void startCalibration() {
        if (Robot.isReal()) {
            IMU.calibrate();
        } else {
            simCalibrationTimer.start();
        }
        calibrationStarted = true;
    }

    /**
     * Returns true if the IMU is currently in the calibration sequence.
     *
     * @return Whether the IMU is calibrating.
     */
    public boolean isCalibrating() {
        if (Robot.isReal()) {
            return IMU.isCalibrating();
        } else {
            if (simCalibrationTimer.get() < 2) {
                return true;
            } else {
                simCalibrationTimer.stop();
                return false;
            }
        }
    }

    /**
     * Returns true if the IMU has finished the calibration sequence.
     *
     * @return Whether the IMU has finished self-calibrating.
     */
    public boolean isCalibrated() {
        return (calibrationStarted && !isCalibrating());
    }

    @Override
    public void periodic() {
        odometry.update(
                getRotationGyro(), getLeftSideDistanceMeters(), getLeftSideDistanceMeters());
        odometryDisplay.setRobotPose(odometry.getPoseMeters());

        odometryXLogger.update(odometry.getPoseMeters().getX());
        odometryYLogger.update(odometry.getPoseMeters().getY());

        leftPositionLogger.update(getLeftSideDistanceMeters());
        rightPositionLogger.update(getRightSideDistanceMeters());
        leftVelocityLogger.update(getLeftSideVelocityMetersPerSecond());
        rightVelocityLogger.update(getRightSideVelocityMetersPerSecond());

        SmartDashboard.putNumber("left", getLeftSideDistanceMeters());
        SmartDashboard.putNumber("right", getRightSideDistanceMeters());

        gyroAngleLogger.update(getRotationGyro().getRadians());
        calibrationStateLogger.update(isCalibrating());

        SmartDashboard.putData(odometryDisplay);
    }

    @Override
    public void simulationPeriodic() {
        drivetrainSim.update(0.02);

        drivetrainSim.setInputs(leftSideAppliedVoltage, rightSideAppliedVoltage);

        double metersToTicksScaling = (1 / (2 * Math.PI * DrivetrainConstants.WHEEL_DIAMETER_M));

        rightEncoderSim.setCount(
                (int) (drivetrainSim.getRightPositionMeters() * metersToTicksScaling));
        rightEncoderSim.setRate(
                (int) (drivetrainSim.getRightVelocityMetersPerSecond() * metersToTicksScaling));

        leftEncoderSim.setCount(
                (int) (drivetrainSim.getLeftPositionMeters() * metersToTicksScaling));
        leftEncoderSim.setRate(
                (int) (drivetrainSim.getLeftVelocityMetersPerSecond() * metersToTicksScaling));

        simAngleRadians.set(drivetrainSim.getHeading().getRadians());
    }
}
