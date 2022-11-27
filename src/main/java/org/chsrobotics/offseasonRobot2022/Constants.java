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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SerialPort.Port;
import java.io.File;
import org.chsrobotics.lib.util.GearRatioHelper;

public final class Constants {
    public static final class GlobalConstants {
        public static final double GLOBAL_NOMINAL_VOLTAGE_VOLTS = 12;
    }

    public static final class LocalizationConstants {
        public static final double ODOMETRY_ERROR_METERS_STDDEV = 0.5;
        public static final double MODEL_ERROR_THETA_RADIANS_STDDEV = 0.5;
        public static final double MODEL_ERROR_DIST_METERS_STDDEV = 0.5;
    }

    public static final class InputConstants {
        public static final int JOYSTICK_PORT = 0;
        public static final int JOYSTICK_LEFT_VERTICAL_AXIS = 1;
        public static final int JOYSTICK_RIGHT_HORIZONTAL_AXIS = 4;
    }

    public static final class SubsystemConstants {
        public static final class VisionConstants {
            public static final String CAMERA_1_NAME = "default";

            public static final double DIAGONAL_FOV_DEGREES = 148;
            public static final double HORIZONTAL_FOV_DEGREES = 118;

            public static final double CAMERA_PITCH_DEGREES = 0;
            public static final double CAMERA_HEIGHT_METERS = 0;

            public static final Transform2d TRANSFORMATION_ROBOT_TO_CAMERA =
                    new Transform2d(new Translation2d(0.2, 0.2), new Rotation2d(0));

            public static final int CAMERA_RESOLUTION_HORIZONTAL = 640;
            public static final int CAMERA_RESOLUTION_VERTICAL = 480;

            public static final double CONTOUR_THRESHOLD = 1;

            public static final double DEFAULT_ERROR_DISPLACEMENT_METERS_STDDEV = 0.01;
            public static final double DEFAULT_ERROR_ANGLE_RADIANS_STDDEV = 0.001;
        }

        public static final class LEDDisplayConstants {
            public static final int DEFAULT_LED_ANIMATION = 0;
            // public static final LEDStripAnimation DEFAULT_LED_ANIMATION = ...
            public static final int LAUNCHER_PREPARING_LED_ANIMATION = 0;
            // public static final LEDStripAnimation LAUNCHER_PREPARING_LED_ANIMATION = ...
            public static final int LAUNCHER_LAUNCHING_LED_ANIMATION = 0;
            // public static final LEDStripAnimation LAUNCHER_LAUNCHING_LED_ANIMATION = ...
        }

        public static final class DrivetrainConstants {
            public static final NeutralMode DEFAULT_NEUTRAL_MODE = NeutralMode.Coast;

            public static final int FRONT_RIGHT_CANID = 2;
            public static final int BACK_RIGHT_CANID = 4;
            public static final int FRONT_LEFT_CANID = 1;
            public static final int BACK_LEFT_CANID = 3;

            public static final int FRONT_RIGHT_PDP_CHANNEL = 0;
            public static final int BACK_RIGHT_PDP_CHANNEL = 0;
            public static final int FRONT_LEFT_PDP_CHANNEL = 0;
            public static final int BACK_LEFT_PDP_CHANNEL = 0;

            public static final int LEFT_ENCODER_CHANNEL_A = 4;
            public static final int LEFT_ENCODER_CHANNEL_B = 5;

            public static final int RIGHT_ENCODER_CHANNEL_A = 2;
            public static final int RIGHT_ENCODER_CHANNEL_B = 3;

            public static final int ENCODER_CPR = 2048;

            public static final boolean LEFT_MOTORS_INVERTED = true;
            public static final boolean RIGHT_MOTORS_INVERTED = false;

            public static final boolean LEFT_ENCODER_INVERTED = true;
            public static final boolean RIGHT_ENCODER_INVERTED = true;

            public static final int ENCODER_VELOCITY_SMOOTHING_SAMPLES = 5;

            public static final double ENCODER_ERROR_DISPLACEMENT_METERS_STDDEV = 0.01;

            public static final double ENCODER_ERROR_VELOCITY_METERSPERSECOND_STDDEV = 0.05;

            public static final double TRACKWIDTH_EFFECTIVE_M = 0.65;

            public static final double WHEEL_DIAMETER_M = 0.152;

            public static final GearRatioHelper MOTOR_TO_WHEEL_HELPER =
                    new GearRatioHelper(1, 10.49);

            public static final double MODEL_ROT_KS = 0.75; // volts
            public static final double MODEL_ROT_KV = 2.3; // volts per (m/s)
            public static final double MODEL_ROT_KA = 0.4; // volts per (m/s^2)

            public static final double MODEL_LIN_KS = 0.57; // volts
            public static final double MODEL_LIN_KV = 2.15; // volts per (m/s)
            public static final double MODEL_LIN_KA = 0.5; // volts per (m/s^2)

            public static final double WHEEL_VELOCITY_KP = 0.1;
            public static final double WHEEL_VELOCITY_KI = 0;
            public static final double WHEEL_VELOCITY_KD = 0;

            public static final Port NAVX_PORT = Port.kMXP;

            public static final double GYRO_ERROR_ANGLE_RADIANS_STDDEV = 0.01;
        }

        public static final class LauncherConstants {
            public static final File launcherMapCSV =
                    new File(Filesystem.getDeployDirectory(), "launcherMap.csv");
            public static final int FORWARD_FLYWHEEL_MOTOR_CANID = 5;
            public static final int AFT_FLYWHEEL_MOTOR_CANID = 6;

            public static final int HOOD_MOTOR_CANID = 7;

            public static final int FORWARD_FLYWHEEL_MOTOR_PDP_CHANNEL = 0;
            public static final int AFT_FLYWHEEL_MOTOR_PDP_CHANNEL = 0;

            public static final int HOOD_MOTOR_PDP_CHANNEL = 0;

            public static final boolean FORWARD_FLYWHEEL_MOTOR_INVERTED = false;
            public static final boolean AFT_FLYWHEEL_MOTOR_INVERTED = false;

            public static final boolean HOOD_MOTOR_INVERTED = false;

            public static final boolean FLYWHEEL_ENCODER_INVERTED = false;

            public static final boolean HOOD_ENCODER_INVERTED = false;

            public static final int HOOD_LIMIT_SWITCH_DIO = 6;

            public static final int HOOD_POTENTIOMETER_ANALOG_IN = 0;

            public static final boolean HOOD_LIMIT_SWITCH_TRUE_WHEN_ACTIVATED = true;

            public static final FeedbackDevice HOOD_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;

            public static final int HOOD_ENCODER_CPR = 2048;

            public static final GearRatioHelper MOTOR_TO_HOOD = new GearRatioHelper(1, 30);

            public static final double HOOD_KS = 1.5; // volts
            public static final double HOOD_KG_GAIN = 0.2; // volts * cos(hood angle)
            public static final double HOOD_KV = 1; // volts / (rad/s)
            public static final double HOOD_KA = 1; // volts / (rad/s^2)

            public static final double HOOD_KP = 12;
            public static final double HOOD_KI = 5;
            public static final double HOOD_KD = 1;

            public static final double HOOD_MAX_VEL = 2 * Math.PI; // rad / s
            public static final double HOOD_MAX_ACCEL = 4 * Math.PI; // rad / s^2

            public static final int HOOD_ENCODER_VELOCITY_SMOOTHING_SAMPLES = 10;

            public static final double HOOD_MAX_ANGLE_RADIANS = Math.PI / 2;

            public static final double HOOD_MOMENT = 0.0086; // kg m^2
            public static final double HOOD_RADIUS_METERS = 0.5;
            public static final double HOOD_MASS_KG = 0.9;

            public static final double HOOD_ENCODER_ERROR_RADIANS_STDDEV = 0.001;

            public static final double FLYWHEEL_KS = 1.5; // volts
            public static final double FLYWHEEL_KV = 0.0075; // volts / (rad/s)
            public static final double FLYWHEEL_KA = 0.01; // volts / (rad/s^2)

            public static final double FLYWHEEL_KP = 0.3;
            public static final double FLYWHEEL_KI = 0;
            public static final double FLYWHEEL_KD = 0;

            public static final double FLYWHEEL_MOMENT = 0.01; // kg m^2

            public static final double FLYWHEEL_ENCODER_ERROR_RADIANS_STDDEV = 0.001;

            public static final int FLYWHEEL_ENCODER_VELOCITY_SMOOTHING_SAMPLES = 10;

            public static final Type FLYWHEEL_ENCODER_TYPE = Type.kQuadrature;
            public static final int FLYWHEEL_ENCODER_CPR = 2048;

            public static final GearRatioHelper MOTOR_TO_FLYWHEEL = new GearRatioHelper(18, 22);
        }
    }

    public static final class CommandConstants {
        public static final class TrajectoryFollowerConstants {
            public static final double RAMSETE_CONVERGENCE = 2;
            public static final double RAMSETE_DAMPING = 0.7;
        }

        public static final class AutoPivotConstants {
            public static final double KP = 1;
            public static final double KI = 0;
            public static final double kD = 0;

            public static final double KS = 0; // volts
            public static final double KV = 0; // volts / (rad/sec)
            public static final double KA = 0; // volts / (rad/sec^2)

            public static final double MAX_V_RADS_PER_SEC = 4 * Math.PI;
            public static final double MAX_A_RADS_PER_SEC_SQ = 8 * Math.PI;
        }

        public static final class LauncherControllerConstants {
            public static final double HOOD_DEFAULT_ANGLE_RADIANS = Math.PI / 6; // 30 degrees

            public static final double FLYWHEEL_IDLE_VELOCITY_RADS_PER_SECOND = 0;
            public static final double FLYWHEEL_STANDBY_VELOCITY_RADS_PER_SECOND = 50 * Math.PI;
        }

        public static final class AutoLaunchConstants {
            // time for the drivetrain to wait after stopping, to ensure it loses its momentum
            public static final double DRIVETRAIN_INITIAL_FREEZE_DURATION_SECONDS = 0.2;
        }
    }
}
