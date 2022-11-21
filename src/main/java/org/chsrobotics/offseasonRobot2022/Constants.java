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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;
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
            public static final String CAMERA_NAME = "default";

            public static final double DIAGONAL_FOV_DEGREES = 180;

            public static final double CAMERA_PITCH_DEGREES = 0;
            public static final double CAMERA_HEIGHT_METERS = 0;

            public static final Transform2d TRANSFORMATION_CAMERA_TO_ROBOT =
                    new Transform2d(new Translation2d(0, 0), new Rotation2d(0));

            public static final int CAMERA_RESOLUTION_HORIZONTAL = 360;
            public static final int CAMERA_RESOLUTION_VERTICAL = 360;

            public static final double CONTOUR_THRESHOLD = 1;

            public static final double DEFAULT_ERROR_DISPLACEMENT_METERS_STDDEV = 0.01;
            public static final double DEFAULT_ERROR_ANGLE_RADIANS_STDDEV = 0.001;
        }

        public static final class DrivetrainConstants {
            public static final int FRONT_RIGHT_CANID = 2;
            public static final int BACK_RIGHT_CANID = 4;
            public static final int FRONT_LEFT_CANID = 1;
            public static final int BACK_LEFT_CANID = 2;

            public static final int FRONT_RIGHT_PDP_CHANNEL = 1; // change these
            public static final int BACK_RIGHT_PDP_CHANNEL = 2;
            public static final int FRONT_LEFT_PDP_CHANNEL = 3;
            public static final int BACK_LEFT_PDP_CHANNEL = 4;

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

            public static final double MODEL_LIN_KS = 0.75; // volts
            public static final double MODEL_LIN_KV = 2.3; // volts per (m/s)
            public static final double MODEL_LIN_KA = 0.4; // volts per (m/s^2)

            public static final Port NAVX_PORT = Port.kMXP;

            public static final double GYRO_ERROR_ANGLE_RADIANS_STDDEV = 0.01;
        }
    }

    public static final class CommandConstants {}
}
