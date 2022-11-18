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

import edu.wpi.first.wpilibj.SerialPort.Port;
import org.chsrobotics.lib.util.GearRatioHelper;

public final class Constants {
    public static final class GlobalConstants {
        public static final double GLOBAL_NOMINAL_VOLTAGE_VOLTS = 11.5;
    }

    public static final class InputConstants {
        public static final int JOYSTICK_PORT = 0;
        public static final int JOYSTICK_LEFT_VERTICAL_AXIS = 1;
        public static final int JOYSTICK_RIGHT_HORIZONTAL_AXIS = 4;
    }

    public static final class SubsystemConstants {
        public static final class DrivetrainConstants {
            public static final int FRONT_RIGHT_CANID = 2;
            public static final int BACK_RIGHT_CANID = 4;
            public static final int FRONT_LEFT_CANID = 1;
            public static final int BACK_LEFT_CANID = 2;

            public static final int LEFT_ENCODER_CHANNEL_A = 4;
            public static final int LEFT_ENCODER_CHANNEL_B = 5;

            public static final int RIGHT_ENCODER_CHANNEL_A = 2;
            public static final int RIGHT_ENCODER_CHANNEL_B = 3;

            public static final int ENCODER_CPR = 2048;

            public static final boolean LEFT_MOTORS_INVERTED = true;
            public static final boolean RIGHT_MOTORS_INVERTED = false;

            public static final boolean LEFT_ENCODER_INVERTED = true;
            public static final boolean RIGHT_ENCODER_INVERTED = true;

            public static final int RIGHT_ENCODER_VELOCITY_SMOOTHING_SAMPLES = 5;
            public static final int LEFT_ENCODER_VELOCITY_SMOOTHING_SAMPLES = 5;

            public static final double RIGHT_ENCODER_NOISE_DISPLACEMENT_METERS_STDDEV = 0;
            public static final double LEFT_ENCODER_NOISE_DISPLACEMENT_METERS_STDDEV = 0;

            public static final double RIGHT_ENCODER_NOISE_VELOCITY_METERSPERSECOND_STDDEV = 0;
            public static final double LEFT_ENCODER_NOISE_VELOCITY_METERSPERSECOND_STDDEV = 0;

            public static final double TRACKWIDTH_EFFECTIVE_M = 1;

            public static final double WHEEL_DIAMETER_M = 0.152;

            public static final GearRatioHelper MOTOR_TO_WHEEL_HELPER =
                    new GearRatioHelper(1, 10.49);

            public static final double MODEL_ROT_KS = 1; // volts
            public static final double MODEL_ROT_KV = 2; // volts per (m/s)
            public static final double MODEL_ROT_KA = 4; // volts per (m/s^2)

            public static final double MODEL_LIN_KS = 1; // volts
            public static final double MODEL_LIN_KV = 2; // volts per (m/s)
            public static final double MODEL_LIN_KA = 4; // volts per (m/s^2)

            public static final Port NAVX_PORT = Port.kMXP;

            public static final double GYRO_NOISE_ANGLE_RADIANS_STDDEV = 0;

            public static final double ODOMETRY_NOISE_X_METERS_STDDEV = 0;
            public static final double ODOMETRY_NOISE_Y_METERS_STDDEV = 0;
        }
    }

    public static final class CommandConstants {
        public static final class TeleopDriveCommandConstants {
            public static final double LEFT_KS = 0;
            public static final double LEFT_KV = 0;
            public static final double LEFT_KA = 0;

            public static final double RIGHT_KS = 0;
            public static final double RIGHT_KV = 0;
            public static final double RIGHT_KA = 0;
        }
    }
}
