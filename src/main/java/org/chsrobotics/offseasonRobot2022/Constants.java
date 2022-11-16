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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.SerialPort.Port;
import org.chsrobotics.lib.util.GearRatioHelper;

public final class Constants {
    public static final class GlobalConstants {
        public static final double GLOBAL_NOMINAL_VOLTAGE = 11.5;
    }

    public static final class InputConstants {
        public static final int JOYSTICK_PORT = 0;
        public static final int JOYSTICK_LEFT_VERTICAL_AXIS = 1;
        public static final int JOYSTICK_RIGHT_HORIZONTAL_AXIS = 4;
    }

    public static final class SubsystemConstants {
        public static final class InertialMeasurementUnitConstants {
            public static final Port IMU_PORT = Port.kUSB;
        }

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

            public static final LinearFilter RIGHT_ENCODER_VELOCITY_SMOOTHING =
                    LinearFilter.movingAverage(5);
            public static final LinearFilter LEFT_ENCODER_VELOCITY_SMOOTHING =
                    LinearFilter.movingAverage(5);

            public static final GearRatioHelper ENCODER_TO_GROUND_HELPER =
                    new GearRatioHelper(1, 7);
        }
    }

    public static final class CommandConstants {
        public static final class TeleopDriveCommandConstants {
            public static final SimpleMotorFeedforward LEFT_FEEDFORWARD =
                    new SimpleMotorFeedforward(0, 0, 0);
            public static final SimpleMotorFeedforward RIGHT_FEEDFORWARD =
                    new SimpleMotorFeedforward(0, 0, 0);
        }
    }
}
