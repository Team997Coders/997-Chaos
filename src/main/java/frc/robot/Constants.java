// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class Drivetrain {
      public static int FRONT_RIGHT_CAN_ID = 2;
      public static int BACK_RIGHT_CAN_ID = 4;
      public static int FRONT_LEFT_CAN_ID = 1;
      public static int BACK_LEFT_CAN_ID = 3;
      public static boolean INVERT_LEFT = true;
      public static boolean INVERT_RIGHT = false;
  }
  public static int DRIVE_CONTROLLER_PORT = 0;

  public static final class Shooter {
    public static int TOP_FLYWHEEL_CAN_ID = 5;
    public static int BOTTOM_FLYWHEEL_CAN_ID = 6;

    public static int SERVO_CHANNEL = 0;

    public static boolean FLYWHEEL_IS_INVERTED = true;
  }


}
