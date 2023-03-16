package frc;

import org.chsrobotics.lib.telemetry.DashboardChooser;

public class Config {
    public static enum DriveMode implements DashboardChooser.Option {
        ARCADE("Arcade"), 
        CURVATURE("Curvature"),
        MIXED_ARCADE("Mixed Arcade"),
        MIXED_CURVATURE("Mixed Curvature"),
        TANK("Tank");

        private final String displayName;

        DriveMode(String displayName) {
            this.displayName = displayName;
        }

        @Override
        public String getDisplayName() {
            return displayName;
        }
    }
    public static DashboardChooser<DriveMode> DRIVE_MODE_CHOOSER = DashboardChooser.fromEnum(DriveMode.class, DriveMode.ARCADE);
}
