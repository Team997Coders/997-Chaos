package frc.robot.commands;



import org.chsrobotics.lib.drive.differential.ArcadeDrive;
import org.chsrobotics.lib.drive.differential.DifferentialDriveMode;
import org.chsrobotics.lib.drive.differential.DifferentialDrivetrainInput;
import org.chsrobotics.lib.input.JoystickAxis;
import org.chsrobotics.lib.input.XboxController;
import org.chsrobotics.lib.telemetry.HighLevelLogger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TeleopControl extends CommandBase {
    private final XboxController driverController = new XboxController(Constants.DRIVE_CONTROLLER_PORT); 

    private final DifferentialDriveMode driveMode;


    private final Drivetrain drivetrain;
    private final Shooter shooter;

    private final Timer timer = new Timer();

    public TeleopControl(Drivetrain drivetrain, Shooter shooter) {
      this.drivetrain = drivetrain;
      this.shooter = shooter;
      JoystickAxis linear = driverController.leftStickVerticalAxis();
      JoystickAxis rotational = driverController.rightStickHorizontalAxis();
      rotational.setInverted(true);

      // HighLevelLogger.getInstance().logMessage(rotational.isInverted() ? "STICK INVERTED": "STICK NOT INVERTED");      

      driveMode = new ArcadeDrive(linear, rotational, 1, 1, Constants.Drivetrain.LINEAR_RAMP_RATE, Constants.Drivetrain.ANGULAR_RAMP_RATE);
    }

    @Override
    public void initialize() {
      timer.start();
    }

    @Override
    public void execute() {
        // boolean aButton = driverController.AButton().getAsBoolean();
        // boolean bButton = driverController.BButton().getAsBoolean();
        // boolean yButton = driverController.YButton().getAsBoolean();
       // boolean rightTriger = driverController.rightTriggerAxis();
      // boolean leftTrigger = driverController.leftTrigger().getAsBoolean();

      drivetrain.setOutput(driveMode.execute());
        
        //     double turn = driverController.getRightX();
        //     double forward = driverController.getLeftY();
        //     if (turn >0){
        //       drivetrain.setOutput(forward*driveModifier, (forward+turn)*driveModifier);
        //     } else {
        //       drivetrain.setOutput((forward-turn)*driveModifier,forward*driveModifier);
        //     }
            
        //     if (rightTriger == true){
        //       drivetrain.setOutput(-1*driveModifier, 1*driveModifier);
        //     } else if (leftTrigger == true){
        //       drivetrain.setOutput(1*driveModifier, -1*driveModifier);
        //     }
    
        //     if (aButton==true){
        //       shooter.setFlywheelOutput(1.0);
        //     } else if (aButton==false) {
        //       shooter.setFlywheelOutput(0);
        //       timer.reset();
        //     }

        //  //   shooter.setServoAngle(90);
    
        //      if (timer.get() >= 0.75) {
        //       shooter.setServoAngle(90);
        //     } else if (timer.get() < 0.75) {
        //       shooter.setServoAngle(165);
        //     }
    }

}
