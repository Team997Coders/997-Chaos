package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TeleopControl extends CommandBase {
    private final CommandXboxController driverController = 
    new CommandXboxController(Constants.DRIVE_CONTROLLER_PORT);

    private final Drivetrain drivetrain;
    private final Shooter shooter;

    private final Timer timer = new Timer();

    private double driveModifier;

    public TeleopControl(Drivetrain drivetrain, Shooter shooter, double driveModifier) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.driveModifier = driveModifier;

    }

    @Override
    public void initialize() {
      timer.start();
    }

    @Override
    public void execute() {
        boolean aButton = driverController.button(1).getAsBoolean();
        boolean bButton = driverController.button(2).getAsBoolean();
        boolean yButton = driverController.button(3).getAsBoolean();
        boolean rightTriger = driverController.rightTrigger().getAsBoolean();
        boolean leftTrigger = driverController.leftTrigger().getAsBoolean();
        
            double turn = driverController.getRightX();
            double forward = driverController.getLeftY();
            if (turn >0){
              drivetrain.setOutput(forward*driveModifier, (forward+turn)*driveModifier);
            } else {
              drivetrain.setOutput((forward-turn)*driveModifier,forward*driveModifier);
            }
            
            if (rightTriger == true){
              drivetrain.setOutput(-1*driveModifier, 1*driveModifier);
            } else if (leftTrigger == true){
              drivetrain.setOutput(1*driveModifier, -1*driveModifier);
            }
    
            if (aButton==true){
              shooter.setFlywheelOutput(1.0);
            } else if (aButton==false) {
              shooter.setFlywheelOutput(0);
              timer.reset();
            }

         //   shooter.setServoAngle(90);
    
             if (timer.get() >= 0.75) {
              shooter.setServoAngle(90);
            } else if (timer.get() < 0.75) {
              shooter.setServoAngle(165);
            }
    }

}
