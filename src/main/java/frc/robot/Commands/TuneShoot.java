package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class TuneShoot extends Command{
    private final Shooter shooter;
    
    private static int shooterSpeed = 53;

    private static XboxController operator = new XboxController(1);

    private Boolean shooterAtSpeed = false; //Is the shooter at the correct RPS

    public TuneShoot(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }


    @Override
    public void execute() {
        if(operator.getAButtonReleased())
        {
           shooterSpeed++;
        }
        else if(operator.getBButtonReleased())
        {
           shooterSpeed--;
        }
        shooter.setShooterSpeed(shooterSpeed);
        shooterAtSpeed = shooter.isAtSpeed(shooterSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
