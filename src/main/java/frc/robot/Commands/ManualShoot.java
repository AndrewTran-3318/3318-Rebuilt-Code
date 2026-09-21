package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class ManualShoot extends Command{
    private final Intake intake;
    private final Hopper hopper;
    private final Feeder feeder;
    private final Shooter shooter;
    
    private enum State {CLEAR, PREPARE, FEED}
    private State state;
    private Timer timer = new Timer();

    private Boolean shooterAtSpeed = false; //Is the shooter at the correct RPS

    public ManualShoot(Intake intake, Hopper hopper, Feeder feeder, Shooter shooter) {
        this.intake = intake;
        this.hopper = hopper;
        this.feeder = feeder;
        this.shooter = shooter;

        addRequirements(intake, hopper, feeder, shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        state = State.CLEAR;
        timer.start();
    }

    @Override
    public void execute() {
        switch(state){
        case CLEAR:
            feeder.setFeederSpeed(Constants.CLEAR_POWER);

            if (timer.hasElapsed(Constants.CLEAR_TIME_SEC)){
                feeder.stop();
                timer.stop();
                state = State.PREPARE;
            }
            break;

        case PREPARE:
            shooter.setShooterSpeed(Constants.MANUAL_SHOOTER_SPEED);
            shooterAtSpeed = shooter.isAtSpeed(Constants.MANUAL_SHOOTER_SPEED);
            if(shooterAtSpeed) {
                state = State.FEED;
            }
            break;
        
        case FEED:
            shooter.setShooterSpeed(Constants.MANUAL_SHOOTER_SPEED);
            feeder.setFeederSpeed(Constants.FEED_POWER);
            hopper.setHopperSpeed(Constants.FEED_POWER);
            intake.setWheelSpeedPercentage(Constants.FEED_POWER);
            break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        intake.stopWheel();
        shooter.stop();
        feeder.stop();
        hopper.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
