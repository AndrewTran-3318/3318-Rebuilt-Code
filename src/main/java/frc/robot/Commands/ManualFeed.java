package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class ManualFeed extends Command{
    private final Hopper hopper;
    private final Feeder feeder;

    public ManualFeed(Hopper hopper, Feeder feeder) {
        this.hopper = hopper;
        this.feeder = feeder;

        addRequirements(hopper, feeder);
    }

    @Override
    public void execute()
    {
        feeder.setFeederSpeed(Constants.FEED_POWER);
        //hopper.setHopperSpeed(Constants.FEED_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        hopper.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
