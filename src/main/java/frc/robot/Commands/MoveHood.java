package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;

public class MoveHood extends Command{

    private final Hood hood;
    CommandSwerveDrivetrain drivetrain;
    
    public MoveHood(Hood hood, CommandSwerveDrivetrain drivetrain)
    {
        this.hood = hood;
        this.drivetrain=drivetrain;
        addRequirements(this.hood);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hood.setPosition(calddist());

    }

    @Override
    public void end(boolean interrupted) {
    
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double calddist()
    {
        return  Math.sqrt(Math.pow(drivetrain.getState().Pose.getX() - Constants.RED_HUB_CORDS.getX(), 2)+Math.pow(drivetrain.getState().Pose.getY() - Constants.RED_HUB_CORDS.getY(), 2));
    }
}

