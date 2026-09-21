package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

import frc.robot.subsystems.Hood;

public class ManualHood extends Command{
    private final Hood hood;
    private CommandXboxController opController;

    public ManualHood(Hood hood, CommandXboxController opController) {
        this.hood = hood;
        this.opController = opController;
        addRequirements(hood);
    }

    @Override
    public void execute()
    {
        hood.setPosition(4*Math.abs(opController.getLeftY()));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
