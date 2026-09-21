package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveIntakeArmDown extends Command {
    private final Intake intakeSubsystem;
    private final SparkClosedLoopController[] pidControllers;         
    private final RelativeEncoder[] encoders;  
    private boolean reachedClosePosition = false;

    public MoveIntakeArmDown(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.pidControllers= intakeSubsystem.getPIDController();
        this.encoders = intakeSubsystem.getEncoder();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //start moving to the up or down position based on if it's currently up or down
        //if (intakeSubsystem.isArmUp()) {
        pidControllers[0].setSetpoint(Constants.INTAKE_LIFTER_DOWN_POS, ControlType.kPosition);
        pidControllers[1].setSetpoint(-Constants.INTAKE_LIFTER_DOWN_POS *2, ControlType.kPosition);
        //}
    }

    @Override
    public void execute() {
        if (!reachedClosePosition){
            if ((Math.abs(encoders[0].getPosition() - Constants.INTAKE_LIFTER_DOWN_POS) < Constants.INTAKE_LIFTER_TOLERANCE)
                && (Math.abs(encoders[1].getPosition() + 2*Constants.INTAKE_LIFTER_DOWN_POS) < Constants.INTAKE_LIFTER_TOLERANCE)    ){
                reachedClosePosition = true;

                intakeSubsystem.stop();
            }
        }
    }

    @Override
    public boolean isFinished() {
        //continuously check if it has reached the up/down position (fault tolerance of 0.05)
        return reachedClosePosition;
    }

    @Override
    public void end(boolean interrupted) {
        //intakeSubsystem.stop();
        intakeSubsystem.stop();
        reachedClosePosition = false;
    }
}