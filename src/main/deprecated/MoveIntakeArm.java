// Currently unused code. Please do not delete, since this would be useful if we 
// decided to go back to automatically deciding to move up or down.

package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveIntakeArm extends Command {
    private final Intake intakeSubsystem;
    private final SparkClosedLoopController pidController;  
    private final RelativeEncoder encoder;  


    private boolean armWasUp;

    public MoveIntakeArm(Intake intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.pidController = intakeSubsystem.getPIDController();
        this.encoder = intakeSubsystem.getEncoder();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //start moving to the up or down position based on if it's currently up or down
        if (intakeSubsystem.isArmUp()) {
            armWasUp = true;
            pidController.setSetpoint(Constants.INTAKE_LIFTER_DOWN_POS, ControlType.kPosition);
        } else {
            armWasUp = false;
            pidController.setSetpoint(Constants.INTAKE_LIFTER_UP_POS, ControlType.kPosition);
        }
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        //continuously check if it has reached the up/down position (fault tolerance of 0.05)
        if(armWasUp){
            return Math.abs(encoder.getPosition() - Constants.INTAKE_LIFTER_DOWN_POS) < Constants.INTAKE_LIFTER_TOLERANCE;
        }else{
            return Math.abs(encoder.getPosition() - Constants.INTAKE_LIFTER_UP_POS) < Constants.INTAKE_LIFTER_TOLERANCE;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}