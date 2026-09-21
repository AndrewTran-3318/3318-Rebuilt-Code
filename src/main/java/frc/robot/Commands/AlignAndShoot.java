package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class AlignAndShoot extends Command{
    private final Intake intake;
    private final Hopper hopper;
    private final Feeder feeder;
    private final Shooter shooter;
    //private final PhotonCamera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    private final Hood hood;

    private enum State {CLEAR, PREPARE, FEED, DONE}
    private State state;
    private Pose2d hubCoords = null;
    private Timer timer = new Timer();
    //private Timer lostTarget = new Timer();
    double targetSpeed = Constants.MANUAL_SHOOTER_SPEED;

    public AlignAndShoot(Intake intake, Hopper hopper, Feeder feeder, Shooter shooter, CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, PhotonCamera camera, Hood hood) {
        this.intake = intake;
        this.hopper = hopper;
        this.feeder = feeder;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.drive = drive;
        this.hood = hood;
        //this.camera = camera;

        addRequirements(intake, hopper, feeder, shooter, drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        //lostTarget.reset();
        state = State.CLEAR;
        //LimelightHelpers.setPipelineIndex(Constants.LIMELIGHT_NAME, Constants.HUB_PIPELINE_INDEX); //looks for only the hub april tags, limelight
        timer.start();
        //lostTarget.start();
        hubCoords = shooter.findHubCoords();
    }

    @Override
    public void execute() {
        switch(state){
            case CLEAR: //Runs the feeder motors backwards to clear any balls still stuck in the shooter
                feeder.setFeederSpeed(Constants.CLEAR_POWER);
                if (timer.hasElapsed(Constants.CLEAR_TIME_SEC)){
                    feeder.stop();
                    timer.stop();
                    state = State.PREPARE;
                }
                break;

            case PREPARE: //Aligns the bot with the hub and spins the shooter motors up to speed

                //locks the translational movement and turns the robot towards the hub
                drivetrain.setControl(
                    drive.withVelocityX(0) 
                        .withVelocityY(0) 
                        .withRotationalRate(1)
                        .withRotationalRate(calcRotation()) // Locks rotation towards april tag
                );
                targetSpeed = shooter.getCorrectedRPS(calcDistance());//spins the shooter up to speed (and maybe moves linear actuator)
                hood.setPosition(calcDistance());
                shooter.setShooterSpeed(targetSpeed); //Sets the shooter to the calculated speed



                //checks if robot is at the correction orientation and shooter is at speed
                if (Math.abs(calcCorrectionTheta()) < 3.0 && shooter.isAtSpeed(targetSpeed)) {
                    state = State.FEED;
                }
                break;

            case FEED: //Runs all of the motors required in the shooting sequence
                feeder.setFeederSpeed(Constants.FEED_POWER);
                hopper.setHopperSpeed(Constants.FEED_POWER);
                intake.setWheelSpeedPercentage(Constants.FEED_POWER);
                shooter.setShooterSpeed(targetSpeed);
                break;
            
            case DONE: //does nothing, transitions to isFinished
                break;
        }

        
    }

    @Override
    public void end(boolean interrupted) { //stops all motors and resets april tag
        intake.stopWheel();
        shooter.stop();
        feeder.stop();
        hopper.stop();

        //LimelightHelpers.setPipelineIndex(Constants.LIMELIGHT_NAME, Constants.POSE_PIPELINE_INDEX); //resets to looking for all tags

        state = State.DONE;
    }

    @Override 
    public boolean isFinished() {
        return state == State.DONE; 
    }

    // --- Helper Methods ---
    
    //find the alliance color and set the specific hub coords

    //calculates the angle the bot needs to turn at to face the hub, uses the bot's internal pose and odometry
    public double calcRotation(){
        double correctionTheta = calcCorrectionTheta();
        double output = correctionTheta * Constants.POSE_ALIGN_GAIN;
        if(Math.abs(correctionTheta) > Constants.ALIGN_ANGLE_TOLERANCE)
        {
            output+= Math.copySign(Constants.ALIGN_MINIMUM__ROTATION_SPEED, correctionTheta);
        }
        else
        {
            output = 0; //don't move if below tolerance
        }
        return MathUtil.clamp(output, -Constants.ALIGN_MAXIMUM_ROTATION_SPEED, Constants.ALIGN_MAXIMUM_ROTATION_SPEED);
    }

    //returns the difference between the angle between the robot and the hub and the actual rotation of the robot
    public double calcCorrectionTheta(){
        Pose2d swervePose = drivetrain.getState().Pose;
        double botRotation = drivetrain.getState().Pose.getRotation().getRadians();
        double targetTheta = Math.PI / 2.0; //the only time this won't be correctly set is when the target angle is pi/2 or -pi/2 b/c of divide by 0

        try{
            targetTheta = Math.atan( (swervePose.getY()-hubCoords.getY()) / (swervePose.getX() - hubCoords.getX())) + ((hubCoords.getX() - swervePose.getX() < 0)? Math.PI : 0);
        }
        catch(Exception e)
        {
        }
        return MathUtil.angleModulus(targetTheta-botRotation);
        //double correctionTheta = MathUtil.angleModulus(targetTheta-botRotation);
    }
    
    //returns the distance between the hub and the robot
    public double calcDistance(){
        Pose2d swervePose = drivetrain.getState().Pose;
        double xOffSet = swervePose.getX() - hubCoords.getX();
        double yOffSet = swervePose.getY() - hubCoords.getY();
        double distance = Math.sqrt(xOffSet * xOffSet + yOffSet * yOffSet);
        return distance;
    }
}
