package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.ctre.phoenix6.CANBus;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TunerConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionEstimation;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Commands.SpinIntakeWheelMotor;
import frc.robot.Commands.TuneShoot;
import frc.robot.Commands.ManualShoot;
import frc.robot.Commands.MoveHood;
import frc.robot.Commands.MoveIntakeArm;
import frc.robot.Commands.AlignAndShoot;
import frc.robot.Commands.HomeIntakeArm;
import frc.robot.Commands.ManualFeed;
import frc.robot.Commands.ManualHood;
import frc.robot.Commands.MoveIntakeArmDown;

public class RobotContainer {
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    private final Intake intakeSubsystem = new Intake();
    private final Hopper hopperSubsystem = new Hopper();
    private final Shooter shooterSubsystem = new Shooter();
    private final Feeder feederSubsystem = new Feeder();
    private final Hood hoodSubsystem = new Hood();
    private final CANBus driveTrainCanBus = new CANBus("drivetrain");

    private final PhotonCamera camera = new PhotonCamera(Constants.PHOTON_NAME);
    private final VisionEstimation VisionEstimator = new VisionEstimation(camera);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(VisionEstimator, driveTrainCanBus);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MAX_TRANSLATION_MPS * 0.2).withRotationalDeadband(Constants.MAX_ROTATION_RADPS * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();



    public Command m_autoSelected = null;
    public final SendableChooser<Command> m_chooser;

    public RobotContainer() {
        configureControllerButtons();

        // ---- Sets the hopper's default state to be running
        // hopperSubsystem.setDefaultCommand(
        //     new RunCommand(
        //         () -> hopperSubsystem.setHopperSpeed(-0.1),
        //         hopperSubsystem
        //     )
        // );
        
        // configureWebCam(); //Deprecated

        //new HomeIntakeArm(intakeSubsystem).schedule();
        //CommandScheduler.getInstance().schedule(new HomeIntakeArm(intakeSubsystem));

    NamedCommands.registerCommand("AutoShootBalls", new AlignAndShoot(intakeSubsystem, hopperSubsystem, feederSubsystem, shooterSubsystem, drivetrain, drive, camera, hoodSubsystem)
                                                    .withTimeout(6.0)
    );
    NamedCommands.registerCommand("ManualShoot", new ManualShoot(intakeSubsystem, hopperSubsystem, feederSubsystem, shooterSubsystem)
                                                    .withTimeout(6.0));
    NamedCommands.registerCommand("IntakeArmDown", new MoveIntakeArmDown(intakeSubsystem));
    NamedCommands.registerCommand("Intake", new SpinIntakeWheelMotor(intakeSubsystem, hopperSubsystem, -0.5)
                                                            .withTimeout(3.0));

    Command RightNeutral = AutoBuilder.buildAuto("RightNeutralAuton");
    Command LeftNeutral = AutoBuilder.buildAuto("LeftNeutralAuton");
    Command OnlyShoot = AutoBuilder.buildAuto("OnlyShootBalls");
    Command HumanDump = AutoBuilder.buildAuto("HumanDump");
    Command ProjectHailMary = AutoBuilder.buildAuto("ProjectHailMary");

      
    m_chooser = AutoBuilder.buildAutoChooser();
    m_chooser.setDefaultOption("Do Nothing", new InstantCommand());
    m_chooser.addOption("RightNeutralAuton", RightNeutral);
    m_chooser.addOption("LeftNeutralAuton", LeftNeutral);
    m_chooser.addOption("OnlyShootBalls", OnlyShoot);
    m_chooser.addOption("HumanDump", HumanDump);
    m_chooser.addOption("ProjectHailMary", ProjectHailMary);
    SmartDashboard.putData("Auto choices", m_chooser);

    }
    
    @Deprecated
    // public void configureWebCam()
    // {
    //     UsbCamera camera = CameraServer.startAutomaticCapture(Constants.CAMERA_DEVICE_ID);
    //     camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    //     camera.setResolution(640, 480);
    //     camera.setFPS(10);

    //     if (camera != null) 
    //     {
    //         new VisionThread(camera, lastFrameTimeStamp ).start();
    //     }
    // }

    public void configureControllerButtons() {
        
        //Manual Shooter, set at 53 RPS
        operator.rightBumper().whileTrue(new ManualShoot(intakeSubsystem, hopperSubsystem, feederSubsystem, shooterSubsystem)); 
        //Auto Align and Shoot
        operator.rightTrigger().whileTrue(new AlignAndShoot(intakeSubsystem, hopperSubsystem, feederSubsystem, shooterSubsystem, drivetrain, drive, camera, hoodSubsystem));
        
        operator.leftTrigger().whileTrue(new SpinIntakeWheelMotor(intakeSubsystem, hopperSubsystem, Constants.INTAKE_POWER)); //Outtake
        operator.leftBumper().whileTrue(new SpinIntakeWheelMotor(intakeSubsystem, hopperSubsystem, -Constants.INTAKE_POWER)); //Intake

        operator.y().whileTrue(new HomeIntakeArm(intakeSubsystem));
        operator.start().onTrue(new MoveIntakeArmDown(intakeSubsystem)); 
        operator.x().whileTrue(new MoveHood(hoodSubsystem, drivetrain));

        // operator.start().toggleOnTrue(new TuneShoot(shooterSubsystem));
        
        operator.back().whileTrue(new ManualFeed(hopperSubsystem, feederSubsystem));

        operator.a().whileTrue(
            new RunCommand(
                () -> hopperSubsystem.setHopperSpeed(operator.getRightY()), hopperSubsystem)
        );

        operator.b().whileTrue(new ManualHood(hoodSubsystem, operator));
        

        //Deprecated, limelight
        //  driver.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
        //       drive.withVelocityX(driver.getLeftY() * Constants.MAX_TRANSLATION_MPS) // Drive forward with negative Y (forward)
        //       .withVelocityY(driver.getLeftX() * Constants.MAX_TRANSLATION_MPS) // Drive left with negative X (left)
        //       .withRotationalRate(LimelightHelpers.getTX(Constants.LIMELIGHT_NAME) * -Constants.TX_ROTATIONAL_SPEED) // Locks rotation towards april tag
        //       )
        //  ); 

        //test
        /* 
        driver.rightTrigger().whileTrue(drivetrain.applyRequest(() -> {
            var results = camera.getAllUnreadResults();
            PhotonTrackedTarget target = null;
            for(var result: results){
                if (result.hasTargets()){
                    target = result.getBestTarget();
                }
            }
              return drive.withVelocityX(driver.getLeftY() * Constants.MAX_TRANSLATION_MPS) // Drive forward with negative Y (forward)
              .withVelocityY(driver.getLeftX() * Constants.MAX_TRANSLATION_MPS) // Drive left with negative X (left)
              .withRotationalRate(target.getYaw() * -Constants.TX_ROTATIONAL_SPEED); // Locks rotation towards april tag
            })
        ); 
        */

        driver.rightBumper().whileTrue(drivetrain.applyRequest(() -> {
            Pose2d swervePose = drivetrain.getState().Pose;
            double botRotation = drivetrain.getState().Pose.getRotation().getRadians();
            double targetTheta = Math.PI / 2.0; //the only time this won't be correctly set is when the target angle is pi/2 or -pi/2 b/c of divide by 0

            Pose2d hubCords = shooterSubsystem.findHubCoords();
            try{
                targetTheta = Math.atan( (swervePose.getY()-hubCords.getY()) / (swervePose.getX() - hubCords.getX())) + ((hubCords.getX() - swervePose.getX() < 0)? Math.PI : 0);
            }
            catch(Exception e)
            {
            }
                double correctionTheta = MathUtil.angleModulus(targetTheta-botRotation);
                double output = correctionTheta * Constants.POSE_ALIGN_GAIN;
                if(Math.abs(correctionTheta) > Constants.ALIGN_ANGLE_TOLERANCE)
                {
                    output+= Math.copySign(Constants.ALIGN_MINIMUM__ROTATION_SPEED, correctionTheta);
                }
                else
                {
                    output = 0; //don't move if below tolerance
                }
                SmartDashboard.putNumber("Target", targetTheta);
                SmartDashboard.putNumber("BotRot", botRotation);
              return drive.withVelocityX(driver.getLeftY() * Constants.ALIGN_MAXIXMUM_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
              .withVelocityY(driver.getLeftX() * Constants.ALIGN_MAXIXMUM_TRANSLATION_SPEED) // Drive left with negative X (left)
              .withRotationalRate(MathUtil.clamp(output, -Constants.ALIGN_MAXIMUM_ROTATION_SPEED, Constants.ALIGN_MAXIMUM_ROTATION_SPEED)); // Locks rotation towards april tag
            })
        ); 



        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getLeftY() * Constants.MAX_TRANSLATION_MPS) // Drive forward with negative Y (forward)
                    .withVelocityY(driver.getLeftX() * Constants.MAX_TRANSLATION_MPS) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * Constants.MAX_ROTATION_RADPS) // Drive counterclockwise with negative X (left)
            )

        );

        // // Idle while the robot is disabled. This ensures the configured
        // // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // drivetrain.registerTelemetry(logger::telemeterize);
    }

}
