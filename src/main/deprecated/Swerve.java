package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotSwerveModule;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final Translation2d m_frontLeftModuleLocation = new Translation2d(-1, 2);
    private final Translation2d m_frontRightModuleLocation = new Translation2d(1, 2);
    private final Translation2d m_backLeftModuleLocation = new Translation2d(1, -2);
    private final Translation2d m_backRightModuleLocation = new Translation2d(-1, 2);

    private final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
            m_frontLeftModuleLocation,
            m_frontRightModuleLocation,
            m_backLeftModuleLocation,
            m_backRightModuleLocation);

    private SwerveDrivePoseEstimator m_PoseEstimator;
    private static Pigeon2 gyro = new Pigeon2(Constants.PIGEON);

   

    private RobotSwerveModule frontLeftSwerveMod = new RobotSwerveModule(
        Constants.FRONT_LEFT_DRIVE, 
        Constants.FRONT_LEFT_STEER, 
        Constants.FRONT_LEFT_CANCODER,
        "FrontLeft");
    private RobotSwerveModule frontRightSwerveMod = new RobotSwerveModule(
        Constants.FRONT_RIGHT_DRIVE, 
        Constants.FRONT_RIGHT_STEER , 
        Constants.FRONT_RIGHT_CANCODER,
        "FrontRight");
    private RobotSwerveModule backLeftSwerveMod = new RobotSwerveModule(
        Constants.BACK_LEFT_DRIVE, 
        Constants.BACK_LEFT_STEER , 
        Constants.BACK_LEFT_CANCODER,
        "BackLeft");
    private RobotSwerveModule backRightSwerveMod = new RobotSwerveModule(
        Constants.BACK_RIGHT_DRIVE, 
        Constants.BACK_RIGHT_STEER , 
        Constants.BACK_RIGHT_CANCODER,
        "BackRight");
    private RobotSwerveModule[] swerveModules = {
            frontLeftSwerveMod,
            frontRightSwerveMod,
            backLeftSwerveMod,
            backRightSwerveMod
    };

    private double maxLinearSpeed = 4.5;
    private PIDController headingPidController = new PIDController(0.1, 0, 0.05);    

    public Swerve() {
        headingPidController.enableContinuousInput(-Math.PI, Math.PI);
        m_PoseEstimator = new SwerveDrivePoseEstimator(
                m_Kinematics,
                getHeading(),
                getModulePositions(),
                new Pose2d());
        for(RobotSwerveModule sm : swerveModules)
        {
            sm.resetToAbsolute();
        }
    }

    /**
     * FL,FR,BL,BR
     * 
     * @return an array of swerve module positions (distance, angle)
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modPositions = {
                frontLeftSwerveMod.getModulePosition(),
                frontRightSwerveMod.getModulePosition(),
                backLeftSwerveMod.getModulePosition(),
                backRightSwerveMod.getModulePosition()
        };
        return modPositions;
    }

    /**
     * 
     * @return The robots pose based on the swerve's odometry
     */
    public Pose2d getPose() {
        return m_PoseEstimator.getEstimatedPosition();
    }

    /**
     * Swerve Drive Inputer Handler
     * 
     * @param strafeControllerInput      LeftX Controller Input: controls left-right
     *                                   motion
     * @param translationControllerInput LeftY Controller Input: controls
     *                                   forward-backward motion
     * @param rotationControllerInput    RightX Controller Input: controls rotation
     */
    public void drive(double strafeControllerInput, double translationControllerInput, double rotationControllerInput) {
        maxLinearSpeed = 4.5;
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                strafeControllerInput,
                translationControllerInput,
                rotationControllerInput,
                getHeading());

        SwerveModuleState[] swerveModuleStates = m_Kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxLinearSpeed);

        frontLeftSwerveMod.setDesiredState(swerveModuleStates[0]);
        frontRightSwerveMod.setDesiredState(swerveModuleStates[1]);
        backLeftSwerveMod.setDesiredState(swerveModuleStates[2]);
        backRightSwerveMod.setDesiredState(swerveModuleStates[3]);
    }

    //Im unsure how position data with camera is going to work, so that will be handled else where
    public void driveFaceHub(double strafeControllerInput, double translationControllerInput)
    {
        maxLinearSpeed = 2;
        double targetHeading = getFieldHeadingToHub(); // rads
        headingPidController.setSetpoint(targetHeading);
        double clampedRotationalInput = MathUtil.clamp(
            headingPidController.calculate(getConstrainedGyroAngleInRadians()),
            -1,
        1
        ); // only would act weird if you are behind hub

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                strafeControllerInput,
                translationControllerInput,
                clampedRotationalInput,
                getHeading());

        SwerveModuleState[] swerveModuleStates = m_Kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxLinearSpeed);

        frontLeftSwerveMod.setDesiredState(swerveModuleStates[0]);
        frontRightSwerveMod.setDesiredState(swerveModuleStates[1]);
        backLeftSwerveMod.setDesiredState(swerveModuleStates[2]);
        backRightSwerveMod.setDesiredState(swerveModuleStates[3]);
    }
    
    private double getConstrainedGyroAngleInRadians()
    {
        double currentAngle = gyro.getYaw().getValueAsDouble();
        int rotations = (int)(currentAngle / 360);
        double constrainedAngle = currentAngle - rotations * 360;
        return Math.toRadians(constrainedAngle);
    }

    //Returns radians
    // GYRO POSITIVE IS CCW
    public double getFieldHeadingToHub()
    {
        double hubXCord = 1;
        double hubYCord = 2; //meters

        double robotXCord = m_PoseEstimator.getEstimatedPosition().getX();
        double robotYCord = m_PoseEstimator.getEstimatedPosition().getY();

        double deltaX = hubXCord- robotXCord;
        double deltaY = hubYCord - robotYCord;

        double offsetToVertical = -1 * Math.PI / 2.0;

        if(deltaX == 0 ) return Math.PI / 2;
        else
        {
            double angleInRads = Math.atan(deltaY/deltaX);
            return deltaX < 0 ?  angleInRads + Math.PI +offsetToVertical: angleInRads+offsetToVertical; //accounts for domain restriction on arctan
        }
    }


    /**
     * 
     * @return Yaw angle from gyroscope in radians
     */
    public Rotation2d getHeading() {
        return new Rotation2d(gyro.getYaw().getValueAsDouble() * Math.PI / 180.0);
    }

    public RobotSwerveModule[] getSwerveModules()
    {
        return swerveModules;
    }

    @Override
    public void periodic() {
        m_PoseEstimator.update(getHeading(), getModulePositions());
        
         for(RobotSwerveModule swervemod : swerveModules)
         {
            SmartDashboard.putNumber(swervemod.getName()+"CanCoder:", swervemod.getCanCoderPosition());
            SmartDashboard.putNumber(swervemod.getName()+"SwervePos:", swervemod.getSwervePosition());
            SmartDashboard.putNumber(swervemod.getName()+"DriveVel:", swervemod.getDriveVelocity());
            SmartDashboard.putNumber(swervemod.getName()+"DriveTemp:", swervemod.getDriveTemp());
            SmartDashboard.putNumber(swervemod.getName()+"SwerveTemp:", swervemod.getSwerveTemp());
         }
         SmartDashboard.putNumber("PoseX:", m_PoseEstimator.getEstimatedPosition().getX());
         SmartDashboard.putNumber("PoseY:", m_PoseEstimator.getEstimatedPosition().getY());
         SmartDashboard.putNumber("GyroHeading", gyro.getYaw().getValueAsDouble());
    }

    public void faceHub()
    {
        while(getFieldHeadingToHub()-getHeading().getRadians() > 0.1)
        {
            driveFaceHub(0, 0);
        }
    }
}
