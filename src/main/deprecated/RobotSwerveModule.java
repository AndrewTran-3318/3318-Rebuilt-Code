package frc.robot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotSwerveModule {

    private int driveMotorCANID;
    private int swerveMotorCANID;
    private int CANCoderID;
    private TalonFX driveMotor;
    private TalonFX swerveMotor;
    private CANcoder absoluteEncoder;

    private String moduleName;

    private double angleOffset;

    private final double wheelRadiusMeters = 0.1;
    private final double maxLinearSpeed = Constants.MAX_TRANSLATION_MPS;
    private final double maxAngularSpeed = Constants.MAX_ROTATION_RADPS;

    private PositionVoltage positionRequest = new PositionVoltage(0);

    public RobotSwerveModule(int driveMotorCANID, int swerveMotorCANID, int CANCoderID, String moduleName) {
        this.driveMotorCANID = driveMotorCANID;
        this.swerveMotorCANID = swerveMotorCANID;
        this.CANCoderID = CANCoderID;
        driveMotor = new TalonFX(driveMotorCANID);
        swerveMotor = new TalonFX(swerveMotorCANID);
        absoluteEncoder = new CANcoder(CANCoderID);
        this.moduleName = moduleName;
        
        // resetToAbsolute();
    }

    public String getName()
    {
        return this.moduleName;
    }

    public void resetToAbsolute() {
        angleOffset = -1 * absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                RPS2MPS(driveMotor.getVelocity().getValueAsDouble()),
                new Rotation2d(ROT2RAD(getSwervePosition())));
    }

    public void setDesiredState(SwerveModuleState newState) {
        newState.optimize(new Rotation2d(ROT2RAD(getSwervePosition())));

        driveMotor.set(newState.speedMetersPerSecond / maxLinearSpeed);
        
        swerveMotor.setControl(positionRequest.withPosition(newState.angle.getRotations()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                ROT2MET(driveMotor.getPosition().getValueAsDouble()),
                new Rotation2d(ROT2RAD(getSwervePosition())));
    }

    private double ROT2MET(double rotations) {
        return rotations * 2 * Math.PI * wheelRadiusMeters;
    }

    private double RPM2MPS(double RPM) {
        return RPM * Math.PI * wheelRadiusMeters / 30.0;
    }

    private double RPS2MPS(double RPS)
    {
        return RPS * 2 * Math.PI * wheelRadiusMeters;
    }

    private double ROT2RAD(double rotations) {
        return rotations * 2 * Math.PI;
    }

    public double getCanCoderPosition()
    {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getSwervePosition()
    {
        return swerveMotor.getPosition().getValueAsDouble() + angleOffset;
    }
    public double getDriveVelocity()
    {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public double getDriveTemp()
    {
        return driveMotor.getDeviceTemp().getValueAsDouble();
    }
    public double getSwerveTemp()
    {
        return swerveMotor.getDeviceTemp().getValueAsDouble();
    }
}
