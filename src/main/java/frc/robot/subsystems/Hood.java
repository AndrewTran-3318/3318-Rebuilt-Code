package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private final Servo actuator;

    public Hood() {
        actuator = new Servo(Constants.ACTUATOR_ID);
        
    }

    public void setPosition(double distance) {
        SmartDashboard.putNumber("hooddistance", distance);
        try{
        actuator.setPulseTimeMicroseconds((int)MathUtil.clamp(1000+getServoPositionFromTarget(getTargetAngleFromDistance(distance)),1000, 2000));

        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("servoPosition", actuator.get());
    }

    public double getTargetAngleFromDistance(double xDistance)
    {
        double A = -(Constants.g *xDistance*xDistance)/(2*Constants.SHOOTER_SPEED*Constants.SHOOTER_SPEED);
        double B = xDistance;
        double C = A - (Constants.HUB_HEIGHT-Constants.SHOOTERHEIGHTINCHES)*0.0254;

        double targetAngle = Math.atan((-1*B + Math.sqrt(B*B-4*A*C))/(2*A));
        return targetAngle;
    }

    public double getServoPositionFromTarget(double targetAngle)
    {
        double mountToFixedX = Constants.RADIUS_SERVO_MOUNT * Math.cos(targetAngle + Constants.MOUNT_OFFSET) - Constants.FIXED_X;
        double mountToFixedY = Constants.RADIUS_SERVO_MOUNT * Math.sin(targetAngle + Constants.MOUNT_OFFSET) - Constants.FIXED_Y;

        double targetBarLength = Math.sqrt(mountToFixedX * mountToFixedX + mountToFixedY*mountToFixedY);

        double targetServoPos = (targetBarLength - Constants.ACTUATOR_LENGTH)/Constants.STROKE_LENGTH;
        SmartDashboard.putNumber("targetServo", targetServoPos);
        return 1000*targetServoPos;
    }
}
