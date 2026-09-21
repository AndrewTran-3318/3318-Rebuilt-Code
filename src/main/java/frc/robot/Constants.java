package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


public final class Constants {
    //CAN IDS
    public static final int CANivore_ID = 21;
    public static final int PIGEON_ID = 13;

    public static final int INTAKE_ARM_RIGHT_ID = 14;
    public static final int INTAKE_ARM_LEFT_ID = 20;
    public static final int INTAKE_WHEEL_KRAKEN_ID_LEFT = 15;
    public static final int INTAKE_WHEEL_KRAKEN_ID_RIGHT = 16;

    public static final int LEFT_SHOOTER_KRAKEN_ID = 17; 
    public static final int RIGHT_SHOOTER_KRAKEN_ID = 18;

    public static final int FEEDER_SHOOTER_NEO_ID = 19; 
    public static final int HOPPER_ID = 22; 

    public static final int FRONT_LEFT_DRIVE_ID = 3;
    public static final int FRONT_LEFT_STEER_ID = 4;
    public static final int FRONT_LEFT_CANCODER_ID = 10;

    public static final int FRONT_RIGHT_DRIVE_ID = 1;
    public static final int FRONT_RIGHT_STEER_ID = 2;
    public static final int FRONT_RIGHT_CANCODER_ID = 9;

    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_STEER_ID = 8;
    public static final int BACK_LEFT_CANCODER_ID = 12;

    public static final int BACK_RIGHT_DRIVE_ID = 5;
    public static final int BACK_RIGHT_STEER_ID = 6;
    public static final int BACK_RIGHT_CANCODER_ID = 11;

    public static  final int ACTUATOR_ID = 0;

    public static final int HALL_EFFECT_HOMING_DIO_PORT = 0;
    public static final double HALL_EFFECT_DEBOUNCE_SEC = 0.02;

    public static final double HUB_HEIGHT = 72;

    public static final Transform3d CAMERAPOSITION = new Transform3d(1, 1, 1, new Rotation3d(1,1,1));
    public static final AprilTagFieldLayout APRILTAGLAYOUT= AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // These are the z=1.124m (44.25in) tags mounted on each alliance's HUB.
    // Red HUB center ≈ (11.92, 4.03), Blue HUB center ≈ (4.63, 4.03).
    public static final int[] HUB_TAG_IDS = {8, 9, 10, 11, 24,25, 26, 27};
    public static final int[] RED_HUB_TAG_IDS  = {8, 9, 10, 11};
    public static final int[] BLUE_HUB_TAG_IDS = {24, 25, 26, 27};

    public static final double TRACK_WIDTH_M = Units.inchesToMeters(21.75);  // 0.5525 m

    // Distance between the FRONT and BACK wheel centers (meters)
    public static final double WHEEL_BASE_M = Units.inchesToMeters(21.75);   // 0.5525 m

    // ---- Wheel & gear math ----
    // 4-inch wheel = 0.1016 m diameter
    public static final double WHEEL_DIAMETER_M     = Units.inchesToMeters(4.0);
    public static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M; // ~0.319 m

    // MK4 L2 gear ratios from the SDS datasheet
    public static final double DRIVE_GEAR_RATIO = 6.75;  // motor turns per 1 wheel turn
    public static final double STEER_GEAR_RATIO = 12.8;  // motor turns per 1 wheel turn

    // ---- Speed limits ----
    // Falcon 500 free speed ≈ 6380 RPM = 106.33 RPS
    // Wheel free speed = 106.33 / 6.75 = 15.75 RPS
    // Max speed = 15.75 × 0.319 ≈ 5.02 m/s theoretical
    // We cap at 4.5 to leave headroom for battery sag.
    public static final double MAX_TRANSLATION_MPS = 5;   // m/s
    public static final double MAX_ROTATION_RADPS  = 5;  // rad/s

    // The maximum physical RPS the drive motor can spin (used for velocity commands)
    // Falcon 500 free speed = 6380 RPM = 106.33 RPS at 12V
    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = 106.33;

    // ---- Module wheel locations (relative to robot center) ----
    // WPILib uses +X = forward, +Y = left
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d( WHEEL_BASE_M / 2.0,  TRACK_WIDTH_M / 2.0);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d( WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEEL_BASE_M / 2.0,  TRACK_WIDTH_M / 2.0);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE_M / 2.0, -TRACK_WIDTH_M / 2.0);


    public static final double INTAKE_LIFTER_UP_POS = 0.0; //define home rotation
    public static final double INTAKE_LIFTER_DOWN_POS = -6.5; //define down position

    public static final double INTAKE_LIFTER_HOMING_SPEED = 0.2;
    public static final double INTAKE_LIFTER_TOLERANCE = 0.25;

    // Webcam Constants
    public static final int CAMERA_DEVICE_ID = 0;
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final int CAMERA_FPS = 15;
    public static final int CAMERA_OVERLAY_STREAM_PORT = 1182;
    public static final Transform3d CAMERATOROBOTLOCATION = new Transform3d(new Translation3d(0, 9, 16.5), new Rotation3d(0, 10.5, 0)); //TODO

    public static final int APRILTAG_NUM_THREADS = 2;
    public static final float APRILTAG_QUAD_DECIMATE = 1.0f;
    public static final float APRILTAG_QUAD_SIGMA = 0.0f;
    public static final double APRILTAG_DECODE_SHARPENING = 0.25;
    public static final int APRILTAG_MIN_CLUSTER_PIXELS = 60;
    public static final int APRILTAG_MAX_NUM_MAXIMA = 10;
    public static final double APRILTAG_CRITICAL_ANGLE_RAD = Math.PI / 4.0;
    public static final float APRILTAG_MAX_LINE_FIT_MSE = 10.0f;
    public static final int APRILTAG_MIN_WHITE_BLACK_DIFF = 5;
    public static final boolean APRILTAG_DEGLITCH = false;
    public static final double APRILTAG_SIZE = 0.1651; //meters
   

    // ---- Limelight Info ----
    // 1/TX when april tag is barely within view
    public static final double TX_ROTATIONAL_SPEED = 0.5;
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int POSE_PIPELINE_INDEX = 0;
    public static final int HUB_PIPELINE_INDEX = 1;

    //Photon Vision
    public static final String PHOTON_NAME = "camera";


    //Manual shooter speed
    public static final double MANUAL_SHOOTER_SPEED = 57.0;

    // "Clear" pulse: briefly run feeder backward to prevent double-feeding
    public static final double CLEAR_POWER    = -0.25;
    public static final double CLEAR_TIME_SEC =  0.10;

    //Hopper, Feeder, Intake speed during feeding
    public static final double FEED_POWER = 0.8;
    public static final double INTAKE_POWER = 0;

    //Shooter Correction Factors - adjust with more real world data
    public static final double K1Offset = 4.06547;
    public static final double K2Offset = 4.30649;

    public static final double FLY_WHEEL_RADIUS = 4 * .0254;

    public static final double g = 9.81;
    public static final double SHOOTERHEIGHTINCHES = 16;
    public static final double HOODANGLEDEGREES = 60;

    public static final Pose2d RED_HUB_CORDS = new Pose2d(3.49885, 0, new Rotation2d()); //  137.75 in
    public static final Pose2d BLUE_HUB_CORDS = new Pose2d(-3.49885, 0, new Rotation2d()); //  137.75 in

    public static final double POSE_ALIGN_GAIN = 3;
    public static final double ALIGN_MINIMUM__ROTATION_SPEED = 1; //radian per second
    public static final double ALIGN_MAXIMUM_ROTATION_SPEED = 3; // radians per second
    public static final double ALIGN_MAXIXMUM_TRANSLATION_SPEED = 0.5; // meters per second
    public static final double ALIGN_ANGLE_TOLERANCE = 0.1; //radians


    public static final double RADIUS_SERVO_MOUNT = 0.125;
    public static final double MOUNT_OFFSET = 0.05;
    public static final double FIXED_X = 0.120;
    public static final double FIXED_Y = -0.120;

    public static final double ACTUATOR_LENGTH = 0.175;
    public static final double STROKE_LENGTH = 0.1;
    public static final double SHOOTER_SPEED = 10;
}
