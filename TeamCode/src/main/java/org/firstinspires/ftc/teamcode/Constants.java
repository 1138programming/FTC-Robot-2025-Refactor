package org.firstinspires.ftc.teamcode;

/** Central place for hardware names and tunable constants. */
public class Constants {
    /** Drivebase: motor names, odometry, PID, rotation. */
    public static class DrivebaseConstants{
        public static final String leftFrontName = "LeftFront";
        public static final String rightFrontName = "RightFront";
        public static final String leftBackName = "LeftBack";
        public static final String rightBackName = "RightBack";
        public static final double motorGearRatioDrivebase = ((1+(46.0/17.0))) * (1+(46.0/11.0));
        public static final double encoderTicksPerRevolution = ((((1.0+(46.0/17.0))) * (1+(46.0/11.0))) * 28.0);
        public static final double wheelCircumferenceIn = 4.09448819; //3.75
        public static final double maxMotorPower = 1.0;
        public static final double acceptableDriveError = 0.1;
        public static final double maxAngularWheelVelocity = 0.5;
        public static final double acceptableAngularError = 2;
        public static final double drivekP = 0.2;
        public static final double drivekI = 0.000000;
        public static final double drivekD = 0.000;
        public static final double drivekF = 0;
        public static final double rotationkP = 0.02;
        public static final double rotationkI = 0;
        public static final double rotationkD = 0.01;
        public static final double rotationkF = 0;
        public static final String navxSensorName = "navx";
        public static final double drivePIDMaxOutput = 0.6;
    }

    /** Flywheel: PID and encoder for velocity control. */
    public static class flywheelConstants{
        public static final String flywheelName = "Flywheel";
        public static final double flykP = 0.02;
        public static final double flykI = 0.000001;
        public static final double flykD = 0;
        public static final double flykF = 0;
        public static final double encoderTicksPerRotation =  28;
        public static final int flywheelTolerance = 20;
    }

    /** Intake motor config. */
    public static class IntakeConstants{
        public static final String intakeMotorName = "Intake";
    }

    /** Indexer servo config. */
    public static class IndexerConstants{
        public static final String indexerServoName = "Indexer";
    }

    /** Limelight 3A: device name, pipeline, navX for heading. */
    public static class LimelightConstants{
        public static final String limelightName = "Limelight67";
        public static final int limelightHzRate = 100;
        public static final int limelightPipeline = 0;
        public static final String navxSensorName = "navx";
    }

    /** Aligner: allowed April tag IDs and tuning (if using drive-based align). */
    public static class DrivebaseLimelightAlignerConstants{
        public static final int blueGoalAprilTagID = 20;
        public static final int redGoalAprilTagID = 24;
        public static final double alignRotationkP = 0.03;
        public static final double alignMaxRotation = 0.5;
        public static final double alignSpeed = 1.0;
    }

    public static class FieldRelativeDriveConstants{
        public static final float speed = 5; //speed multiplier for drivebase
        public static final double flywheelNormalPower = 0.75;
        public static final double flywheelExtraPower = 0.785;
    }

    public static class BlueSideAutonConstants{
        public static final float halfTileIn = 12;
        public static final float fullTileIn = 24;
        public static final float quarterTileIn = 6;
    }
}
