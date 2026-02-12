package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class DrivebaseConstants{
        public static final String leftFrontName = "LeftFront";
        public static final String rightFrontName = "RightFront";
        public static final String leftBackName = "LeftBack";
        public static final String rightBackName = "RightBack";
        public static final double encoderTicksPerRevolution = 537.7;
        public static final double wheelCircumferenceIn = 3.75;
        public static final double acceptableDriveError = 0.1;
        public static final double maxAngularWheelVelocity = 0.5;
        public static final double acceptableAngularError = 2;
        public static final double drivekP = 0.4;
        public static final double drivekI = 0.000001;
        public static final double drivekD = 0.002;
        public static final double drivekF = 0;
        public static final double rotationkP = 0.02;
        public static final double rotationkI = 0;
        public static final double rotationkD = 0.01;
        public static final double rotationkF = 0;
        public static final String navxSensorName = "navx";
    }

    public static class flywheelConstants{
        public static final String flywheelName = "Flywheel";
        public static final double flykP = 0.02;
        public static final double flykI = 0.000001;
        public static final double flykD = 0;
        public static final double flykF = 0;
        public static final double encoderTicksPerRotation =  28;
        public static final int flywheelTolerance = 20;
    }

    public static class IntakeConstants{
        public static final String intakeMotorName = "Intake";
    }

    public static class IndexerConstants{
        public static final String indexerServoName = "Indexer";
    }

    public static class LimelightConstants{
        public static final String limelightName = "Limelight67";
        public static final int limelightHzRate = 100;
        public static final int limelightPipeline = 0;
        public static final String navxSensorName = "navx";
    }

    public static class DrivebaseLimelightAlignerConstants{
        public static final int blueGoalAprilTagID = 20;
        public static final int redGoalAprilTagID = 24;

    }
}
