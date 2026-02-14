package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Constants.BlueSideAutonConstants.*;
@Autonomous(name="BlueSideAutonKatieIan4167WooHoo", group="Linear OpMode")
public class BlueSideAuton extends LinearOpMode {
    Drivebase drivebase;
    DrivebaseLimelightAligner aligner;
    Flywheel flywheel;
    Indexer indexer;
    Intake intake;
    Limelight limelight;

    public void resetPIDTargets(){
        flywheel.assignPIDTarget(0);
        drivebase.setDrivePIDFTargets(0);
    }

    public void moveBackToShootPreloadsAndShootPreloads(){
        resetPIDTargets();

        float oneAndAHalfTiles = fullTileIn + halfTileIn;
        int timeDrivingBack = 3000;
        long startTime = System.currentTimeMillis();

        drivebase.setDrivePIDFTargets(-oneAndAHalfTiles);
        flywheel.assignPIDTarget(4000);

        while(System.currentTimeMillis() - startTime < timeDrivingBack){
            drivebase.updateAuton();
            flywheel.updateAuton();
        }

        resetPIDTargets();


    }

    @Override
    public void runOpMode(){
        waitForStart();

        drivebase = new Drivebase(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);
        aligner = new DrivebaseLimelightAligner(hardwareMap, telemetry, drivebase, limelight);

        moveBackToShootPreloadsAndShootPreloads();
    }

}
