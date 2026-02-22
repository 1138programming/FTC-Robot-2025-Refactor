package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Constants.BlueSideAutonConstants.*;
@Autonomous(name="RedSideAutonKatieIan4167WooHoo", group="Linear OpMode")
public class RedSideAuton extends LinearOpMode {
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

    public void moveBackAndShootPreloads(){
        drivebase.resetDrivePIDFs();
        drivebase.resetMotorEncoders();
        resetPIDTargets();

        int timeDrivingBack = 3000;
        long startTime = System.currentTimeMillis();

        drivebase.setDrivePIDFTargets(-12);
        flywheel.assignPIDTarget(3750);

        while(!drivebase.pidsAtTargets()){ //move back and rev flywheel
            drivebase.updateAuton();
            flywheel.updateAuton();
        }
        drivebase.stop();

        resetPIDTargets();
        flywheel.assignPIDTarget(3750);

        TimeHandler firstTimeHandler = new TimeHandler(13000);

        while (!firstTimeHandler.isTimeExpired()){
            flywheel.updateAuton();
            if (firstTimeHandler.timeInRange(3000, 4000)){
                indexer.setPower(1);
                continue;
            }

            if (firstTimeHandler.timeInRange(5000, 7700)){
                intake.setPower(-1);
                indexer.setPower(1);
                continue;
            }


            if (firstTimeHandler.timeInRange(7700, 13000)){
                indexer.setPower(1);
                intake.setPower(-1);
                continue;
            }

            intake.setPower(0);
        }

        drivebase.rotateDegrees(-90);
        drivebase.driveDistance(-18);

//        drivebase.resetDrivePIDFs();
//        drivebase.setDrivePIDFTargets(-28);
//        while(drivebase.pidsAtTargets()){
//            drivebase.updateAuton();
//        }
    }

    public void intakeBalls(){
        flywheel.stop();

        drivebase.rotateDegrees(100);
        drivebase.stop();
        drivebase.resetDrivePIDFs();
        drivebase.setDrivePIDFTargets(60);

        while (!drivebase.pidsAtTargets()){
            drivebase.calculateDrivebasePID();
            intake.setPower(-1);
        }

        while (!aligner.targetIDDetected(20)){
            drivebase.rotateDegrees(5);
        }
        TimeHandler timeHandler = new TimeHandler(5000);

        drivebase.resetFieldRot();
        while (!timeHandler.isTimeExpired()){
            drivebase.driveFieldRelative(1, 0 , 0, false, 0.5f);
        }

        timeHandler = new TimeHandler(5000);
        flywheel.resetPID();
        flywheel.assignPIDTarget(3500);

        while(!timeHandler.isTimeExpired()){
            indexer.setPower(1);
            intake.setPower(-1);
            flywheel.updateAuton();
        }



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

        moveBackAndShootPreloads();
    }

}
