package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
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

    public void moveBackAndShootPreloads(){
        drivebase.resetDrivePIDFs();
        drivebase.resetMotorEncoders();
        resetPIDTargets();

        int timeDrivingBack = 3000;
        long startTime = System.currentTimeMillis();

        drivebase.setDrivePIDFTargets(-7);
        flywheel.assignPIDTarget(3500);

        while(!drivebase.pidsAtTargets()){ //move back and rev flywheel
            drivebase.updateAuton();
            flywheel.updateAuton();
        }
        drivebase.stop();

        resetPIDTargets();
        flywheel.assignPIDTarget(3500);

        TimeHandler firstTimeHandler = new TimeHandler(9000);

        while (!firstTimeHandler.isTimeExpired()){
            flywheel.updateAuton();
            if (firstTimeHandler.timeInRange(2500, 4000)){
                indexer.setPower(1);
                intake.setPower(-1);
                continue;
            }

            if (firstTimeHandler.timeInRange(6000, 8000)){
                indexer.setPower(1);
            }
        }
    }

    public void intakeBalls(){
        double rotError = drivebase.calculateRotPID(
                drivebase.getYaw() - 90
        );

        while (!drivebase.rotReachedTarget(rotError)){
            rotError = drivebase.calculateRotPID(
                    drivebase.getYaw() - 90
            );
            telemetry.addData("rotError", rotError);
            telemetry.addData("Yaw", drivebase.getYaw());
            telemetry.update();
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
        intakeBalls();
    }

}
