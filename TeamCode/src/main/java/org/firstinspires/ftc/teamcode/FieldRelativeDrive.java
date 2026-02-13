package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.FieldRelativeDriveConstants.*;

/** TeleOp with field-relative drive; instantiates drivebase, flywheel, indexer, intake. */
@TeleOp(name="FieldRelative42", group="Linear OpMode")
public class FieldRelativeDrive extends LinearOpMode {
    Drivebase drivebase;
    Flywheel flywheel;
    Indexer indexer;
    Intake intake;
    boolean lastPress = false;

    public void baseDrivingInput(){
        Gamepad baseDriver = gamepad1;
        float leftStickX = baseDriver.left_stick_x;
        float leftStickY = baseDriver.left_stick_y;
        float rightStickX = baseDriver.right_stick_x;

        drivebase.driveFieldRelative(
           leftStickX, leftStickY, rightStickX, false, speed
        );
    }

    public void armsDrivingInput(){
        Gamepad baseDriver = gamepad1;
        //Indexer input handling
        if (baseDriver.y){
            indexer.setPower(1);
        } else if (baseDriver.x) {
            indexer.setPower(-1);
        } else {
            indexer.setPower(0);
        }

        //Intake input handling
        if (baseDriver.a){
            intake.setPower(1);
        } else if (baseDriver.b){
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        //Flywheel input handling
        if (baseDriver.right_bumper){
            flywheel.setPower(flywheelNormalPower); //normal mode power
        } else if (baseDriver.right_trigger > 0){
            flywheel.setPower(flywheelExtraPower);
        } else {
            flywheel.setPower(0);
        }


        if (gamepad1.dpad_up && !lastPress){
            drivebase.resetFieldRot();
            lastPress = true;
        } else {
            lastPress = false;
        }


    }

    @Override
    public void runOpMode(){
        drivebase = new Drivebase(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Gamepad baseDriver = gamepad1;
        waitForStart();

        drivebase.resetFieldRot();
        while (opModeIsActive()){
            baseDrivingInput();
            armsDrivingInput();
            telemetry.update();
        }
    }
}
