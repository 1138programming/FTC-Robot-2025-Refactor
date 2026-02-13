package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.teamcode.Constants.DrivebaseConstants.*;

/** Mecanum drivebase with field-relative drive, encoder distance, and gyro-based rotation. */
public class Drivebase extends Subsystem{
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private IntegratingGyroscope gyro;
    private Orientation orientation;
    private OpenGLMatrix rotMatrix;
    private OpenGLMatrix fieldForwardRot;
    private AngularVelocity angle;
    private float fieldRot;
    private PIDFController lPIDF, rPIDF;
    private PIDFController rotationController;
    private double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    public Drivebase(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
    }

    @Override
    protected void init(){
        // Motors: left side reversed, right forward; all brake when power 0
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontName);
        leftBack = hardwareMap.get(DcMotorEx.class, leftBackName);
        rightBack = hardwareMap.get(DcMotorEx.class, rightBackName);
        gyro = hardwareMap.get(NavxMicroNavigationSensor.class, navxSensorName);


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        orientation = gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        rotMatrix =  orientation.getRotationMatrix();
        angle = gyro.getAngularVelocity(AngleUnit.DEGREES);

        lPIDF = new PIDFController(drivekP, drivekI, drivekD, drivekF);
        rPIDF = new PIDFController(drivekP, drivekI, drivekD, drivekF);
        rotationController = new PIDFController(rotationkP, rotationkI, rotationkD, rotationkF);
    }

    /** Mecanum mix: axial, lateral, yaw; then scale to maxMotorPower if needed. */
    private void getMotorPowers(double axial, double lateral, double yaw, double speed){
        double max;

        leftFrontPower  = (axial + lateral + yaw) * speed;
        rightFrontPower = (axial - lateral - yaw) * speed;
        leftBackPower   = (axial - lateral + yaw) * speed;
        rightBackPower  = (axial + lateral - yaw) * speed;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > maxMotorPower) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
    }

    /** Apply computed powers; reversed inverts all (driver perspective flip). */
    private void setMotorPowers(boolean reversed){
        if (!reversed){
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);
        } else {
            leftFront.setPower(-leftFrontPower);
            leftBack.setPower(-leftBackPower);
            rightFront.setPower(-rightFrontPower);
            rightBack.setPower(-rightBackPower);
        }
    }

    /** Robot frame: +x = right, +y = forward. reversed flips all powers (e.g. driver facing intake). */
    public void drive(double xVelocity, double yVelocity, double rot, boolean reversed, double speed){
        double axial = -yVelocity;  // mecanum convention: forward is negative axial
        double lateral = xVelocity;
        double yaw = rot;

        getMotorPowers(axial, lateral, yaw, speed);
        setMotorPowers(reversed);
    }

    /** Field-relative: +x = field right, +y = field forward (where forward = heading at last resetFieldRot()). rot is still robot-relative. */
    public void driveFieldRelative(float xVelocity, float yVelocity, float rot, boolean reversed, float speed){
        orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
        rotMatrix = Orientation.getRotationMatrix(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,  getAdjustedAngle() - fieldRot, (float) 0, (float) 0);

        VectorF SVector = new VectorF(new float[] {xVelocity, yVelocity, 0, 1});
        VectorF nVector =  rotMatrix.multiplied(SVector);

        drive(nVector.get(0), nVector.get(1), rot, reversed, speed);
    }

    /** Heading in [0, 360) from gyro (intrinsic ZYX). Used for field-relative; getYaw() is [-180, 180] and used for rotation. */
    public float getAdjustedAngle() {
        orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
        if (orientation.firstAngle < 0) {
            return 180 + (180 + orientation.firstAngle);  // map (-180, 0) -> (180, 360)
        }
        else {
            return orientation.firstAngle;
        }
    }

    /** Set current heading as field forward. Call at teleop start (or when driver wants forward = current direction). */
    public void resetFieldRot() {
        fieldRot = getAdjustedAngle();
    }

    /** Distance this motor has moved in inches. */
    public double getEncoderDistance(DcMotorEx motor){
        double currentMotorPos = motor.getCurrentPosition();
        return (currentMotorPos / encoderTicksPerRevolution) * wheelCircumferenceIn;
    }

    /** Zero encoder counts; leave motors in RUN_USING_ENCODER. */
    private void resetMotorEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Reset drive PIDs (e.g. before new driveDistance). */
    private void resetDrivePIDFs(){
        lPIDF.reset();
        rPIDF.reset();
    }

    /** Blocking: drive straight until both sides reach target distance. Uses left/right front encoders only. */
    public void driveDistance(double inches){
        resetMotorEncoders();
        resetDrivePIDFs();
        lPIDF.setSetPoint(inches);
        rPIDF.setSetPoint(inches);

        lPIDF.setTolerance(acceptableDriveError);
        rPIDF.setTolerance(acceptableDriveError);

        while (!(lPIDF.atSetPoint() || rPIDF.atSetPoint())) {
            double leftOutput = lPIDF.calculate(getEncoderDistance(leftFront));
            double rightOutput = rPIDF.calculate(getEncoderDistance(rightFront));

            leftFront.setPower(leftOutput);
            leftBack.setPower(leftOutput);
            rightFront.setPower(rightOutput);
            rightBack.setPower(rightOutput);
        }

        resetMotorEncoders();
    }
    /** Yaw in degrees, typically [-180, 180]. Used for rotation error. */
    private double getYaw(){
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
    }

    private void resetRotPIDF(){
        rotationController.reset();
    }

    /** Cap rotation power to maxAngularWheelVelocity. */
    private double clampRotationOutput(double output){
        return Math.max(-maxAngularWheelVelocity, Math.min(maxAngularWheelVelocity, output));
    }

    /** Blocking: rotate by given degrees (relative). Error is wrapped so we take the shortest path (e.g. 2° not 358°). */
    public void rotateDegrees(double degrees){
        resetRotPIDF();
        double yaw = getYaw();
        double target = degrees + yaw;
        double error = target + acceptableAngularError + 1;

        // stop when within tolerance, or when only long-way path remains (avoid 358° spin)
        while (Math.abs(error) >= acceptableAngularError && 180 - Math.abs(error) >= acceptableAngularError) {
            error = target - getYaw();
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            double output = rotationController.calculate(0, error);

            leftFront.setPower(clampRotationOutput(output));
            leftBack.setPower(clampRotationOutput(output));
            rightFront.setPower(-output);
            rightBack.setPower(-output);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    @Override
    public void stop(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        resetRotPIDF();
        resetDrivePIDFs();
        resetMotorEncoders();
    }

    @Override
    public void update(){
        telemetry.addData("Gyro:",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Distance:", getEncoderDistance(leftFront));
        telemetry.addData("Ticks:", leftFront.getCurrentPosition());
    }





}
