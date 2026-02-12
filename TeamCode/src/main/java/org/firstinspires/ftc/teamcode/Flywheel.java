package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.flywheelConstants.*;

public class Flywheel extends Subsystem{
    private DcMotorEx flywheelMotor;
    private PIDFController flywheelPIDController;
    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
    }
    @Override
    protected void init(){
        flywheelMotor = hardwareMap.get(DcMotorEx.class, flywheelName);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelPIDController = new PIDFController(flykP, flykI, flykD, flykF);
    }
    private int getFlywheelRPM(){
        double flywheelVelocity = flywheelMotor.getVelocity();
        return (int) (flywheelVelocity / encoderTicksPerRotation) * 60;

    }
    public void setPower(double power){
        flywheelMotor.setPower(power);
    }
    public void assignPIDTarget(int RPM){
        flywheelPIDController.setSetPoint(RPM);
    }
    private boolean isPIDFTargetSet(){
        return !(flywheelPIDController.getSetPoint() == 0.0);
    }

    public void calculateFlywheelPID(){
        if (!isPIDFTargetSet()){
            return;
        }

        flywheelPIDController.setTolerance(flywheelTolerance);

        double motorOutputRaw = flywheelPIDController.calculate(getFlywheelRPM());
        double motorOutput;

        if (Math.abs(motorOutputRaw) > 1){
            motorOutput = 1;
        } else {
            motorOutput = motorOutputRaw;
        }

        setPower(motorOutput);
    }

    public void spinAtRPM(int RPM, int time){
        assignPIDTarget(RPM);

        long startTime = System.currentTimeMillis();
        long curTime = System.currentTimeMillis() + 10000;

        while (!flywheelPIDController.atSetPoint() || curTime - startTime > time){
            calculateFlywheelPID();
            curTime = System.currentTimeMillis();
        }
    }
    @Override
    public void stop(){
        flywheelMotor.setPower(0);
        flywheelPIDController.reset();
    }



}
