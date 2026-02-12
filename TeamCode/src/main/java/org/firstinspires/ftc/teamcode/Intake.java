package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.*;


public class Intake extends Subsystem{
    private DcMotorEx intakeMotor;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
    }
    @Override
    protected void init(){
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorName);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
    }

    @Override
    public void stop(){
        intakeMotor.setPower(0);
    }

    @Override
    public void update(){
        telemetry.addData("Intake power", intakeMotor.getPower());
        telemetry.update();
    }




}
