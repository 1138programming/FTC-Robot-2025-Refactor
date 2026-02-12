package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public Subsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        init();
    }
    protected abstract void init();
    public void update(){} //call repeatedly in teleop
    public void stop(){}
}
