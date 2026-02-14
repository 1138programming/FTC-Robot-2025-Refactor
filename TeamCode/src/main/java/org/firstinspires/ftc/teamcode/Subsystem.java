package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Base for robot subsystems. Constructor calls init() after storing hardwareMap and telemetry. */
public abstract class Subsystem {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public Subsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        init();
    }
    /** Override to get hardware and set up state. */
    protected abstract void init();
    /** Call each loop in teleop to run periodic logic / telemetry. Default no-op. */
    public void update(){}
    /** Call when stopping or switching mode; stop motors and reset state. */
    public void stop(){}

    public void updateAuton(){}
}
