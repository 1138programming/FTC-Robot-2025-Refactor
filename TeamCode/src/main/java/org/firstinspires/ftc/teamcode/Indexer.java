package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.*;

/** Indexer as continuous-rotation servo; power controls direction and speed. */
public class Indexer extends Subsystem{
    CRServo indexer;

    public Indexer(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
    }
    @Override
    protected void init(){
        indexer = hardwareMap.get(CRServo.class, indexerServoName);
    }

    /** Power -1 to 1; sign is direction. */
    public void setPower(double power){
        indexer.setPower(power);
    }

    @Override
    public void stop(){
        setPower(0);
    }
}
