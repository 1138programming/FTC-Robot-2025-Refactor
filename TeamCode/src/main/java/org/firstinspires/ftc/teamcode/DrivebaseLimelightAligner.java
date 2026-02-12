package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.DrivebaseLimelightAlignerConstants.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivebaseLimelightAligner extends Subsystem{
    private Drivebase drivebase;
    private Limelight limelight;

    public DrivebaseLimelightAligner(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
    }
    protected void init(){
        drivebase = new Drivebase(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);
    }

    public void rotateToTarget(int targetID){
        int visibleTargetID = limelight.getTagID();
        if (targetID == redGoalAprilTagID || blueGoalAprilTagID){
            
        }
    }

}
