package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.Constants.DrivebaseLimelightAlignerConstants.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/** Uses Limelight to rotate the drivebase to center a specific April tag (red or blue goal). */
public class DrivebaseLimelightAligner extends Subsystem{
    private Drivebase drivebase;
    private Limelight limelight;

    public DrivebaseLimelightAligner(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
    }
    @Override
    protected void init(){
        drivebase = new Drivebase(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);
    }

    /** If targetID is red/blue goal tag and it's visible, rotates to center it (blocking). Ignores other tags. */
    public void rotateToTarget(int targetID){
        if (!(targetID == redGoalAprilTagID || targetID == blueGoalAprilTagID)){
            return;
        }

        LLResult result = limelight.getResult();
        if (result == null || !limelight.hasTarget(result)) return;

        List<LLResultTypes.FiducialResult> fiducialResults = limelight.getFiducialResults(result);
        LLResultTypes.FiducialResult targetFiducial = null;

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == targetID) {
                targetFiducial = fr;
                break;
            }
        }

        if (targetFiducial == null) return;

        double tx = targetFiducial.getTargetXDegrees();
        drivebase.rotateDegrees(-tx);
    }

}
