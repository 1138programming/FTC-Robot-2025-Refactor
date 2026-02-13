package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import static org.firstinspires.ftc.teamcode.Constants.LimelightConstants.*;

import java.util.List;

/** Limelight 3A wrapper: pipelines, fiducials (April tags), tx/ty, and bot pose. Uses navX for heading in pose. */
public class Limelight extends Subsystem{
    private Limelight3A limelight;
    private IntegratingGyroscope gyro;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
    }

    @Override
    protected void init(){
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        gyro = hardwareMap.get(NavxMicroNavigationSensor.class, navxSensorName);
        limelight.setPollRateHz(limelightHzRate);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /** Latest frame from camera; may be null or invalid. */
    public LLResult getResult(){
        return limelight.getLatestResult();
    }

    public boolean hasTarget(){
        LLResult result = getResult();
        return result != null && result.isValid();
    }

    public boolean hasTarget(LLResult result){
        return result != null && result.isValid();
    }

    /** Horizontal offset of primary target from crosshair (degrees). 0 if no target. */
    public double getTx(){
        LLResult result = getResult();
        if (!hasTarget(result)) return 0;
        return result.getTx();
    }

    /** Vertical offset of primary target from crosshair (degrees). 0 if no target. */
    public double getTy(){
        LLResult result = getResult();
        if (!hasTarget(result)) return 0;
        return result.getTy();
    }

    public List<LLResultTypes.FiducialResult> getFiducialResults(){
        return getResult().getFiducialResults();
    }

    public List<LLResultTypes.FiducialResult> getFiducialResults(LLResult result){
        return result.getFiducialResults();
    }

    /** IDs of all detected April tags this frame. Empty if none. */
    public int[] getTagIDs(){
        LLResult result = getResult();
        if (!hasTarget(result)) return new int[0];

        List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
        if (results.isEmpty()) return new int[0];

        int[] res = new int[results.size()];
        int i = 0;
        for (LLResultTypes.FiducialResult curResult : results){
            res[i] = curResult.getFiducialId();
        }
        return res;
    }

    public int[] getTagIDs(List<LLResultTypes.FiducialResult> fiducialResults){
        int[] res = new int[fiducialResults.size()];
        for (int i = 0; i < res.length; i++){
            res[i] = fiducialResults.get(i).getFiducialId();
        }
        return res;
    }

    /** Send current gyro heading to Limelight for improved pose (e.g. MT2). */
    public void updateLimelightHeading(){
        double heading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        limelight.updateRobotOrientation(heading);
    }

    /** Robot pose from Limelight (MT2). Returns null if result invalid. Call updateLimelightHeading() for better accuracy. */
    public Pose3D getBotPoseMT2(){
        LLResult result = getResult();
        updateLimelightHeading();

        if (!result.isValid()) return null;
        return result.getBotpose_MT2();
    }
}
