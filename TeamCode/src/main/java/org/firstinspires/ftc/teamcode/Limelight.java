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

public class Limelight extends Subsystem{
    private Limelight3A limelight;
    private IntegratingGyroscope gyro;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
    }

    protected void init(){
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        gyro = hardwareMap.get(NavxMicroNavigationSensor.class, navxSensorName);
        limelight.setPollRateHz(limelightHzRate);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    private LLResult getResult(){
        return limelight.getLatestResult();
    }

    public boolean hasTarget(){
        LLResult result = getResult();
        return result != null && result.isValid();
    }

    public boolean hasTarget(LLResult result){
        return result != null && result.isValid();
    }

    public double getTx(){
        LLResult result = getResult();
        if (!hasTarget(result)) return 0;
        return result.getTx();
    }

    public double getTy(){
        LLResult result = getResult();
        if (!hasTarget(result)) return 0;
        return result.getTy();
    }

    public int getTagID(){
        LLResult result = getResult();
        if (!hasTarget(result)) return -1;

        List<LLResultTypes.DetectorResult> results = result.getDetectorResults();
        if (results.isEmpty()) return -1;

        return results.get(0).getClassId();
    }

    public void updateLimelightHeading(){
        double heading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        limelight.updateRobotOrientation(heading);
    }

    public Pose3D getBotPoseMT2(){
        LLResult result = getResult();
        double heading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        limelight.updateRobotOrientation(heading);

        if (result.isValid()) return null;
        return result.getBotpose_MT2();
    }


}
