package org.firstinspires.ftc.teamcode;

public class TimeHandler {
    public static void waitTimeMs(int ms){
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < ms){
            continue;
        }
    }
}
