package org.firstinspires.ftc.teamcode;

public class TimeHandler {
    long startTime;
    long timeToWait;
    public TimeHandler(long timeToWait){
        startTime = System.currentTimeMillis();
        this.timeToWait = timeToWait;
    }

    public boolean isTimeExpired(){
        return currentTimeElapsed() > timeToWait;
    }

    public long currentTimeElapsed(){
        return System.currentTimeMillis() - startTime;
    }

    public boolean timeInRange(int beginning, int end){
        return beginning < currentTimeElapsed() && end > currentTimeElapsed();
    }


}
