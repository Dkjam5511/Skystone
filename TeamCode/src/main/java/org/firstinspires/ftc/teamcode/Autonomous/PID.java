package org.firstinspires.ftc.teamcode.Autonomous;

public class PID {

    //Figure out how to make it all in PID, vector should be combined anyway
    private double yMagnitude; //Not really Magnitude, can be negative. To be Psuedo Stick Input
    private double yError;
    private double xMagnitude; //Not really Magnitude, can be negative. To be Psuedo Stick Input
    private double xError;
    private double[] vector;
    private boolean reached;
    private double P; private double I; private double D;
    private double yLastError;
    private double xLastError;
    private double xLastI;
    private double yLastI;
    private double lastTime;
    private double yIntegrater;
    private double xIntegrater;

    public PID(double up, double right, double p, double i, double d){

        yMagnitude = 0;
        xMagnitude = 0;
        yError = up;
        xError = right;
        vector = new double[2];
        reached = false;
        P = p; I = i; D = d;
        yLastError = 0;
        xLastError = 0;
        yLastI = 0;
        xLastI = 0;
        lastTime = System.currentTimeMillis();
        yIntegrater = 0;
        xIntegrater = 0;
    }
    public double[] getCurrentVector(){return vector;}
    public boolean getReached(){return reached;}
    public double[] calcErrors(double x, double y) {
        yError = y;
        xError = x;
        return calcMagnitude(xError, yError);
    }
    private double[] calcMagnitude(double xE, double yE){
        //calc with error to get speed, do this in loop with calc and putting in values.
        //Find a way to get reached = true when mags = 0, then can end loop
        //Copy in Drew's Mecanum calcs

        //Reset Values from last run-through
        yMagnitude = 0;
        yIntegrater = 0;
        xMagnitude = 0;
        xIntegrater = 0;

        yMagnitude += yE * P;
        yMagnitude += ((System.currentTimeMillis() - lastTime) * (yE - yLastError)) * D;
        //Not adding the integral just yet, need to check if windup occured
        yIntegrater = ((System.currentTimeMillis() - lastTime) * (yE * yLastError/2) + yLastI) * I;
        if (yMagnitude + yIntegrater > 1 && yE > 0 && yIntegrater > 0){
            //Integral is winding up in the positive direction, so it is getting clamped
            yMagnitude = 1;
        }
        else if (yMagnitude + yIntegrater < -1 && yE < 0 && yIntegrater < 0){
            //Integral is winding up in the negative direction, so it is getting clamped
            yMagnitude = -1;
        }
        else {
            //Integral is not winding up, so we can safely use the value and add it in!
            yMagnitude += yIntegrater;
        }

        xMagnitude += xE * P;
        xMagnitude += ((System.currentTimeMillis() - lastTime) * (xE - xLastError)) * D;
        xIntegrater += ((System.currentTimeMillis() - lastTime) * (xE * xLastError/2) + xLastI) * I;
        if (xMagnitude + xIntegrater > 1 && xE > 0 && xIntegrater > 0){
            //Integral is winding up in the positive direction, so it is getting clamped
            xMagnitude = 1;
        }
        else if (xMagnitude + xIntegrater < -1 && xE < 0 && xIntegrater < 0){
            //Integral is winding up in the negative direction, so it is getting clamped
            xMagnitude = -1;
        }
        else {
            //Integral is not winding up, so we can safely use the value and add it in!
            xMagnitude += xIntegrater;
        }
        xLastI = xIntegrater;
        xLastI = xIntegrater;
        lastTime = System.currentTimeMillis();
        yLastError = yE;
        xLastError = xE;

        vector[0] = xMagnitude;
        vector[1] = yMagnitude;

        return vector;
    }
}