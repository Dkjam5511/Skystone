package org.firstinspires.ftc.teamcode;

public class PIDTest{

    //Figure out how to make it all in PID, vector should be combined anyway
    private double uMagnitude; //Not really Magnitude, can be negative. To be Psuedo Stick Input
    private double uError;
    private double rMagnitude; //Not really Magnitude, can be negative. To be Psuedo Stick Input
    private double rError;
    private double[] vector;
    private boolean reached;
    private double P; private double I; private double D;
    private double uLastError;
    private double rLastError;
    private double uLastI;
    private double rLastI;
    private double lastTime;
    private double uIntegrater;
    private double rIntegrater;

    public PIDTest(double up, double right, double p, double i,  double d){

        uMagnitude = 0;
        rMagnitude = 0;
        uError = up;
        rError = right;
        vector = new double[2];
        reached = false;
        P = p; I = i; D = d;
        uLastError = 0;
        rLastError = 0;
        uLastI = 0;
        rLastI = 0;
        lastTime = System.currentTimeMillis();
        uIntegrater = 0;
        rIntegrater = 0;
    }
    public double[] getCurrentVector(){return vector;}
    public boolean getReached(){return reached;}
    public double[] calcErrors(double u, double r) {
        uError = u;
        rError = r;
        return calcMagnitude(uError, rError);
    }
    private double[] calcMagnitude(double uE, double rE){
        //calc with error to get speed, do this in loop with calc and putting in values.
        //Find a way to get reached = true when mags = 0, then can end loop
        //Copy in Drew's Mecanum calcs

        //Reset Values from last run-through
        uMagnitude = 0;
        uIntegrater = 0;
        rMagnitude = 0;
        rIntegrater = 0;

        uMagnitude += uE * P;
        uMagnitude += ((System.currentTimeMillis() - lastTime) * (uE - uLastError)) * D;
        //Not adding the integral just yet, need to check if windup occured
        uIntegrater = ((System.currentTimeMillis() - lastTime) * (uE * uLastError/2) + uLastI) * I;
        if (uMagnitude + uIntegrater > 1 && uE > 0 && uIntegrater > 0){
            //Integral is winding up in the positive direction, so it is getting clamped
            uMagnitude = 1;
        }
        else if (uMagnitude + uIntegrater < -1 && uE < 0 && uIntegrater < 0){
            //Integral is winding up in the negative direction, so it is getting clamped
            uMagnitude = -1;
        }
        else {
            //Integral is not winding up, so we can safely use the value and add it in!
            uMagnitude += uIntegrater;
        }

        rMagnitude += rE * P;
        rMagnitude += ((System.currentTimeMillis() - lastTime) * (rE - rLastError)) * D;
        rIntegrater += ((System.currentTimeMillis() - lastTime) * (rE * rLastError/2) + rLastI) * I;
        if (rMagnitude + rIntegrater > 1 && rE > 0 && rIntegrater > 0){
            //Integral is winding up in the positive direction, so it is getting clamped
            rMagnitude = 1;
        }
        else if (rMagnitude + rIntegrater < -1 && rE < 0 && rIntegrater < 0){
            //Integral is winding up in the negative direction, so it is getting clamped
            rMagnitude = -1;
        }
        else {
            //Integral is not winding up, so we can safely use the value and add it in!
            rMagnitude += rIntegrater;
        }
        uLastI = uIntegrater;
        rLastI = rIntegrater;
        lastTime = System.currentTimeMillis();
        uLastError = uE;
        rLastError = rE;

        vector[0] = uMagnitude;
        vector[1] = rMagnitude;

        return vector;


    }
}