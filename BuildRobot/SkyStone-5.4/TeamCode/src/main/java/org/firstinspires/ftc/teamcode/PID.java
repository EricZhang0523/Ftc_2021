package org.firstinspires.ftc.teamcode;

public class PID {

    public float Kp, Ki, Kd;
    public float error, output, target;
    private float accum, lastErr;
    private float tolerance = 0.05f;
    public float maximumOut = 100;

    /** Initializers **/
    public PID(float Kp){
        this.Kp = Kp;
    }
    public PID(float Kp, float Kd){
        this.Kp = Kp;
        this.Kd = Kd;
    }
    public PID(float Kp, float Kd, float Ki){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
    }


    /** Getters **/
    public float getKp(){ return this.Kp; }
    public float getKd(){ return this.Kd; }
    public float getKi(){ return this.Ki; }
    public float[] getParameters(){ return new float[]{this.Kp, this.Kd, this.Ki}; }
    public float getError(){ return this.error; }
    public float getOutput(){ return this.output; }
    public float getMaximumOut(){ return this.maximumOut; }

    /** Setters **/
    public void setKp(float Kp){ this.Kp = Kp; }
    public void setKd(float Kd){ this.Kd = Kd; }
    public void setKi(float Ki){ this.Ki = Ki; }
    public void setParameters(float[] parameters){
        if(parameters != null) {
            this.Kp = parameters[0];
            this.Kd = parameters[1];
            this.Ki = parameters[2];
        }
    }
    public void setMaximumOut(float max){ this.maximumOut = max; }
    public void setTolerance(float tolerance){ this.tolerance = tolerance; }

    /** Functionality **/
    public boolean calculate(float input){

        this.error = this.target - input;

        this.output = this.error * this.Kp;
        this.output += (this.error - this.lastErr) * this.Kd;
        this.output += (this.error + this.accum) * this.Ki;

        this.output = (Math.abs(this.output) > this.maximumOut)?this.maximumOut:this.output;
        this.lastErr = this.error;
        this.accum += this.error;

        if((this.target * this.tolerance) <= this.error) {
            this.output = 0;
            return true;
        }
        return false;
    }

    public void updateTarget(float target){
        this.target = target;
        this.output = 0;
        this.error = 0;
        this.accum = 0;
        this.lastErr = 0;
    }
}
