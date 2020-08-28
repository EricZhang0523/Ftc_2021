package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class BuildRobot extends LinearOpMode {
    DcMotor leftFront, leftBack;
    DcMotor rightFront, rightBack;
    DcMotor intakeLeft, intakeRight;
    DcMotor armDegree, armDistance;
    Servo servoArm, intakeServo;
    private DistanceSensor sensorRange;
    boolean mode = false, prev_x, current_x;
    boolean mode2 = false, prev_y, current_y;
    boolean mode3 = false, prev_b, current_b;
    boolean mode4 = false, prev_a, current_a;
    boolean running = true;

    PID armDegreePID, armDistancePID;

    public void ultra() {
        while (running) {
            double testSpeed = 0.1;
            if (sensorRange.getDistance(DistanceUnit.CM) > 16.5) {
                leftFront.setPower(-testSpeed);
                leftBack.setPower(-testSpeed);
                rightFront.setPower(testSpeed);
                rightBack.setPower(testSpeed);
            } else if (sensorRange.getDistance(DistanceUnit.CM) < 15.5) {
                leftFront.setPower(testSpeed);
                leftBack.setPower(testSpeed);
                rightFront.setPower(-testSpeed);
                rightBack.setPower(-testSpeed);
            } else {
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setPower(0);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setPower(0);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setPower(0);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setPower(0);
                running = false;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackMotor");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeDcMotorRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeDcMotorLeft");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackMotor");
        armDistance = hardwareMap.get(DcMotor.class, "armDis");
        armDegree = hardwareMap.get(DcMotor.class, "armDg");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        armDegreePID = new PID(-2.0f);
        armDistancePID = new PID(2.0f);
        armDegreePID.setMaximumOut(1);
        armDistancePID.setMaximumOut(1);

        intakeServo.scaleRange(0, 300);
        servoArm.scaleRange(0, 300);

        while (true) {
            intakeLeft.setPower(100);
            intakeRight.setPower(-100);
            current_x = gamepad1.x;
            current_y = gamepad1.y;
            current_b = gamepad1.b;
            current_a = gamepad1.a;

            if (current_x != prev_x && current_x) {
                mode = !mode;
            }
            if (current_y != prev_y && current_y) {
                mode2 = !mode2;
            }
            if (current_b != prev_b && current_b) {
                mode3 = !mode3;
            }
            if (current_a != prev_a && current_a) {
                mode4 = !mode4;
            }

            if (mode) {
                float l_y = -this.gamepad1.left_stick_y;
                float l_x = this.gamepad1.left_stick_x;
                float r_x = this.gamepad1.right_stick_x;
                leftFront.setPower(l_x + l_y + r_x);
                leftBack.setPower(-l_x + l_y - r_x);
                rightFront.setPower(l_x - l_y + r_x);
                rightBack.setPower(-l_x - l_y - r_x);
            } else {
                float l_y = gamepad1.left_stick_y;
                float r_y = gamepad1.right_stick_y;
                leftFront.setPower(-100 * l_y);
                leftBack.setPower(-100 * l_y);
                rightFront.setPower(100 * r_y);
                rightBack.setPower(100 * r_y);
            }

            if (mode2) {
                armDistancePID.updateTarget(0);
                armDegreePID.updateTarget(0);
            }
            if (mode3) {
                if (gamepad1.dpad_down) {
                    armDistancePID.updateTarget(-179);
                    armDegreePID.updateTarget(-200);
                    armDistance.setPower(0);
                    armDegree.setPower(0);
                }
                if (gamepad1.dpad_left) {
                    ultra();
                    armDistancePID.updateTarget(-206);
                    armDegreePID.updateTarget(2858);
                }
                if (gamepad1.dpad_up) {
                    ultra();
                    armDistancePID.updateTarget(-313);
                    armDegreePID.updateTarget(2614);
                }
                if (gamepad1.dpad_right) {
                    ultra();
                    armDistancePID.updateTarget(-469);
                    armDegreePID.updateTarget(2382);
                }
            }else {
                if (gamepad1.dpad_down) {
                    ultra();
                    armDistancePID.updateTarget(-666);
                    armDegreePID.updateTarget(2223);
                }
                if (gamepad1.dpad_left) {
                    ultra();
                    armDistancePID.updateTarget(-9312088);
                    armDegreePID.updateTarget(2088);
                }
                if (gamepad1.dpad_up) {
                    ultra();
                    armDistancePID.updateTarget(-1156);
                    armDegreePID.updateTarget(1992);
                }
                if (gamepad1.dpad_right) {
                    while (running) {
                        double testSpeed = 0.5;
                        if (sensorRange.getDistance(DistanceUnit.CM) > 1.5) {
                            leftFront.setPower(-testSpeed);
                            leftBack.setPower(-testSpeed);
                            rightFront.setPower(testSpeed);
                            rightBack.setPower(testSpeed);
                        } else if (sensorRange.getDistance(DistanceUnit.CM) < 0.5) {
                            leftFront.setPower(testSpeed);
                            leftBack.setPower(testSpeed);
                            rightFront.setPower(-testSpeed);
                            rightBack.setPower(-testSpeed);
                        } else {
                            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            leftFront.setPower(0);
                            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            rightFront.setPower(0);
                            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            leftBack.setPower(0);
                            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            rightBack.setPower(0);
                            running = false;
                        }
                    }
                    armDistancePID.updateTarget(-1251);
                    armDegreePID.updateTarget(1764);
                }
            }
            if (mode4){

            }else{

            }
            if (gamepad1.right_stick_y == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0) {
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setPower(0);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setPower(0);
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBack.setPower(0);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setPower(0);
            }

            telemetry.addData("arm Dg: ", armDegree.getCurrentPosition());
            telemetry.addData("arm Dis: ", armDistance.getCurrentPosition());
            telemetry.addData("intakeServo: ", intakeServo.getPosition());
            telemetry.addData("servoArm: ", servoArm.getPosition());
            telemetry.update();
            armDegreePID.calculate(armDegree.getCurrentPosition());
            armDistancePID.calculate(armDistance.getCurrentPosition());
            armDegree.setPower(armDegreePID.getOutput());
            armDistance.setPower(armDegreePID.getOutput());
            prev_x = current_x;
            prev_y = current_y;
            prev_b = current_b;
            prev_a = current_a;
        }
    }
}