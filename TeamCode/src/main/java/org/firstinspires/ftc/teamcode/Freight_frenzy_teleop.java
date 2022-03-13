package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp

public class Freight_frenzy_teleop extends LinearOpMode {
    //
    DcMotor stangafata, dreaptafata, stangaspate, dreaptaspate;

    Servo cutie, control, retragere;

    DcMotor intake1, intake2, extindere, carusel;
    DistanceSensor distanta;

    @Override
    public void runOpMode() {

        hardware part = new hardware(hardwareMap);
        cutie = part.getCutie();
        control = part.getControl();
        retragere = part.getRetragere();

        //sasiu
        stangafata = part.getMotorstangafata();
        dreaptafata = part.getMotordreaptafata();
        stangaspate = part.getMotorstangaspate();
        dreaptaspate = part.getMotordreaptaspate();

        //motoare dc
        intake1 = part.getIntake1();
        intake2 = part.getIntake2();
        extindere = part.getExtindere();
        carusel=part.getCarusel();

        dreaptafata.setDirection(DcMotorSimple.Direction.FORWARD);
        dreaptaspate.setDirection(DcMotorSimple.Direction.FORWARD);
        stangafata.setDirection(DcMotorSimple.Direction.REVERSE);
        stangaspate.setDirection(DcMotorSimple.Direction.REVERSE);

        stangafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extindere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extindere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            cutie.setPosition(0.43);
            //control miscare
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
        /*
            //sus
            control.setPosition(0);
            cutie.setPosition(0.4);
            //slowmode - default-half speed  pressed-full speed

        */
            if (!gamepad1.left_bumper) {
                x /= 2;
                y /= 2;
            }
            if (!gamepad1.right_bumper) {
                turn /= 2;
            }
            mecanum(x, y, turn);


            //gamepad 2 - control brat si intake + carusel

            if (gamepad2.right_trigger > 0) {
                intake1.setPower(gamepad2.right_trigger);
                intake2.setPower(-gamepad2.right_trigger);

            } else if (gamepad2.left_trigger > 0) {
                intake1.setPower(-gamepad2.left_trigger);
                intake2.setPower(gamepad2.left_trigger);

            } else {

                intake1.setPower(0);
                intake2.setPower(0);
            }


            if (gamepad2.dpad_up) {
                control.setPosition(0.3);
                retragere.setPosition(-0.4);
                sleep(500);
                extindere.setTargetPosition(0);
                extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extindere.setPower(0.8);

            } else if (gamepad2.dpad_down) {
                extindere.setTargetPosition(3300);
                extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extindere.setPower(0.8);
                retragere.setPosition(0.45);
                control.setPosition(0.3);
                sleep(2000);
                control.setPosition(0.51);
                telemetry.addData("ajuns","jos");
                telemetry.update();

            }

            if(gamepad2.y){
                control.setPosition(-1);
                //spate full
            }

            if(gamepad2.left_bumper)
                carusel.setPower(-0.85);
            else
                carusel.setPower(0);
        }
    }
    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Turn    is the second double X value which represents how the base should turn
     */
    public void mecanum(double Strafe, double Forward, double Turn) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)) + rightX;
        final double v2 = (r * Math.sin(robotAngle)) - rightX;
        final double v3 = (r * Math.sin(robotAngle)) + rightX;
        final double v4 = (r * Math.cos(robotAngle)) - rightX;

        stangafata.setPower(v1);
        dreaptafata.setPower(v2);
        stangaspate.setPower(v3);
        dreaptaspate.setPower(v4);

    }
}
