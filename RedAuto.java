package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import static java.lang.Math.*;

@Autonomous(name = "Red Auto")
public class RedAuto extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    
    private DcMotor conveyer;
    private DcMotor intake;
    private DcMotor shooterl;
    private DcMotor shooterr;
    
    private Pose2D pos;
    
    private Pose2D shootPose = new Pose2D(DistanceUnit.MM,
            1269,
            33.5,
            AngleUnit.RADIANS,
            0);
            
    private double shootPower = 0.5;
    
    double max = 1;
    
    double turnKp = 1;
    double turnKd = 0;
    double strafeRatio = 1.2;
    double driveKp = 0.007;
    double driveKd = 0.001;
    
    double x;
    double y;
    double rx;

    // just logging align values
    double Vx = 0;
    double Vy = 0;
    
    GoBildaPinpointDriver odo;
    
    double time = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        
        odo.setOffsets(12, -156, DistanceUnit.MM);
        
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        conveyer = hardwareMap.get(DcMotor.class, "innerintake");
        intake = hardwareMap.get(DcMotor.class, "outerintake");
        shooterl = hardwareMap.get(DcMotor.class, "frontleftshooter");
        shooterr = hardwareMap.get(DcMotor.class, "frontrightshooter");
        shooterr.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        conveyer.setDirection(DcMotor.Direction.REVERSE);

        odo.resetPosAndIMU();
        
        waitForStart();
        telemetry.addData("Waiting", "Ready?");
        telemetry.update();

        // auto starts here
        shooterr.setPower(shootPower);
        shooterl.setPower(shootPower);
        Thread.sleep(500);
        
        aligning(shootPose);
        Thread.sleep(500);
        
        conveyer.setPower(1);
        Thread.sleep(1500);
        
        aligning(new Pose2D(DistanceUnit.MM, 1432, 38.736, AngleUnit.RADIANS, -2.414 + 3 * Math.PI / 2));
        Thread.sleep(500);
        
        max = 0.45;
        intake.setPower(1);
        conveyer.setPower(1);
        aligning(new Pose2D(DistanceUnit.MM, 658.25, 739.7, AngleUnit.RADIANS, -2.414 + 3 * Math.PI / 2));
        Thread.sleep(500);
        intake.setPower(0);
        conveyer.setPower(0);
        max = 1;
        
        aligning(shootPose);
        Thread.sleep(500);
        
        conveyer.setPower(1);
        Thread.sleep(1500);
        
        aligning(new Pose2D(DistanceUnit.MM, 755.173, 526.332, AngleUnit.RADIANS, -2.429 + Math.PI));
    }

    public void aligning(Pose2D target) {
        odo.update();
        pos = odo.getPosition();
        double yError = (target.getY(DistanceUnit.MM) - pos.getY(DistanceUnit.MM));
        double xError = (target.getX(DistanceUnit.MM) - pos.getX(DistanceUnit.MM));
        double distance = hypot(xError, yError);
        double errorVel = (xError * odo.getVelX(DistanceUnit.MM) + yError * odo.getVelY(DistanceUnit.MM)) / distance;
        telemetry.addData("Aligning", "Started");
        telemetry.update();

        while (distance > 20 || !atHeading(target.getHeading(AngleUnit.RADIANS)) || errorVel > 2) {
            odo.update();
            pos = odo.getPosition();
            yError = (target.getY(DistanceUnit.MM) - pos.getY(DistanceUnit.MM));
            xError = (target.getX(DistanceUnit.MM) - pos.getX(DistanceUnit.MM));
            distance = hypot(xError, yError);
            errorVel = (xError * odo.getVelX(DistanceUnit.MM) + yError * odo.getVelY(DistanceUnit.MM)) / distance;
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.RADIANS));
            telemetry.addData("Aligning", "Going");
            telemetry.addData("xError: ", xError);
            telemetry.addData("yError: ", yError);
            telemetry.addData("errorVel: ", errorVel);
            telemetry.addData("Is at distance: ", hypot(xError, yError) <= 20);
            telemetry.addData("Is at angle: ", atHeading(target.getHeading(AngleUnit.RADIANS)));
            telemetry.addData("Position", data);
            telemetry.update();
            align(target);
        }
        telemetry.addData("Aligning", "Finished");
        x = 0;
        y = 0;
        rx = 0;
        drive();
        telemetry.update();
    }


    public void align(Pose2D target) {
        double heading = pos.getHeading(AngleUnit.RADIANS);
        // field relative: x is forward, y is to the right
        double yError = (target.getY(DistanceUnit.MM) - pos.getY(DistanceUnit.MM));
        double xError = (target.getX(DistanceUnit.MM) - pos.getX(DistanceUnit.MM));
        double turnError = AngleUnit.normalizeRadians(target.getHeading(AngleUnit.RADIANS) - heading);

        double distance = hypot(xError, yError);
        double errorVel = (xError * odo.getVelX(DistanceUnit.MM) + yError * odo.getVelY(DistanceUnit.MM)) / distance;

        x = 0;
        y = 0;
        rx = 0;
        
        if (!atHeading(target.getHeading(AngleUnit.RADIANS))) {
            rx = -1 * turnKp * turnError + turnKd * odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        } else if (distance > 10) {
            double setpointScale = Math.min(driveKp * distance - driveKd * errorVel, max / strafeRatio);
            Vx = xError / distance * setpointScale;
            Vy = yError / distance * setpointScale;
            // now converting to robot relative, y is forward
            y = Vx * cos(heading) + Vy * sin(heading);
            x = -1 * strafeRatio * (-Vx * sin(heading) + Vy * cos(heading));
        }
        
        drive();
    }
    
    private boolean atHeading(double heading) {
        return Math.abs(AngleUnit.normalizeRadians(heading - pos.getHeading(AngleUnit.RADIANS))) < 0.1;
    }
    
    private void drive() {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FrontLeft.setPower((y + x + rx)/denominator);
        FrontRight.setPower(((y - x) - rx)/denominator);
        BackLeft.setPower(((y - x) + rx)/denominator);
        BackRight.setPower(((y + x) - rx)/denominator);
    }
}
