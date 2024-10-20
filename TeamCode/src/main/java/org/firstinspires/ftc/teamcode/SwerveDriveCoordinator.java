package org.firstinspires.ftc.teamcode;

public class SwerveDriveCoordinator {
    org.firstinspires.ftc.robotcore.external.Telemetry Telemetry;
    private SwerveDriveWheel LFWheel, LRWheel, RFWheel, RRWheel;

    public SwerveDriveCoordinator(
            org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
            SwerveDriveWheel lfWheel, SwerveDriveWheel lrWheel, SwerveDriveWheel rfWheel, SwerveDriveWheel rrWheel){
        Telemetry = telemetry;
        LFWheel = lfWheel;
        LRWheel = lrWheel;
        RFWheel = rfWheel;
        RRWheel = rrWheel;
    }
    public void drive(double vx, double vy, double vr) {
        Telemetry.addData("Velocity X", vx);
        Telemetry.addData("Velocity Y", vy);
        Telemetry.addData("Angular Velocity", vr);

        double lfX = vx + vr;
        double lfY = vy + vr;
        // Note: Inverting the Y axis provided to atan2 makes it positive in the
        // clockwise rather than counterclockwise direction.
        double lfDir = 90 + Math.atan2(-lfY, lfX) * 180 / Math.PI;
        double lfPow = Math.sqrt(lfX*lfX + lfY*lfY);

        double lrX = vx - vr;
        double lrY = vy + vr;
        double lrDir = 90 + Math.atan2(-lrY, lrX) * 180 / Math.PI;
        double lrPow = Math.sqrt(lrX*lrX + lrY*lrY);

        double rfX = vx + vr;
        double rfY = vy - vr;
        double rfDir = 90 + Math.atan2(-rfY, rfX) * 180 / Math.PI;
        double rfPow = Math.sqrt(rfX*rfX + rfY*rfY);

        double rrX = vx - vr;
        double rrY = vy - vr;
        double rrDir = 90 + Math.atan2(-rrY, rrX) * 180 / Math.PI;
        double rrPow = Math.sqrt(rrX*rrX + rrY*rrY);

        // Scale motor powers so the fastest one is 1.0 at most. We can
        // use a simple maximum over the power values because they are
        // the always-positive magnitude of each motor's direction vector.
        double powerScale = Math.max(Math.max(lfPow, lrPow), Math.max(rfPow, rrPow));
        if (powerScale > 1.0) {
            lfPow /= powerScale;
            lrPow /= powerScale;
            rfPow /= powerScale;
            rrPow /= powerScale;
        }

        LFWheel.drive(lfDir, lfPow);
        LRWheel.drive(lrDir, lrPow);
        RFWheel.drive(rfDir, rfPow);
        RRWheel.drive(rrDir, rrPow);
    }
}
