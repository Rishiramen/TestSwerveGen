package frc.robot.Constants;

public class ProjectileCalculations {
    private final double dragCoeff = 0.47;

    private final double airDens = 1.21;

    private final double area = Math.PI*(0.005625); // in m

    private final double gravAccel = 9.795; // in m/s^2

    private double pitch; //launch angle

    private final double mass = 0.219; // in kg

    private final double yTarget = 1.44145; // in m


    public ProjectileCalculations(double pitch) {
        this.pitch = pitch;
    }

    public double forceGrav(double weight){
        return weight*gravAccel;
    }

    public double calcInitVertVel(double initVel, double angle){
        return initVel * Math.sin(angle);
    }

    public double calcInitHorizVel(double initVel, double angle){
        return initVel * Math.cos(angle);
    }

    public double forceDrag(double velocity, double area){
        return (0.5) * (dragCoeff) * (airDens) * (area) * Math.pow((velocity),2);
    }

    public double calcHorizAccel(double initVel){
        double startVel = calcInitHorizVel(initVel, pitch);
        if (startVel > 0) {
            double vel = calcHorizVelocity(initVel);
            double val1 = ((-1) * dragCoeff * area * airDens * Math.pow(vel, 2));
            double val2 = 2 * mass;
            return val1 / val2;
        }
        else{
            return 0.0;
        }
    }

    public double calcHorizAccel(double initVel, double t){
        double startVel = calcInitHorizVel(initVel, pitch);
        if (startVel > 0) {
            double vel = calcHorizVelocity(initVel, t);
            double val1 = ((-1) * dragCoeff * area * airDens * Math.pow(vel, 2));
            double val2 = 2 * mass;
            return val1 / val2;
        }
        else{
            return 0.0;
        }
    }

    public double calcVertAccel(double initVel){
        double startVel = calcInitVertVel(initVel, pitch);
        if (startVel > 0) {
            double vel = calcVertVelocity(initVel);
            double val1 = ((-1) * dragCoeff * area * airDens * Math.pow(vel, 2));
            double val2 = 2 * mass;
            return ((-1) * gravAccel) - (val1 / val2);
        }
        else{
            return 0.0;
        }
    }

    public double calcVertAccel(double initVel, double t){
        double startVel = calcInitVertVel(initVel, pitch);
        if (startVel > 0) {
            double vel = calcVertVelocity(initVel, t);
            double val1 = ((-1) * dragCoeff * area * airDens * Math.pow(vel, 2));
            double val2 = 2 * mass;
            return ((-1) * gravAccel) - (val1 / val2);
        }
        else{
            return 0.0;
        }
    }

    public double calcVelTer(){
        double val1 = 2 * mass * gravAccel;
        double val2 = dragCoeff * area * airDens;
        return (Math.pow(Math.abs(val1/val2), 2));
    }

    public double calcAirTime(double initVel){
        double startVel = calcInitVertVel(initVel, pitch);
        if (startVel > 0) {
            double val1 = (calcVelTer()/gravAccel);
            double val2 = Math.atan(startVel/calcVelTer());
            return val1 * val2;
        }
        else{
            return 0.0;
        }
    }

    public double calcVertVelocity(double initVel){
        double startVel = calcInitVertVel(initVel, pitch);
        if (startVel > 0) {
            double t = calcAirTime(initVel);
            double vt = calcVelTer();
            double divVal1 = (startVel) - (vt * Math.tan((t * gravAccel)/vt));
            double divVal2 = (vt) + (startVel * Math.tan((t * gravAccel)/vt));
            return (vt * (divVal1/divVal2));
        }
        else{
            return 0.0;
        }
    }

    public double calcVertVelocity(double initVel, double t){
        double startVel = calcInitVertVel(initVel, pitch);
        if (startVel > 0) {
            double vt = calcVelTer();
            double divVal1 = (startVel) - (vt * Math.tan((t * gravAccel)/vt));
            double divVal2 = (vt) + (startVel * Math.tan((t * gravAccel)/vt));
            return (vt * (divVal1/divVal2));
        }
        else{
            return 0.0;
        }
    }

    public double calcMaxAscent(double initVel){
        double startVel = calcInitVertVel(initVel, pitch);
        if (startVel > 0) {
            double vt = calcVelTer();
            double vertVal1 = (Math.pow(vt, 2)) / (2 * gravAccel);
            double vertVal2 = ((Math.pow(startVel, 2)) + (Math.pow(vt, 2)));
            double vertVal3 = Math.pow(vt, 2);
            return vertVal1 * Math.log(Math.abs(vertVal2 / vertVal3));
        }
        else{
            return 0.0;
        }
    }

    public double calcVertAscent(double initVel){
        double startVel = calcInitVertVel(initVel, pitch);
        if (startVel > 0) {
            double vt = calcVelTer();
            double vel = calcVertVelocity(initVel);
            double vertVal1 = (Math.pow(vt, 2)) / (2 * gravAccel);
            double vertVal2 = ((Math.pow(startVel, 2)) + (Math.pow(vt, 2)));
            double vertVal3 = ((Math.pow(vt,2)) + (Math.pow(vel,2 )));
            return vertVal1 * Math.log(Math.abs(vertVal2/vertVal3));
        }
        else{
            return 0.0;
        }
    }

    public double calcVertAscent(double initVel, double t){
        double startVel = calcInitVertVel(initVel, pitch);
        if (startVel > 0) {
            double vt = calcVelTer();
            double vel = calcVertVelocity(initVel, t);
            double vertVal1 = Math.pow(vt, 2) / (2 * gravAccel);
            double vertVal2 = ((Math.pow(startVel, 2)) + (Math.pow(vt, 2)));
            double vertVal3 = ((Math.pow(vt, 2)) + (Math.pow(vel, 2)));
            return vertVal1 * Math.log(Math.abs(vertVal2/vertVal3));
        }
        else{
            return 0.0;
        }
    }

    public double calcHorizVelocity(double initVel){
        double startVel = calcInitHorizVel(initVel, pitch);
        if (startVel > 0) {
            double vt = calcVelTer();
            double t = calcAirTime(initVel);
            double horVal1 = (Math.pow(vt, 2)) * startVel;
            double horVal2 = (Math.pow(vt, 2)) + (gravAccel * startVel * t);
            return horVal1 / horVal2;
        }
        else {
            return 0.0;
        }
    }

    public double calcHorizVelocity(double initVel, double t){
        double startVel = calcInitHorizVel(initVel, pitch);
        if (startVel > 0) {
            double vt = calcVelTer();
            double horVal1 = (Math.pow(vt, 2)) * startVel;
            double horVal2 = (Math.pow(vt, 2)) + (gravAccel * startVel * t);
            return horVal1 / horVal2;
        }
        else {
            return 0.0;
        }
    }

    public double calcDistance(double initVel){
        double startVel = calcInitHorizVel(initVel, pitch);
        if (startVel > 0) {
            double vt = calcVelTer();
            double t = calcAirTime(initVel);
            double horVal1 = (Math.pow(vt, 2)) / gravAccel;
            double horVal2 = (Math.pow(vt, 2)) + (gravAccel * startVel * t);
            double horVal3 = Math.pow(vt, 2);
            return horVal1 * Math.log(Math.abs(horVal2/horVal3));
        }
        else {
            return 0.0;
        }
    }

    public double calcDistance(double initVel, double t){
        double startVel = calcInitHorizVel(initVel, pitch);
        if (startVel > 0) {
            double vt = calcVelTer();
            double horVal1 = (Math.pow(vt, 2)) / gravAccel;
            double horVal2 = (Math.pow(vt, 2)) + (gravAccel * startVel * t);
            double horVal3 = Math.pow(vt, 2);
            return horVal1 * Math.log(Math.abs(horVal2/horVal3));
        }
        else {
            return 0.0;
        }
    }
// === QUADRATIC-DRAG SHOOTER SOLVER (RK4 + bisection) ===
// Uses your existing: dragCoeff, airDens, area, mass, gravAccel, yTarget

    // Predict horizontal x at the instant the projectile crosses yTarget (descending).
// Returns NaN if it never reaches yTarget (e.g., too low speed/angle).
    private double xAtYTargetQuadratic(double u0) {
        final double k = 0.5 * airDens * dragCoeff * area; // quadratic drag coefficient
        final double g = gravAccel;

        // Initial state
        double x = 0.0, y = 0.0;
        double vx = u0 * Math.cos(pitch);
        double vy = u0 * Math.sin(pitch);

        // Integrate with RK4
        final double dt = 0.002;        // 2 ms step; adjust if needed
        final double tMax = 5.0;        // safety cap (s)
        double prevX = x, prevY = y;

        for (double t = 0.0; t < tMax; t += dt) {
            // Save previous point for crossing interpolation
            prevX = x; prevY = y;

            // RK4 step
            // a(v) = -(k/m) * |v| * v - (0, g)
            java.util.function.BiConsumer<double[], double[]> deriv = (s, ds) -> {
                double sx = s[0], sy = s[1], svx = s[2], svy = s[3];
                double v = Math.hypot(svx, svy);
                double ax = -(k / mass) * v * svx;
                double ay = -(k / mass) * v * svy - g;
                ds[0] = svx;
                ds[1] = svy;
                ds[2] = ax;
                ds[3] = ay;
            };

            double[] s = { x, y, vx, vy };
            double[] k1 = new double[4], k2 = new double[4], k3 = new double[4], k4 = new double[4], tmp = new double[4];

            deriv.accept(s, k1);

            for (int i = 0; i < 4; i++) tmp[i] = s[i] + 0.5 * dt * k1[i];
            deriv.accept(tmp, k2);

            for (int i = 0; i < 4; i++) tmp[i] = s[i] + 0.5 * dt * k2[i];
            deriv.accept(tmp, k3);

            for (int i = 0; i < 4; i++) tmp[i] = s[i] + dt * k3[i];
            deriv.accept(tmp, k4);

            for (int i = 0; i < 4; i++) {
                s[i] = s[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }

            x = s[0]; y = s[1]; vx = s[2]; vy = s[3];

            // If we crossed yTarget on this step while descending, interpolate x
            if (prevY > yTarget && y <= yTarget) {
                double alpha = (prevY - yTarget) / (prevY - y); // linear in y over the small dt
                double xCross = prevX + alpha * (x - prevX);
                return xCross;
            }

            // Early exit if projectile fell well below target height
            if (y < (yTarget - 2.0)) break;
        }

        return Double.NaN;
    }

    // Find initial speed u0 so that xAtYTargetQuadratic(u0, theta) ≈ xDist.
// Returns NaN if not reachable within speed bounds.
    public double initialSpeedForShotQuadratic(double xDist) {
        if (xDist <= 0) return 0.0;

        // Lower/upper bounds for speed (m/s)
        double lo = 0.0;
        // Heuristic upper bound: start from no-drag estimate if feasible, else a modest guess.
        double c = Math.cos(pitch), s = Math.sin(pitch);
        double nodrag = Double.NaN;
        if (Math.abs(c) > 1e-9 && xDist * s - yTarget * c > 0) {
            // No-drag: u0^2 = g x^2 / (2 c^2 (x tanθ - y))
            nodrag = Math.pow((gravAccel * xDist * xDist) / (2.0 * c * c * (xDist * Math.tan(pitch) - yTarget)), 2);
        }
        double hi = Double.isNaN(nodrag) ? 8.0 : Math.max(8.0, nodrag);

        // Expand hi until we can reach xDist at yTarget (or give up)
        for (int i = 0; i < 30; i++) {
            double xHit = xAtYTargetQuadratic(hi);
            if (!Double.isNaN(xHit) && xHit >= xDist) break;
            hi *= 1.6;
            if (hi > 120.0) break; // safety cap; tune for your shooter
        }

        // If still not reachable, signal failure
        double xHi = xAtYTargetQuadratic(hi);
        if (Double.isNaN(xHi) || xHi < xDist) return Double.NaN;

        // Bisection on u0
        for (int it = 0; it < 80; it++) {
            double mid = 0.5 * (lo + hi);
            double xHit = xAtYTargetQuadratic(mid);
            if (!Double.isNaN(xHit) && Math.abs(xHit - xDist) < 1e-4) return mid;
            if (Double.isNaN(xHit) || xHit < xDist) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

//    public double xToInitVel(double xDist) {
//        double vt = calcVelTer();
//        double u0 = 5.0;
//        double tol = 1e-6;
//        int maxIter = 100;
//
//        for (int i = 0; i < maxIter; i++) {
//            double v0y = u0 * Math.tan(pitch);
//            double denom = 2.0 * Math.atan(v0y / vt);
//
//
//            if (Math.abs(denom) < 1e-9) return 0.0;
//
//
//            double newU0 = (vt / denom) * (Math.exp((gravAccel * xDist) / (vt * vt)) - 1.0);
//
//            if (Math.abs(newU0 - u0) < tol) {
//                u0 = newU0;
//                break;
//            }
//
//            u0 = 0.5 * (u0 + newU0);
//        }
//
//        return u0;
//    }
//    public double xToInitVel(double xDist, double pitchAvg) {
//
//        double vt = calcVelTer();
//
//
//        double u0 = 5.0;
//        double tol = 1e-6;
//        int maxIter = 100;
//
//        for (int i = 0; i < maxIter; i++) {
//            double v0y = u0 * Math.tan(pitchAvg);
//            double denom = 2.0 * Math.atan(v0y / vt);
//
//
//            if (Math.abs(denom) < 1e-9) return 0.0;
//
//
//            double newU0 = (vt / denom) * (Math.exp((gravAccel * xDist) / (vt * vt)) - 1.0);
//
//            if (Math.abs(newU0 - u0) < tol) {
//                u0 = newU0;
//                break;
//            }
//
//            u0 = 0.5 * (u0 + newU0);
//        }
//
//        return u0;
//    }
}