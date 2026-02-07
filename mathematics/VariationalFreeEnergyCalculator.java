package mathematics;

import belief.Belief;
import gvf.IGVF;
import manifold.ManifoldPoint;
import manifold.RiemannianManifold;
import manifold.TangentVector;
import manifold.endomorphism.Endomorphism;
import solver.Sensor;

public class VariationalFreeEnergyCalculator {
    public static <Pose extends ManifoldPoint<Pose>, Twist extends TangentVector<Pose, Twist>,
            Workspace extends RiemannianManifold<Pose, Twist, Endomorphism<Twist>>>
        double calcPoseFreeEnergy(Sensor<Pose, Twist> sensor, Belief<Pose, Twist, Workspace> belief, Workspace workspace) {
        Twist innovation = workspace.log(belief.muPose, sensor.read());
        double quadraticForm = workspace.quadraticForm(innovation, sensor.covariance());
        double trace = belief.sigma.one.multiply(sensor.inverseCovariance()).trace();
        return quadraticForm + trace;
    }

    public static <Pose extends ManifoldPoint<Pose>, Twist extends TangentVector<Pose, Twist>,
            Workspace extends RiemannianManifold<Pose, Twist, Endomorphism<Twist>>>
        double calcTwistFreeEnergy(IGVF<Pose, Twist, Workspace> path,
                                   Belief<Pose, Twist, Workspace> belief, Workspace workspace) {
        Twist innovation = path.evaluate(belief.muPose).subtract(belief.muTwist);
        double quadraticForm = workspace.quadraticForm(innovation, path.covariance());
        Endomorphism<Twist> covariantDerivative = workspace.covariantDerivative(belief.muPose, path);
        double trace = covariantDerivative
                .multiply(belief.sigma.two)
                .multiply(covariantDerivative)
                .multiply(path.inverseCovariance())
                .trace();
        return quadraticForm + trace;
    }

    public static <Pose extends ManifoldPoint<Pose>, Twist extends TangentVector<Pose, Twist>,
            Workspace extends RiemannianManifold<Pose, Twist, Endomorphism<Twist>>>
        double calcVariationalFreeEnergy(IGVF<Pose, Twist, Workspace> path,
                                         Sensor<Pose, Twist> sensor, Belief<Pose, Twist, Workspace> belief,
                                         Workspace workspace) {
        return calcPoseFreeEnergy(sensor, belief, workspace) + calcTwistFreeEnergy(path, belief, workspace);
    }
}
