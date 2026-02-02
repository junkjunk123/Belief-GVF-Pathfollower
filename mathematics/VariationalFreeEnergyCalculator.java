package mathematics;

import belief.Belief;
import gvf.GVFPath;
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
        double calcTwistFreeEnergy(GVFPath<Workspace, Pose, Twist, Endomorphism<Twist>> path,
                                   Belief<Pose, Twist, Workspace> belief, Workspace workspace) {
        Twist innovation = path.evaluate(belief.muPose).subtract(belief.muTwist);
        double quadraticForm = workspace.quadraticForm(innovation, path.covariance());
        double trace = path.covariance().multiply(belief.sigma.two).multiply(path.covariance()).trace();
        return quadraticForm + trace;
    }

    public static <Pose extends ManifoldPoint<Pose>, Twist extends TangentVector<Pose, Twist>,
            Workspace extends RiemannianManifold<Pose, Twist, Endomorphism<Twist>>>
        double calcVariationalFreeEnergy(GVFPath<Workspace, Pose, Twist, Endomorphism<Twist>> path,
                                         Sensor<Pose, Twist> sensor, Belief<Pose, Twist, Workspace> belief,
                                         Workspace workspace) {
        return calcPoseFreeEnergy(sensor, belief, workspace) + calcTwistFreeEnergy(path, belief, workspace);
    }
}
