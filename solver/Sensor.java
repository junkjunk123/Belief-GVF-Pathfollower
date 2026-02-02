package solver;

import manifold.ManifoldPoint;
import manifold.TangentVector;
import manifold.endomorphism.Endomorphism;

public interface Sensor<Pose extends ManifoldPoint<Pose>, Vector extends TangentVector<Pose, Vector>> {
    Endomorphism<Vector> covariance();
    default Endomorphism<Vector> inverseCovariance() {
        //override to cache if wanted
        return covariance().inverse();
    };
    Pose read();
}
