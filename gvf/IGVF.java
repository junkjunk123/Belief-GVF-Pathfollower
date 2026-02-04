package gvf;

import manifold.ManifoldPoint;
import manifold.RiemannianManifold;
import manifold.TangentVector;
import manifold.endomorphism.Endomorphism;

import java.util.function.Function;

public interface IGVF<
        P extends ManifoldPoint<P>,
        V extends TangentVector<P, V>,
        Manifold extends RiemannianManifold<P, V, Endomorphism<V>>> extends VectorField <P, V, Endomorphism<V>, Manifold> {
    Endomorphism<V> covariance();

    default Endomorphism<V> inverseCovariance() {
        //Override here if caching wanted
        return covariance().inverse();
    }
}
