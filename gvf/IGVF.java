package gvf;

import manifold.ManifoldPoint;
import manifold.RiemannianManifold;
import manifold.TangentVector;
import manifold.endomorphism.Endomorphism;

import java.util.function.Function;

public interface IGVF<
        P extends ManifoldPoint<P>,
        V extends TangentVector<P, V>,
        M extends Endomorphism<V>,
        Manifold extends RiemannianManifold<P, V, M>> extends Function<P, V> {
    V evaluate(P pose);
    M covariance();

    /**
     * Use the canonical Levi-Civita connection here.
     */
    V covariantDerivative();
}
