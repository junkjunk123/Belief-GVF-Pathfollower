package gvf;

import manifold.ManifoldPoint;
import manifold.RiemannianManifold;
import manifold.TangentVector;
import manifold.endomorphism.Endomorphism;

public interface VectorField<
        P extends ManifoldPoint<P>,
        V extends TangentVector<P, V>,
        M extends Endomorphism<V>,
        Manifold extends RiemannianManifold<P, V, M>> {
    V evaluate(P pose);
}
