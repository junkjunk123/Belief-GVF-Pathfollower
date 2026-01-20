package manifold;

import java.util.function.Function;

public interface GuidingVectorField<
        P extends ManifoldPoint<P>,
        V extends TangentVector<P, V>,
        M extends Matrix<V>,
        Manifold extends RiemannianManifold<P, V, M>> extends Function<P, V> {
}
