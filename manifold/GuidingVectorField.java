package manifold;

import java.util.function.Function;

public interface GuidingVectorField<
        P extends ManifoldPoint<P>,
        V extends TangentVector<P, V>,
        M extends BilinearForm<V>,
        Manifold extends RiemannianManifold<P, V, M>> extends Function<P, V> {
}
