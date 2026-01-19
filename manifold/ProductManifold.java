package manifold;

import java.util.function.BiFunction;
import java.util.function.Function;

public class ProductManifold<
        M1P extends ManifoldPoint<M1P>,
        M1V extends TangentVector<M1P, M1V>,
        M2P extends ManifoldPoint<M2P>,
        M2V extends TangentVector<M2P, M2V>,
        M1 extends RiemannianManifold<M1P, M1V>,
        M2 extends RiemannianManifold<M2P, M2V>>
        implements RiemannianManifold<
        ProductManifold<M1P, M1V, M2P, M2V, M1, M2>.ProductPoint,
        ProductManifold<M1P, M1V, M2P, M2V, M1, M2>.ProductTangentVector> {

    public final M1 firstManifold;
    public final M2 secondManifold;

    public ProductManifold(M1 firstManifold, M2 secondManifold) {
        this.firstManifold = firstManifold;
        this.secondManifold = secondManifold;
    }

    public static <M1P extends ManifoldPoint<M1P>,
     M1V extends TangentVector<M1P, M1V>,
     M2P extends ManifoldPoint<M2P>,
     M2V extends TangentVector<M2P, M2V>,
     M1 extends RiemannianManifold<M1P, M1V>,
     M2 extends RiemannianManifold<M2P, M2V>> ProductManifold<M1P, M1V, M2P, M2V, M1, M2> of(M1 firstManifold, M2 secondManifold) {
        return new ProductManifold<M1P, M1V, M2P, M2V, M1, M2>(firstManifold, secondManifold);
    };

    @Override
    public BiFunction<ProductTangentVector, ProductTangentVector, Double> metric(ProductPoint point) {
        return (u, v) -> {;
            double firstMetric = firstManifold.metric(point.points.one()).apply(u.vectors.one(), v.vectors.one());
            double secondMetric = secondManifold.metric(point.points.two()).apply(u.vectors.two(), v.vectors.two());
            return firstMetric + secondMetric;
        };
    }

    @Override
    public ProductPoint exp(ProductPoint point, ProductTangentVector vector) {
        M1P p1 = firstManifold.exp(
                point.points.one(),
                vector.vectors.one()
        );

        M2P p2 = secondManifold.exp(
                point.points.two(),
                vector.vectors.two()
        );

        return new ProductPoint(p1, p2);
    }

    @Override
    public ProductTangentVector log(ProductPoint start, ProductPoint end) {
        M1V v1 = firstManifold.log(
                start.points.one(),
                end.points.one()
        );

        M2V v2 = secondManifold.log(
                start.points.two(),
                end.points.two()
        );

        return new ProductTangentVector(v1, v2);
    }

    @Override
    public ProductTangentVector gradient(ProductPoint point, Function<ProductPoint, Double> function) {
        Function<M1P, Double> f1 =
                p1 -> function.apply(new ProductPoint(p1, point.points.two()));

        Function<M2P, Double> f2 =
                p2 -> function.apply(new ProductPoint(point.points.one(), p2));

        M1V g1 = firstManifold.gradient(point.points.one(), f1);
        M2V g2 = secondManifold.gradient(point.points.two(), f2);

        return new ProductTangentVector(g1, g2);
    }

    public class ProductPoint implements ManifoldPoint<ProductPoint> {
        private final Pair<M1P, M2P> points;

        public ProductPoint(M1P pointOne, M2P pointTwo) {
            this.points = new Pair<>(pointOne, pointTwo);
        }

        @Override
        public ProductPoint copy() {
            return new ProductPoint(points.one().copy(), points.two().copy());
        }
    }

    public class ProductTangentVector implements TangentVector<ProductPoint, ProductTangentVector> {
        private final Pair<M1V, M2V> vectors;

        public ProductTangentVector(M1V vectorOne, M2V vectorTwo) {
            this.vectors = new Pair<>(vectorOne, vectorTwo);
        }

        @Override
        public ProductPoint basePoint() {
            return new ProductPoint(vectors.one().basePoint(), vectors.two().basePoint());
        }

        @Override
        public ProductTangentVector add(ProductTangentVector other) {
            M1V newVectorOne = vectors.one().add(other.vectors.one());
            M2V newVectorTwo = vectors.two().add(other.vectors.two());
            return new ProductTangentVector(newVectorOne, newVectorTwo);
        }

        @Override
        public ProductTangentVector scale(double alpha) {
            M1V newVectorOne = vectors.one().scale(alpha);
            M2V newVectorTwo = vectors.two().scale(alpha);
            return new ProductTangentVector(newVectorOne, newVectorTwo);
        }
    }
}
