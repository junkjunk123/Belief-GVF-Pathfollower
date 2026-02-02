package manifold;

import manifold.endomorphism.Endomorphism;
import mathematics.DifferentiableScalarFunction;
import util.Pair;

import java.util.function.BiFunction;

public class ProductManifold<
        M1P extends ManifoldPoint<M1P>,
        M1V extends TangentVector<M1P, M1V>,
        M1M extends Endomorphism<M1V>,
        M2P extends ManifoldPoint<M2P>,
        M2V extends TangentVector<M2P, M2V>,
        M2M extends Endomorphism<M2V>,
        M1 extends RiemannianManifold<M1P, M1V, M1M>,
        M2 extends RiemannianManifold<M2P, M2V, M2M>>
        implements RiemannianManifold<
        ProductManifold.ProductPoint<M1P, M2P>,
        ProductManifold.ProductTangentVector<M1P, M1V, M2P, M2V>,
        Endomorphism<ProductManifold.ProductTangentVector<M1P, M1V, M2P, M2V>>> {

    public final M1 firstManifold;
    public final M2 secondManifold;

    public ProductManifold(M1 firstManifold, M2 secondManifold) {
        this.firstManifold = firstManifold;
        this.secondManifold = secondManifold;
    }

    public static <
            M1P extends ManifoldPoint<M1P>,
            M1V extends TangentVector<M1P, M1V>,
            M1M extends Endomorphism<M1V>,
            M2P extends ManifoldPoint<M2P>,
            M2V extends TangentVector<M2P, M2V>,
            M2M extends Endomorphism<M2V>,
            M1 extends RiemannianManifold<M1P, M1V, M1M>,
            M2 extends RiemannianManifold<M2P, M2V, M2M>>
    ProductManifold<M1P, M1V, M1M, M2P, M2V, M2M, M1, M2> of(M1 firstManifold, M2 secondManifold) {
        return new ProductManifold<>(firstManifold, secondManifold);
    };

    @Override
    public BiFunction<ProductTangentVector<M1P, M1V, M2P, M2V>, ProductTangentVector<M1P, M1V, M2P, M2V>, Double>
        metric(ProductPoint<M1P, M2P> point) {
        return (u, v) -> {;
            double firstMetric = firstManifold.metric(point.points.one()).apply(u.vectors.one(), v.vectors.one());
            double secondMetric = secondManifold.metric(point.points.two()).apply(u.vectors.two(), v.vectors.two());
            return firstMetric + secondMetric;
        };
    }

    @Override
    public int dim() {
        return firstManifold.dim() + secondManifold.dim();
    }

    @Override
    public ProductPoint<M1P, M2P> exp(ProductPoint<M1P, M2P> point, ProductTangentVector<M1P, M1V, M2P, M2V> vector) {
        M1P p1 = firstManifold.exp(
                point.points.one(),
                vector.vectors.one()
        );

        M2P p2 = secondManifold.exp(
                point.points.two(),
                vector.vectors.two()
        );

        return new ProductPoint<>(p1, p2);
    }

    @Override
    public ProductTangentVector<M1P, M1V, M2P, M2V> log(ProductPoint<M1P, M2P> start, ProductPoint<M1P, M2P> end) {
        M1V v1 = firstManifold.log(
                start.points.one(),
                end.points.one()
        );

        M2V v2 = secondManifold.log(
                start.points.two(),
                end.points.two()
        );

        return new ProductTangentVector<>(v1, v2);
    }

    @Override
    public ProductTangentVector<M1P, M1V, M2P, M2V> gradient(ProductPoint<M1P, M2P> point, DifferentiableScalarFunction<ProductPoint<M1P, M2P>> function) {
        DifferentiableScalarFunction<M1P> f1 =
                p1 -> function.evaluate(new ProductPoint<>(p1, point.points.two()));

        DifferentiableScalarFunction<M2P> f2 =
                p2 -> function.evaluate(new ProductPoint<>(point.points.one(), p2));

        M1V g1 = firstManifold.gradient(point.points.one(), f1);
        M2V g2 = secondManifold.gradient(point.points.two(), f2);

        return new ProductTangentVector<>(g1, g2);
    }

    @Override
    public ProductTangentVector<M1P, M1V, M2P, M2V> parallelTransport(ProductPoint<M1P, M2P> start,
                                                                      ProductPoint<M1P, M2P> end,
                                                                      ProductTangentVector<M1P, M1V, M2P, M2V> vector) {
        return new ProductTangentVector<>(
                firstManifold.parallelTransport(
                        start.points.one(),
                        end.points.one(),
                        vector.vectors.one()
                ),
                secondManifold.parallelTransport(
                        start.points.two(),
                        end.points.two(),
                        vector.vectors.two()
                )
        );
    }

    public static class ProductPoint<M1P extends ManifoldPoint<M1P>, M2P extends ManifoldPoint<M2P>>
            implements ManifoldPoint<ProductPoint<M1P, M2P>> {
        public final Pair<M1P, M2P> points;

        public ProductPoint(M1P pointOne, M2P pointTwo) {
            this.points = new Pair<>(pointOne, pointTwo);
        }

        @Override
        public ProductPoint<M1P, M2P> copy() {
            return new ProductPoint<M1P, M2P>(points.one().copy(), points.two().copy());
        }
    }

    public static class ProductTangentVector<M1P extends ManifoldPoint<M1P>,
            M1V extends TangentVector<M1P, M1V>,
            M2P extends ManifoldPoint<M2P>,
            M2V extends TangentVector<M2P, M2V>> implements TangentVector<ProductPoint<M1P, M2P>,
            ProductTangentVector<M1P, M1V, M2P, M2V>> {
        public final Pair<M1V, M2V> vectors;

        public ProductTangentVector(M1V vectorOne, M2V vectorTwo) {
            this.vectors = new Pair<>(vectorOne, vectorTwo);
        }

        public ProductTangentVector(Pair<M1V, M2V> vectors) {
            this.vectors = vectors;
        }

        @Override
        public ProductPoint<M1P, M2P> basePoint() {
            return new ProductPoint<>(vectors.one().basePoint(), vectors.two().basePoint());
        }

        @Override
        public ProductTangentVector<M1P, M1V, M2P, M2V> add(ProductTangentVector<M1P, M1V, M2P, M2V> other) {
            M1V newVectorOne = vectors.one().add(other.vectors.one());
            M2V newVectorTwo = vectors.two().add(other.vectors.two());
            return new ProductTangentVector<>(newVectorOne, newVectorTwo);
        }

        @Override
        public ProductTangentVector<M1P, M1V, M2P, M2V> scale(double alpha) {
            M1V newVectorOne = vectors.one().scale(alpha);
            M2V newVectorTwo = vectors.two().scale(alpha);
            return new ProductTangentVector<>(newVectorOne, newVectorTwo);
        }

        @Override
        public Endomorphism<ProductTangentVector<M1P, M1V, M2P, M2V>> tensorProduct(ProductTangentVector<M1P, M1V, M2P, M2V> other) {
            return new ProductEndomorphism<>(vectors.one().tensorProduct(other.vectors.one()),
                    vectors.two().tensorProduct(other.vectors.two()));
        }
    }

    public static class ProductEndomorphism<M1P extends ManifoldPoint<M1P>,
            M1V extends TangentVector<M1P, M1V>,
            M2P extends ManifoldPoint<M2P>,
            M2V extends TangentVector<M2P, M2V>> implements Endomorphism<ProductTangentVector<M1P, M1V, M2P, M2V>> {
        public final Endomorphism<M1V> one;
        public final Endomorphism<M2V> two;

        public ProductEndomorphism(Endomorphism<M1V> one, Endomorphism<M2V> two) {
            this.one = one;
            this.two = two;
        }

        @Override
        public double determinant() {
            return one.determinant() * two.determinant();
        }

        @Override
        public ProductEndomorphism<M1P, M1V, M2P, M2V> inverse() {
            return new ProductEndomorphism<>(one.inverse(), two.inverse());
        }

        @Override
        public ProductTangentVector<M1P, M1V, M2P, M2V> multiply(ProductTangentVector<M1P, M1V, M2P, M2V> vector) {
            return new ProductTangentVector<>(one.multiply(vector.vectors.one()), two.multiply(vector.vectors.two()));
        }

        @Override
        public ProductEndomorphism<M1P, M1V, M2P, M2V> multiply(Endomorphism<ProductTangentVector<M1P, M1V, M2P, M2V>> other) {
            ProductEndomorphism<M1P,M1V,M2P,M2V> o = (ProductEndomorphism<M1P,M1V,M2P,M2V>) other;
            return new ProductEndomorphism<>(
                    one.multiply(o.one),
                    two.multiply(o.two)
            );
        }

        @Override
        public ProductEndomorphism<M1P, M1V, M2P, M2V> add(Endomorphism<ProductTangentVector<M1P, M1V, M2P, M2V>> other) {
            ProductEndomorphism<M1P,M1V,M2P,M2V> o = (ProductEndomorphism<M1P,M1V,M2P,M2V>) other;
            return new ProductEndomorphism<>(
                    one.add(o.one),
                    two.add(o.two)
            );
        }

        @Override
        public ProductEndomorphism<M1P, M1V, M2P, M2V> scale(double alpha) {
            return new ProductEndomorphism<>(one.scale(alpha), two.scale(alpha));
        }

        @Override
        public ProductEndomorphism<M1P, M1V, M2P, M2V> transpose() {
            return new ProductEndomorphism<>(one.transpose(), two.transpose());
        }

        @Override
        public double trace() {
            return one.trace() + two.trace();
        }
    }

    public ProductTangentVector<M1P, M1V, M2P, M2V> concat(M1V vectorOne, M2V vectorTwo) {
        return new ProductTangentVector<>(vectorOne, vectorTwo);
    }
}
