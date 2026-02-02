package manifold.endomorphism;

import manifold.TangentVector;

public class ScaledIdentityEndomorphism<V extends TangentVector<?, V>> implements Endomorphism<V> {
    public final double scaleFactor;
    private final double dim;

    public ScaledIdentityEndomorphism(double scaleFactor, double dim) {
        this.scaleFactor = scaleFactor;
        this.dim = dim;
    }

    @Override
    public double determinant() {
        return Math.pow(scaleFactor, dim);
    }

    @Override
    public Endomorphism<V> inverse() {
        if (scaleFactor < 1e-8) throw new IllegalArgumentException("Cannot invert zero endomorphism");
        return new ScaledIdentityEndomorphism<>(1 / scaleFactor, dim);
    }

    @Override
    public V multiply(V vector) {
        return vector.scale(scaleFactor);
    }

    @Override
    public Endomorphism<V> multiply(Endomorphism<V> other) {
        ScaledIdentityEndomorphism<V> o = (ScaledIdentityEndomorphism<V>) other;
        if (dim != o.dim) throw new IllegalArgumentException("Cannot multiply matrices of different dimension");
        return new ScaledIdentityEndomorphism<>(scaleFactor * o.scaleFactor, dim);
    }

    @Override
    public Endomorphism<V> add(Endomorphism<V> other) {
        ScaledIdentityEndomorphism<V> o = (ScaledIdentityEndomorphism<V>) other;
        if (dim != o.dim) throw new IllegalArgumentException("Cannot add matrices of different dimension");
        return new ScaledIdentityEndomorphism<>(scaleFactor + o.scaleFactor, dim);
    }

    @Override
    public Endomorphism<V> scale(double alpha) {
        return new ScaledIdentityEndomorphism<>(scaleFactor * alpha, dim);
    }

    @Override
    public Endomorphism<V> transpose() {
        return this;
    }

    @Override
    public double trace() {
        return scaleFactor * dim;
    }

    public double getDimension() {
        return dim;
    }
}
