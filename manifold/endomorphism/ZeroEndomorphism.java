package manifold.endomorphism;

import manifold.TangentVector;

public class ZeroEndomorphism<V extends TangentVector<?, V>> extends ScaledIdentityEndomorphism<V> {
    public ZeroEndomorphism(double dim) {
        super(0.0, dim);
    }
}
