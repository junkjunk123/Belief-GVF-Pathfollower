package manifold.endomorphism;

import manifold.TangentVector;

public class IdentityEndomorphism<V extends TangentVector<?, V>> extends ScaledIdentityEndomorphism<V> {
    public IdentityEndomorphism(double dim) {
        super(1.0, dim);
    }
}
