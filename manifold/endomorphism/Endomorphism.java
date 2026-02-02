package manifold.endomorphism;

import manifold.TangentVector;

public interface Endomorphism<VectorType extends TangentVector<?, VectorType>> {
    double determinant();
    Endomorphism<VectorType> inverse();
    VectorType multiply(VectorType vector);
    Endomorphism<VectorType> multiply(Endomorphism<VectorType> other);
    Endomorphism<VectorType> add(Endomorphism<VectorType> other);
    Endomorphism<VectorType> scale(double alpha);
    Endomorphism<VectorType> transpose();
    double trace();

    default Endomorphism<VectorType> subtract(Endomorphism<VectorType> other) {
        return this.add(other.scale(-1));
    }

    /**
     * Computes the Frobenius inner product between this bilinear form and another bilinear form.
     * @param other the other bilinear form
     * @return the Frobenius inner product
     */
    default double innerProduct(Endomorphism<VectorType> other) {
        return transpose().multiply(other).trace();
    }
}
