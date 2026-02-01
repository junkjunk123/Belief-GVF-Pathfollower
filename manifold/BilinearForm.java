package manifold;

import solver.Workspace;

public interface BilinearForm<VectorType extends TangentVector<?, VectorType>> {
    double determinant();
    BilinearForm<VectorType> inverse();
    VectorType multiply(VectorType vector);
    BilinearForm<VectorType> multiply(BilinearForm<VectorType> other);
    BilinearForm<VectorType> add(BilinearForm<VectorType> other);
    BilinearForm<VectorType> scale(double alpha);
    BilinearForm<VectorType> transpose();
    double trace();

    default BilinearForm<VectorType> subtract(BilinearForm<VectorType> other) {
        return this.add(other.scale(-1));
    }

    /**
     * Computes the Frobenius inner product between this bilinear form and another bilinear form.
     * @param other the other bilinear form
     * @return the Frobenius inner product
     */
    default double innerProduct(BilinearForm<VectorType> other) {
        return transpose().multiply(other).trace();
    }
}
