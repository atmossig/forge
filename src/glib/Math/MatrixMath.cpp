#include "MatrixMath.h"

namespace glib {
namespace math {

//------------------------------------------------------------------------------
// Matrix Factorization Implementations
//------------------------------------------------------------------------------

// LU decomposition with partial pivoting
template <typename T>
bool LUDecomposition(
    const Matrix4x4<T>& matrix,
    Matrix4x4<T>& lower,
    Matrix4x4<T>& upper,
    Matrix4x4<T>& permutation)
{
    const int n = 4;  // Matrix dimension
    
    // Initialize matrices
    lower = Matrix4x4<T>::Identity();
    upper = matrix;
    permutation = Matrix4x4<T>::Identity();
    
    // Perform LU decomposition with partial pivoting
    for (int i = 0; i < n; i++) {
        // Find pivot row
        T maxVal = std::abs(upper(i, i));
        int maxRow = i;
        
        for (int j = i + 1; j < n; j++) {
            T absVal = std::abs(upper(j, i));
            if (absVal > maxVal) {
                maxVal = absVal;
                maxRow = j;
            }
        }
        
        // Check for singular matrix
        if (maxVal < EPSILON) {
            return false;  // Matrix is singular
        }
        
        // Swap rows if necessary
        if (maxRow != i) {
            for (int j = 0; j < n; j++) {
                // Swap rows in upper
                std::swap(upper(i, j), upper(maxRow, j));
                
                // Swap rows in permutation
                std::swap(permutation(i, j), permutation(maxRow, j));
                
                // Swap already computed elements of lower
                if (j < i) {
                    std::swap(lower(i, j), lower(maxRow, j));
                }
            }
        }
        
        // Compute elements of lower and upper matrices
        for (int j = i + 1; j < n; j++) {
            lower(j, i) = upper(j, i) / upper(i, i);
            
            for (int k = i; k < n; k++) {
                upper(j, k) -= lower(j, i) * upper(i, k);
            }
        }
    }
    
    return true;
}

// QR decomposition using Gram-Schmidt process
template <typename T>
bool QRDecomposition(
    const Matrix4x4<T>& matrix,
    Matrix4x4<T>& q,
    Matrix4x4<T>& r)
{
    // Extract columns of the matrix
    Vector4<T> a[4];
    for (int i = 0; i < 4; i++) {
        a[i] = Vector4<T>(matrix(0, i), matrix(1, i), matrix(2, i), matrix(3, i));
    }
    
    // Use Gram-Schmidt process to compute Q
    Vector4<T> q_col[4];
    
    // First column is normalized a0
    T lenA0 = a[0].Length();
    if (lenA0 < EPSILON) {
        return false;  // Matrix is singular
    }
    q_col[0] = a[0] / lenA0;
    
    // Second column
    T dot01 = a[1].Dot(q_col[0]);
    q_col[1] = a[1] - q_col[0] * dot01;
    
    T lenQ1 = q_col[1].Length();
    if (lenQ1 < EPSILON) {
        return false;  // Matrix is singular
    }
    q_col[1] = q_col[1] / lenQ1;
    
    // Third column
    T dot02 = a[2].Dot(q_col[0]);
    T dot12 = a[2].Dot(q_col[1]);
    q_col[2] = a[2] - q_col[0] * dot02 - q_col[1] * dot12;
    
    T lenQ2 = q_col[2].Length();
    if (lenQ2 < EPSILON) {
        return false;  // Matrix is singular
    }
    q_col[2] = q_col[2] / lenQ2;
    
    // Fourth column
    T dot03 = a[3].Dot(q_col[0]);
    T dot13 = a[3].Dot(q_col[1]);
    T dot23 = a[3].Dot(q_col[2]);
    q_col[3] = a[3] - q_col[0] * dot03 - q_col[1] * dot13 - q_col[2] * dot23;
    
    T lenQ3 = q_col[3].Length();
    if (lenQ3 < EPSILON) {
        return false;  // Matrix is singular
    }
    q_col[3] = q_col[3] / lenQ3;
    
    // Fill in Q matrix
    q = Matrix4x4<T>::Identity();
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            q(j, i) = q_col[i][j];  // column i, row j
        }
    }
    
    // Calculate R = Q^T * A
    r = Matrix4x4<T>::Identity();
    for (int i = 0; i < 4; i++) {
        for (int j = i; j < 4; j++) {  // R is upper triangular
            r(i, j) = q_col[i].Dot(a[j]);
        }
    }
    
    return true;
}

// Singular Value Decomposition (simplified version)
template <typename T>
bool SVDecomposition(
    const Matrix4x4<T>& matrix,
    Matrix4x4<T>& u,
    Vector4<T>& sigma,
    Matrix4x4<T>& v)
{
    // This is a very simplified SVD implementation that only works for well-conditioned matrices
    // For production use, you'd want to use a dedicated linear algebra library
    
    // Step 1: Compute A^T * A and its eigenvalues/eigenvectors
    Matrix4x4<T> ata;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ata(i, j) = 0;
            for (int k = 0; k < 4; k++) {
                ata(i, j) += matrix(k, i) * matrix(k, j);
            }
        }
    }
    
    // Use a fixed number of iterations for power method
    // In a real implementation, use Jacobi or QR algorithm
    Vector4<T> eigenValues;
    Matrix4x4<T> eigenVectors = Matrix4x4<T>::Identity();
    
    // We'll use a simplified approach where we compute one eigenvalue/vector at a time
    // using the power method, then deflate the matrix
    for (int i = 0; i < 4; i++) {
        // Starting vector for power iteration
        Vector4<T> vec(1, 1, 1, 1);
        vec.Normalize();
        
        // Iterate to find dominant eigenvector
        for (int iter = 0; iter < 20; iter++) {
            Vector4<T> nextVec;
            
            // Matrix-vector multiplication: nextVec = ata * vec
            for (int row = 0; row < 4; row++) {
                nextVec[row] = 0;
                for (int col = 0; col < 4; col++) {
                    nextVec[row] += ata(row, col) * vec[col];
                }
            }
            
            // Normalize to prevent overflow
            T length = nextVec.Length();
            if (length < EPSILON) {
                break;  // Vector converged to zero
            }
            
            vec = nextVec / length;
        }
        
        // Compute eigenvalue using Rayleigh quotient
        T eigenValue = 0;
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                eigenValue += vec[row] * ata(row, col) * vec[col];
            }
        }
        
        // Store eigenvalue and eigenvector
        eigenValues[i] = eigenValue;
        for (int j = 0; j < 4; j++) {
            eigenVectors(j, i) = vec[j];
        }
        
        // Deflate matrix: ata = ata - eigenvalue * vec * vec^T
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                ata(row, col) -= eigenValue * vec[row] * vec[col];
            }
        }
    }
    
    // Sort eigenvalues and eigenvectors in descending order
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (eigenValues[i] < eigenValues[j]) {
                std::swap(eigenValues[i], eigenValues[j]);
                
                // Swap columns in eigenvectors
                for (int k = 0; k < 4; k++) {
                    std::swap(eigenVectors(k, i), eigenVectors(k, j));
                }
            }
        }
    }
    
    // V is the matrix of eigenvectors
    v = eigenVectors;
    
    // Compute singular values (square roots of eigenvalues)
    for (int i = 0; i < 4; i++) {
        if (eigenValues[i] < 0) {
            return false;  // Numerical error, eigenvalues should be non-negative
        }
        sigma[i] = std::sqrt(eigenValues[i]);
    }
    
    // Compute U = A * V * Sigma^-1
    u = Matrix4x4<T>::Identity();
    
    for (int i = 0; i < 4; i++) {
        if (sigma[i] < EPSILON) {
            // If singular value is zero, set corresponding column of U to zero
            for (int j = 0; j < 4; j++) {
                u(j, i) = 0;
            }
        } else {
            Vector4<T> uCol;
            
            // Compute A * v_i
            for (int j = 0; j < 4; j++) {
                uCol[j] = 0;
                for (int k = 0; k < 4; k++) {
                    uCol[j] += matrix(j, k) * v(k, i);
                }
            }
            
            // Normalize by sigma
            T invSigma = 1.0 / sigma[i];
            for (int j = 0; j < 4; j++) {
                u(j, i) = uCol[j] * invSigma;
            }
        }
    }
    
    return true;
}

// Explicit template instantiations for float and double
template bool LUDecomposition(const Matrix4x4<float>&, Matrix4x4<float>&, Matrix4x4<float>&, Matrix4x4<float>&);
template bool LUDecomposition(const Matrix4x4<double>&, Matrix4x4<double>&, Matrix4x4<double>&, Matrix4x4<double>&);
template bool QRDecomposition(const Matrix4x4<float>&, Matrix4x4<float>&, Matrix4x4<float>&);
template bool QRDecomposition(const Matrix4x4<double>&, Matrix4x4<double>&, Matrix4x4<double>&);
template bool SVDecomposition(const Matrix4x4<float>&, Matrix4x4<float>&, Vector4<float>&, Matrix4x4<float>&);
template bool SVDecomposition(const Matrix4x4<double>&, Matrix4x4<double>&, Vector4<double>&, Matrix4x4<double>&);

} // namespace math
} // namespace glib