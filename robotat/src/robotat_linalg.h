/**
 * @file robotat_linalg.h
 *
 * Adapted from CControl (https://github.com/DanielMartensson/CControl) with the following changes:
 *
 * 1. Removed all but linear algebra and (some) optimization routines.
 * 2. Changed all variable length arrays for fixed size to increase portability (as VLAs are compiler
 *    dependent extensions since C11 and generally a bad idea in embedded).
 * 3. Merged all function implementations into a single source file, this introduces some clutter
 *    but allows a more manageable memory footprint by defining static, auxiliary, fixed-size float
 *    arrays (originally some function calls like inv needed a huge, impractical stack size when using
 *    both variable-length and fixed-length arrays created inside the routines).
 * 4. Defined a matrix data structure to add code readability, decrease redundancy in constantly passing
 *    matrix dimensions as parameters and add compatibility with ARM's CMSIS DSP libraries (this will
 *    allow us to define wrappers for ARM's HW accelerated routines). This also adds a layer of safety
 *    when doing linear algebra operations. It's even recommended to use single row or column matrices
 *    instead of arrays to gain these dimension checks even though it has a speed penalty (check next point).
 * 5. Added a size mismatch check similar to ARM's CMSIS DSP matrix libraries (this can be disabled to
 *    reduce overhead by undefining the MATH_MATRIX_CHECK macro).
 
 *   Created on: 5 oct. 2019
 *      @author: Daniel Martensson
 *  Modified on: 1 aug. 2021
 *           By: Miguel Zea (mezea@uvg.edu.gt)
 */

#ifndef ROBOTAT_LINALG_H_
#define ROBOTAT_LINALG_H_

 /**
  * Dependencies.
  */
#include <string.h>	                    // For memcpy, memset etc.
#include <stdio.h>                      // For printf.
#include <stdlib.h>                     // Standard library.
#include <stdint.h>	                    // For uint8_t, uint16_t and uint16_t.
#include <math.h>	                    // For sqrtf.
#include <float.h>	                    // Required for FLT_EPSILON.
#include <stdbool.h>                    // For bool datatype.
#include <time.h>                       // For srand, clock.

// ====================================================================================================
// Constant macro definitions
// ====================================================================================================
#define MAX_ITERATION_COUNT_SVD (30)   	/**< Maximum number of iterations for svd_jacobi_one_sided.c */
#define MAX_VEC_SIZE	        (100)	/**< Maximum number of elements allowed for a single row vector. */
#define MAX_MAT_SIZE            (MAX_VEC_SIZE*MAX_VEC_SIZE)     /**< Maximum number of elements allowed for a matrix. */
#define MATH_MATRIX_CHECK               /**< Comment this to disable matrix size checking. */

// ====================================================================================================
// Data structures, enums and type definitions
// ====================================================================================================
/**
 * @brief Floating point matrix data structure.
 * 
 * Used for readability and compatibility with ARM's CMSIS-DSP library.
 */
typedef struct
{
    uint16_t num_rows;  /**< Number of rows of the matrix. */
    uint16_t num_cols;  /**< Number of columns of the matrix. */
    float* p_data;      /**< Points to the data of the matrix. */
} matf32_t;


/**
 * @brief Error status from matrix operations.
 * 
 * Defined this way for compatibility with ARM's CMSIS-DSP library.
 */
typedef enum
{
    MATH_SUCCESS,
    MATH_ARGUMENT_ERROR,
    MATH_LENGTH_ERROR,
    MATH_SIZE_MISMATCH,
    MATH_NANINF,
    MATH_SINGULAR,
    MATH_TEST_FAILURE,
    MATH_DECOMPOSITION_FAILURE
} err_status_t;


// ====================================================================================================
// Matrix datatype-based linear algebra routines
// TODO: add inline function wrappers if the target is an ARM processor to use the CMSIS-DSP library.
// ====================================================================================================
/**
 * @brief   Constructor for the floating point matrix data structure.
 * 
 * @param[in, out]  instance    Points to an instance of the floating-point matrix structure.
 * @param[in]       num_rows    Number of rows in the matrix.
 * @param[in]       num_cols    Number of columns in the matrix.
 * @param[in]       p_data      Points to the matrix data array.
 * 
 * @return  None
 */
void
matf32_init(matf32_t* const instance, uint16_t num_rows, uint16_t num_cols, float* p_data);


/**
 * @brief   Adds two matrices. Both need to be of the same dimension.
 *
 * @param[in]       p_srca  Points to first input matrix structure.
 * @param[in]       p_srcb  Points to second input matrix structure.
 * @param[in, out]  p_dst   Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_add(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst);


/**
 * @brief   Substracts two matrices. Both need to be of the same dimension.
 *
 * @param[in]       p_srca  Points to first input matrix structure.
 * @param[in]       p_srcb  Points to second input matrix structure.
 * @param[in, out]  p_dst   Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_sub(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst);


/**
 * @brief   Multiplies a matrix by a scalar, element-wise.
 *
 * @param[in]       p_src   Points to input matrix.
 * @param[in]       scalar  Scaling factor.
 * @param[in, out]  p_dst   Points to output matrix.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_scale(const matf32_t* p_src, float scalar, matf32_t* p_dst);


/**
 * @brief   Transposes a matrix.
 *
 * @param[in]       p_src   Points to input matrix.
 * @param[in, out]  p_dst   Points to output matrix.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_trans(const matf32_t* p_src, matf32_t* p_dst);


/**
 * @brief   Multiplies two matrices. The number of columns of the first matrix must be the same as
 * the number of rows of the second matrix. Output matrix cannot be the same as one of the inputs.
 *
 * @param[in]       p_srca  Points to first input matrix structure.
 * @param[in]       p_srcb  Points to second input matrix structure.
 * @param[in, out]  p_dst   Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_mul(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst);


/**
 * @brief   Computes the LU decomposition (with partial pivoting) of a square matrix A, pointed by p_src,
 * such that PA = LU.
 *
 * @param[in]       p_src   Points to square matrix to decompose.
 * @param[in, out]  p_lu    Points to the result of the decomposition.
 * @param[in, out]  p_dst   Points to the pivot vector.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 *              MATH_SINGULAR :         Matrix is singular.
 */
err_status_t
matf32_lup(const matf32_t* p_src, matf32_t* p_lu, uint16_t* pivot);


/**
 * @brief   Computes the inverse of a square, non-singular matrix.
 * 
 * NOTE: use only as a last resort, solving the linear system Ax = b should always be the first choice.
 * This routine is also very numerically sensitive, as the matrix are defined with 32-bit floating point
 * data.
 *
 * @param[in]       p_src   Points to input matrix.
 * @param[in, out]  p_dst   Points to output matrix.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 *              MATH_SINGULAR :         Matrix is singular.
 */
err_status_t
matf32_inv(const matf32_t* p_src, matf32_t* p_dst);


/**
 * @brief   Matrix-vector post multiplication, i.e. Ax. Assumes a column vector.
 *
 * @param[in]       p_srcm  Points to input matrix.
 * @param[in]       p_srcv  Points to input vector.
 * @param[in, out]  p_dst   Points to result.
 *
 * @return  None.
 */
void
matf32_vecposmult(const matf32_t* p_srcm, float* p_srcv, float* p_dst);


/**
 * @brief   Prints matrix data to console (formatted).
 *
 * @param[in]   p_src   Points to input matrix.
 *
 * @return  None.
 */
void
matf32_print(const matf32_t* p_src);


/**
 * @brief   Gets an specific element from a matrix.
 * 
 * @param[in]       p_src   Points to matrix.
 * @param[in]       row     Row of element.
 * @param[in]       col     Column of element.
 * @param[in, out]  dst     Points to variable to store element.
 * 
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
static inline err_status_t
matf32_get(const matf32_t* p_src, uint16_t row, uint16_t col, float* dst)
{
#ifdef MATH_MATRIX_CHECK 
    if ((row < p_src->num_rows) && (col < p_src->num_cols));
    else return MATH_SIZE_MISMATCH;
#endif
    * dst = p_src->p_data[row*p_src->num_rows + col];
}


/**
 * @brief   Sets an specific element in a matrix.
 *
 * @param[in, out]  p_src   Points to matrix.
 * @param[in]       row     Row of element.
 * @param[in]       col     Column of element.
 * @param[in]       dst     Value of element to set.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
static inline err_status_t
matf32_set(matf32_t* const p_src, uint16_t row, uint16_t col, float value)
{
#ifdef MATH_MATRIX_CHECK 
    if ((row < p_src->num_rows) && (col < p_src->num_cols));
    else return MATH_SIZE_MISMATCH;
#endif
    p_src->p_data[row * p_src->num_rows + col] = value;
}


/**
 * Checks whether or not two matrices have the same size.
 *
 * @param[in]	p_srca	Points to first input matrix.
 * @param[in]	p_srcb	Points to second input matrix.
 *
 * @return  true if matrices have the same size, false otherwise.
 */
static inline bool
matf32_is_same_size(const matf32_t* p_srca, const matf32_t* p_srcb)
{
    return ((p_srca->num_rows == p_srcb->num_rows) && (p_srca->num_cols == p_srcb->num_cols));
}


/**
 * Checks if a matrix has a specified size.
 *
 * @param[in]	p_src	Points to input matrix.
 *
 * @return  true if the matrix has the specified size, false otherwise.
 */
static inline bool
matf32_size_check(const matf32_t* p_src, uint16_t rows, uint16_t cols)
{
    return ((p_src->num_rows == rows) && (p_src->num_cols == cols));
}


/**
 * @brief   Size-aware matrix copy.
 *
 * @param[in]       p_src   Points to matrix to copy from.
 * @param[in, out]  p_dst   Points to matrix to copy to.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
static inline err_status_t
matf32_copy(const matf32_t* p_src, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
    if (matf32_is_same_size(p_src, p_dst));
    else return MATH_SIZE_MISMATCH;
#endif
    memcpy(p_dst->p_data, p_src->p_data, p_src->num_rows * p_src->num_cols * sizeof(float));
    return MATH_SUCCESS;
}


/**
 * @brief   Changes the shape of a given matrix structure. 
 * 
 * WARNING: this routine is unable to check if the data array is large enough to allow a reshape, 
 * as matrices are statically allocated. Can only be used safely if the matrix was originally 
 * defined to have the max number of rows and columns. Use matf32_reshape_safe if you want to
 * automatically verify if the reshape is possible given the input matrix dimensions.
 *
 * @param[in, out]  p_src       Points to matrix to reshape.
 * @param[in]       new_rows    New number of rows.
 * @param[in]       new_cols    New number of columns.
 *
 * @return  None.
 */
static inline void
matf32_reshape(matf32_t* const p_src, uint16_t new_rows, uint16_t new_cols)
{
    p_src->num_rows = new_rows;
    p_src->num_cols = new_cols;
}


/**
 * @brief   Changes the shape of a given matrix structure if possible, given the input matrix dimensions.
 *
 * WARNING: this routine is unable to reshape a matrix to a bigger size. Use matf32_reshape if you want
 * to do this.
 *
 * @param[in, out]  p_src       Points to matrix to reshape.
 * @param[in]       new_rows    New number of rows.
 * @param[in]       new_cols    New number of columns.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
static inline err_status_t
matf32_reshape_safe(matf32_t* const p_src, uint16_t new_rows, uint16_t new_cols)
{
    if ((new_rows * new_cols) <= (p_src->num_rows * p_src->num_cols))
    {
        p_src->num_rows = new_rows;
        p_src->num_cols = new_cols;
        return MATH_SUCCESS;
    }
    else
        return MATH_SIZE_MISMATCH;
}


/**
 * @brief   Sets a matrix structure to the identity matrix.
 *
 * @param[in, out]  p_dst   Points to matrix to allocate the identity matrix.
 *
 * @return None.
 */
void
matf32_eye(matf32_t* const p_dst);


/**
 * @brief   Sets a matrix structure to a diagonal matrix, created from a given vector.
 *
 * @param[in]       p_src       Points to vector with diagonal entries.
 * @param[in,out]   p_dst       Points to matrix to allocate the diagonal matrix.
 *
 * @return None.
 */
void
matf32_diag(float* p_src, matf32_t* const p_dst);


/**
 * @brief   Sets a matrix structure to the zero matrix.
 *
 * @param[in, out]  p_dst   Points to matrix to allocate the zero matrix.
 *
 * @return None.
 */
void 
matf32_zeros(matf32_t* const p_dst);


/**
 * @brief   Sets a matrix structure to a ones matrix.
 *
 * @param[in, out]  p_dst   Points to matrix to allocate the ones matrix.
 *
 * @return None.
 */
void
matf32_ones(matf32_t* const p_dst);


/**
 * @brief   Sets a matrix structure to a matrix with random entries, sampled from a normal
 * distribution.
 *
 * @param[in, out]  p_dst   Points to random matrix.
 * @param[in]       mu      Mean.
 * @param[in]       sigma   Standard deviation.
 *
 * @return  None.
 */
void
matf32_randn(matf32_t* const p_dst, float mu, float sigma);


/**
 * @brief   Adds an array of matrices sequentially. The size of all matrices must be the same.
 *
 * @param[in]       p_matarray  Points to the matrix array.
 * @param[in]       length      Number of matrices in the array.
 * @param[in, out]  p_dst       Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_arr_add(const matf32_t** p_matarray, const uint16_t length, matf32_t* p_dst);


/**
 * @brief   Subtracts an array of matrices sequentially. The size of all matrices must be the same.
 *
 * @param[in]       p_matarray  Points to the matrix array.
 * @param[in]       length      Number of matrices in the array.
 * @param[in, out]  p_dst       Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_arr_sub(const matf32_t** p_matarray, const uint16_t length, matf32_t* p_dst);


/**
 * @brief   Multiplies an array of matrices sequentially. The number of columns of any matrix must be the same as
 * the number of rows of the next. Output matrix cannot be the same as one of the inputs.
 *
 * @param[in]       p_matarray  Points to the matrix array.
 * @param[in]       length      Number of matrices in the array.
 * @param[in, out]  p_dst       Points to output matrix structure.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
err_status_t
matf32_arr_mul(const matf32_t** p_matarray, const uint16_t length, matf32_t* p_dst);


// ====================================================================================================
// Linear algebra routines that do not depend on the matrix datatype
// ====================================================================================================
/**
 * @brief   Find the dot product of two vectors, pointed by p_srca and p_srcb, of the same size.
 *
 * @param[in]   p_srca  Points to vector 1.
 * @param[in]   p_srcb  Points to vector 2.
 * @param[in]   length  Length of vectors.
 *
 * @return Dot product between the vectors.
 */
float
dot(float* p_srca, float* p_srcb, uint16_t length);


/**
 * @brief   Create an identity matrix array with size row x column.
 *
 * @param[in, out]  p_dst   Points to array to allocate the identity matrix.
 * @param[in]       row     Number of required rows.
 * @param[in]       column  Number of required columns.
 *
 * @return None.
 */
void
eye(float* p_dst, uint16_t row, uint16_t column);


/**
 * @brief   Creates a diagonal matrix array pointed by p_dst with the size row x column,
 * from a vector pointed by p_src. Notice that the row of vector x need to be the same
 * length as the column of the matrix.
 *
 * @param[in]       p_src       Points to vector with diagonal entries.
 * @param[in,out]   p_dst       Points to array to allocate the diagonal matrix.
 * @param[in]       row_d       Number of required rows.
 * @param[in]       column_d    Number of required columns.
 *
 * @return None.
 */
void
diag(float* p_src, float* p_dst, int row_d, int column_d);


/**
 * @brief   Turn all elements of the matrix array pointed by p_dst, size row x column, into 0.
 *
 * @param[in,out]   p_dst   Points to zero matrix array.
 * @param[in]       row     Number or rows.
 * @param[in]       column  Number of columns.
 *
 * @return None.
 */
void
zeros(float* p_dst, int row, int column);


/**
 * @brief   Turn all elements of the matrix array pointed by p_dst, size row x column, into 1.
 *
 * @param[in,out]   p_dst   Points to ones matrix array.
 * @param[in]       row     Number or rows.
 * @param[in]       column  Number of columns.
 *
 * @return None.
 */
void
ones(float* p_dst, int row, int column);


/**
 * @brief   Creates a random array with values sampled from a normal (Gaussian) distribution.
 *
 * @param[in, out]  p_dst   Points to random vector to create.
 * @param[in]       length  Vector length.
 * @param[in]       mu      Mean.
 * @param[in]       sigma   Standar deviation.
 *
 * @return  None.
 */
void
randn(float* p_dst, uint16_t length, float mu, float sigma);


// ====================================================================================================
// Miscellaneous
// ====================================================================================================
/**
 * @brief   Size-aware matrix copy.
 *
 * @param[in]       p_src   Points to matrix to copy from.
 * @param[in, out]  p_dst   Points to matrix to copy to.
 *
 * @return  Execution status
 *              MATH_SUCCESS :          Operation successful.
 *              MATH_SIZE_MISMATCH :    Matrix size check failed.
 */
void
copy(float* p_src, float* p_dst, int row, int column);


/**
 * @brief   Prints array to console (formatted).
 *
 * @param[in]   p_src   Points to array to print.
 * @param[in]   row     Number of rows.
 * @param[in]   column  Number of columns.
 *
 * @return  None.
 */
void
print(float* p_src, uint16_t row, uint16_t column);


/**
 * @brief   Saturates the input, given upper and lower limits.
 * 
 * @param[in]   input           Input value.
 * @param[in]   lower_limit     Lower saturation threshold.
 * @param[in]   upper_limit     Upper saturation threshold.
 * 
 * @return  Saturated output.
 */
float
saturation(float input, float lower_limit, float upper_limit);


/**
 * @brief   Gets the input's sign.
 *
 * @param[in]   number      Input value.
 * 
 * @return  Sign of input.
 */
float
sign(float number);


/**
 * @brief   Gets the mean of a given array.
 *
 * @param[in]   p_src   Points to input array.
 * @param[in]   length  Array length.
 *
 * @return  Mean of array.
 */
float
mean(float* p_src, uint16_t length);


/**
 * @brief   Gets the standard deviation of a given array.
 *
 * @param[in]   p_src   Points to input array.
 * @param[in]   length  Array length.
 *
 * @return  Standard deviation of array.
 */
float
std(float* p_src, uint16_t length);


//// ====================================================================================================
//// Miscellaneous
//// ====================================================================================================
//void
//cut(float A[], uint16_t row, uint16_t column, float B[], uint16_t start_row, uint16_t stop_row, uint16_t start_column, uint16_t stop_column);
//
//void
//insert(float A[], float B[], uint16_t row_a, uint16_t column_a, uint16_t column_b, uint16_t startRow_b, uint16_t startColumn_b);
///**
//  * Linear algebra.
//  */
//void
//linsolve_upper_triangular(float* A, float* x, float* b, uint16_t column);
//
//void
//svd_jacobi_one_sided(float A[], uint16_t row, uint8_t max_iterations, float U[], float S[], float V[]);
//
//void
//dlyap(float A[], float P[], float Q[], uint16_t row);
//
//uint8_t
//svd_golub_reinsch(float A[], uint16_t row, uint16_t column, float U[], float S[], float V[]);
//
//void
//qr(float A[], float Q[], float R[], uint16_t row_a, uint16_t column_a);
//
//void
//linsolve_qr(float A[], float x[], float b[], uint16_t row, uint16_t column);
//
//void
//linsolve_lower_triangular(float A[], float x[], float b[], uint16_t row);

//float
//det(float A[], uint16_t row);
//
//uint8_t
//linsolve_lup(float A[], float x[], float b[], uint16_t row);
//
//void
//chol(float A[], float L[], uint16_t row);
//
//void
//linsolve_chol(float A[], float x[], float b[], uint16_t row);
//
//void
//pinv(float A[], uint16_t row, uint16_t column);
//
//void
//hankel(float V[], float H[], uint16_t row_v, uint16_t column_v, uint16_t row_h, uint16_t column_h, uint16_t shift);
//
//void
//balance(float A[], uint16_t row);
//
//void
//eig(float A[], float wr[], float wi[], uint16_t row);
//
//void
//eig_sym(float A[], uint16_t row, float d[]);
//
//void
//sum(float A[], uint16_t row, uint16_t column, uint8_t l);
//
//float
//norm(float A[], uint16_t row, uint16_t column, uint8_t l);
//
//void
//expm(float A[], uint16_t row);
//
//void
//nonlinsolve(void (*nonlinear_equation_system)(float[], float[], float[]), float b[], float x[], uint8_t elements, float alpha, float max_value, float min_value, bool random_guess_active);
//
//void
//linsolve_gauss(float* A, float* x, float* b, uint16_t row, uint16_t column, float alpha);
//
//
//
///**
//  * Optimization.
//  */
//  /** @TODO: implement convex quadprog and general gradient descent w/o constraints. */
//void
//linprog(float c[], float A[], float b[], float x[], uint8_t row_a, uint8_t column_a, uint8_t max_or_min, uint8_t iteration_limit);

#endif /* ROBOTAT_LINALG_H_ */
