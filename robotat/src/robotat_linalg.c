/**
 * @file robotat_linalg.c
 */
#include "robotat_linalg.h"

// ====================================================================================================
// Private variables
// ====================================================================================================
// Preallocated auxiliary matrices and vectors to use inside matrix operations. This reduces memory 
// and data structure initialization overhead.
static float m1data[MAX_MAT_SIZE];
static matf32_t m1;
static float m2data[MAX_MAT_SIZE];
static matf32_t m2;
static float v1[MAX_VEC_SIZE];
static uint16_t p1[MAX_VEC_SIZE];

// ====================================================================================================
// Private function definitions/implementations
// ====================================================================================================
static float 
generate_gauss(float mu, float sigma) 
{
	float U1, U2, W, scalar;
	static float X1, X2;
	static int call = 0;

	if (call == 1) 
	{
		call = !call;
		return (mu + sigma * (float)X2);
	}

	// Compute the uniform norm
	do 
	{
		U1 = -1 + ((float)rand() / RAND_MAX) * 2;
		U2 = -1 + ((float)rand() / RAND_MAX) * 2;
		W = pow(U1, 2) + pow(U2, 2);
	} while (W >= 1 || W == 0);

	scalar = sqrt((-2 * log(W)) / W);
	X1 = U1 * scalar;
	X2 = U2 * scalar;

	call = !call;

	return (mu + sigma * (float)X1);
}


static void 
solve(float* A, float* x, float* b, uint16_t* P, float* LU, uint16_t row) 
{
	// Forward substitution with pivoting
	for (uint16_t i = 0; i < row; ++i) 
	{
		x[i] = b[P[i]];

		for (uint16_t j = 0; j < i; ++j)
			x[i] = x[i] - LU[row * P[i] + j] * x[j];
	}

	// Backward substitution with pivoting
	for (int16_t i = row - 1; i >= 0; --i) 
	{
		for (int16_t j = i + 1; j < row; ++j)
			x[i] = x[i] - LU[row * P[i] + j] * x[j];

		x[i] = x[i] / LU[row * P[i] + i];
	}
}


// ====================================================================================================
// Public function implementations
// ====================================================================================================
// Matrix datatype-based linear algebra routines
// ====================================================================================================
void
matf32_init(matf32_t* const instance, uint16_t num_rows, uint16_t num_cols, float* p_data)
{
	instance->num_rows = num_rows;
	instance->num_cols = num_cols;
	instance->p_data = p_data;
}


err_status_t
matf32_add(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
	if (matf32_is_same_size(p_srca, p_srcb))
		if (matf32_is_same_size(p_srca, p_dst));
		else return MATH_SIZE_MISMATCH;
	else return MATH_SIZE_MISMATCH;
#endif

	float* p_data_srca = p_srca->p_data;
	float* p_data_srcb = p_srcb->p_data;
	float* p_data_dst = p_dst->p_data;

	for (int i = 0; i < p_srca->num_rows * p_srca->num_cols; i++)
		*(p_data_dst++) = *(p_data_srca++) + *(p_data_srcb++);

	return MATH_SUCCESS;
}


err_status_t
matf32_sub(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
	if (matf32_is_same_size(p_srca, p_srcb))
		if (matf32_is_same_size(p_srca, p_dst));
		else return MATH_SIZE_MISMATCH;
	else return MATH_SIZE_MISMATCH;
#endif

	float* p_data_srca = p_srca->p_data;
	float* p_data_srcb = p_srcb->p_data;
	float* p_data_dst = p_dst->p_data;

	for (int i = 0; i < p_srca->num_rows * p_srca->num_cols; i++)
		*(p_data_dst++) = *(p_data_srca++) - *(p_data_srcb++);

	return MATH_SUCCESS;
}


err_status_t
matf32_scale(const matf32_t* p_src, float scalar, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
	if (matf32_is_same_size(p_src, p_dst));
	else return MATH_SIZE_MISMATCH;
#endif

	float* p_data_src = p_src->p_data;
	float* p_data_dst = p_dst->p_data;

	for (int i = 0; i < p_src->num_rows * p_src->num_cols; i++)
		*(p_data_dst++) = scalar * (*(p_data_src++));

	return MATH_SUCCESS;
}


err_status_t
matf32_trans(const matf32_t* p_src, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
	if (matf32_size_check(p_dst, p_src->num_cols, p_src->num_rows));
	else return MATH_SIZE_MISMATCH;
#endif
	float* p_data_src = p_src->p_data;
	float* p_trans;
	matf32_t* p_tmpmat = &m1;

	matf32_init(p_tmpmat, p_src->num_rows, p_src->num_cols, m1data);

	p_dst->num_rows = p_src->num_cols;
	p_dst->num_cols = p_src->num_rows;

	for (int i = 0; i < p_src->num_rows; i++)
	{
		p_trans = &p_tmpmat->p_data[i];
		for (int j = 0; j < p_src->num_cols; j++)
		{
			*p_trans = *(p_data_src++);
			p_trans += p_src->num_rows;
		}
	}

	memcpy(p_dst->p_data, p_tmpmat->p_data, p_dst->num_rows * p_dst->num_cols * sizeof(float));
	return MATH_SUCCESS;
}


err_status_t
matf32_mul(const matf32_t* p_srca, const matf32_t* p_srcb, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
	// Check size consistency
	if (!matf32_size_check(p_dst, p_srca->num_rows, p_srcb->num_cols) || (p_srca->num_cols != p_srcb->num_rows))
		return MATH_SIZE_MISMATCH;

	/*if ((p_srca->num_cols != p_srcb->num_rows) || (p_srca->num_rows != p_dst->num_rows) || (p_srcb->num_cols != p_dst->num_cols))
		return MATH_SIZE_MISMATCH;*/
#else
	// Set output matrix dimensions
	p_dst->num_rows = p_srca->num_rows;
	p_dst->num_cols = p_srcb->num_cols;
#endif

	// Checks if one of the inputs is being used to store the output (this is NOT allowed even in the square matrix case)
	if ((p_srca->p_data == p_dst->p_data) || (p_srcb->p_data == p_dst->p_data))
		return MATH_ARGUMENT_ERROR;

	// Data matrix
	float* data_a;
	float* data_b;
	float* data_c = p_dst->p_data;

	for (uint16_t i = 0; i < p_srca->num_rows; i++) {
		// Then we go through every column of b
		for (uint16_t j = 0; j < p_srcb->num_cols; j++) {
			data_a = &p_srca->p_data[i * p_srca->num_cols];
			data_b = &p_srcb->p_data[j];

			*data_c = 0; // Reset
			// And we multiply rows from a with columns of b
			for (uint16_t k = 0; k < p_srca->num_cols; k++) {
				*data_c += *data_a * *data_b;
				data_a++;
				data_b += p_srcb->num_cols;
			}
			data_c++;
		}
	}

	return MATH_SUCCESS;
}


err_status_t
matf32_lup(const matf32_t* p_src, matf32_t* p_lu, uint16_t* pivot)
{
#ifdef MATH_MATRIX_CHECK 
	if (matf32_is_same_size(p_src, p_lu));
	else return MATH_SIZE_MISMATCH;
#endif

	// Check if the input matrix is square 
	if (p_src->num_cols != p_src->num_rows)
		return MATH_SIZE_MISMATCH;

	uint16_t ind_max;
	uint16_t tmp_int;
	uint16_t row = p_lu->num_rows;

	// Don't copy if the pointer to the decomposition data is the same as the input
	if (p_src->p_data != p_lu->p_data)
		memcpy(p_lu->p_data, p_src->p_data, p_src->num_rows * p_src->num_cols * sizeof(float));

	// Create the pivot vector
	for (uint16_t i = 0; i < row; ++i)
		pivot[i] = i;

	for (uint16_t i = 0; i < row - 1; ++i)
	{
		ind_max = i;
		for (uint16_t j = i + 1; j < p_src->num_rows; ++j)
			if (fabsf(p_lu->p_data[row * pivot[j] + i]) > fabsf(p_lu->p_data[row * pivot[ind_max] + i]))
				ind_max = j;

		tmp_int = pivot[i];
		pivot[i] = pivot[ind_max];
		pivot[ind_max] = tmp_int;

		if (fabsf(p_lu->p_data[row * pivot[i] + i]) < FLT_EPSILON)
			return MATH_SINGULAR; // matrix is singular (up to tolerance)

		for (uint16_t j = i + 1; j < row; ++j)
		{
			p_lu->p_data[row * pivot[j] + i] = p_lu->p_data[row * pivot[j] + i] / p_lu->p_data[row * pivot[i] + i];

			for (uint16_t k = i + 1; k < row; ++k)
				p_lu->p_data[row * pivot[j] + k] = p_lu->p_data[row * pivot[j] + k]
				- p_lu->p_data[row * pivot[i] + k] * p_lu->p_data[row * pivot[j] + i];
		}
	}

	return MATH_SUCCESS;
}


err_status_t
matf32_inv(const matf32_t* p_src, matf32_t* p_dst)
{
#ifdef MATH_MATRIX_CHECK 
	if (matf32_is_same_size(p_src, p_dst));
	else return MATH_SIZE_MISMATCH;
#endif

	// Get number of rows
	uint16_t row = p_src->num_rows;

	// Check if the input matrix is square 
	if (p_src->num_cols != row)
		return MATH_SIZE_MISMATCH;

	// Define and reset temporary vectors and matrices
	matf32_t* lu = &m1;
	matf32_t* invmat = &m2;
	float* tmpvec = v1;
	uint16_t* p = p1;

	matf32_init(lu, row, row, m1data);
	matf32_init(invmat, row, row, m2data);
	memset(tmpvec, 0, row * sizeof(float));

	// Check if the determinant is 0
	if (matf32_lup(p_src, lu, p) == MATH_SINGULAR)
		return MATH_SINGULAR; // matrix is singular

	// Create the inverse
	for (uint16_t i = 0; i < row; ++i)
	{
		tmpvec[i] = 1.0;
		solve(p_src->p_data, &invmat->p_data[row * i], tmpvec, p, lu->p_data, row);
		tmpvec[i] = 0.0;
	}

	// Transpose of temp A^-1
	matf32_trans(invmat, invmat);

	// Copy data from temp to A^-1 (this allows to overwrite the input matrix for the inverse)
	memcpy(p_dst->p_data, invmat->p_data, row * row * sizeof(float));

	return MATH_SUCCESS;
}


void
matf32_vecposmult(const matf32_t* p_srcm, float* p_srcv, float* p_dst)
{
	float* tmpvec;
	float* res = v1;

	for (int i = 0; i < p_srcm->num_rows; i++)
	{
		tmpvec = p_srcm->p_data + i * p_srcm->num_rows;
		for (int j = 0; j < p_srcm->num_cols; j++)
			*res += (*(tmpvec++)) * (*(p_srcv++));
		res++;
	}

	memcpy(p_dst, res, p_srcm->num_rows * sizeof(float));
}


void
matf32_print(const matf32_t* p_src)
{
	float* p_data_src = p_src->p_data;

	for (uint16_t i = 0; i < p_src->num_rows; i++)
	{
		for (uint16_t j = 0; j < p_src->num_cols; j++)
		{
			printf("%0.18f\t", *(p_data_src++));
		}
		printf("\n");
	}
	printf("\n");
}


void
matf32_eye(matf32_t* const p_dst)
{
	eye(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_diag(float* p_src, matf32_t* const p_dst)
{
	diag(p_src, p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_zeros(matf32_t* const p_dst)
{
	zeros(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_ones(matf32_t* const p_dst)
{
	ones(p_dst->p_data, p_dst->num_rows, p_dst->num_cols);
}


void
matf32_randn(matf32_t* const p_dst, float mu, float sigma)
{
	randn(p_dst->p_data, p_dst->num_rows * p_dst->num_cols, mu, sigma);
}


err_status_t
matf32_arr_add(const matf32_t** p_matarray, const uint16_t length, matf32_t* p_dst)
{
	if (length < 3)
		return MATH_ARGUMENT_ERROR;

	matf32_t* tmpmat = &m1;
	matf32_init(tmpmat, p_matarray[0]->num_rows, p_matarray[0]->num_cols, m1data);
	matf32_zeros(tmpmat);

	for (uint16_t i = 0; i < length; i++)
	{
#ifdef MATH_MATRIX_CHECK 
		if (matf32_add(tmpmat, p_matarray[i], tmpmat) == MATH_SIZE_MISMATCH)
			return MATH_SIZE_MISMATCH;
#else
		matf32_add(tmpmat, p_matarray[i], tmpmat);
#endif
	}
	matf32_copy(tmpmat, p_dst);
	return MATH_SUCCESS;
}


err_status_t
matf32_arr_sub(const matf32_t** p_matarray, const uint16_t length, matf32_t* p_dst)
{
	if (length < 3)
		return MATH_ARGUMENT_ERROR;

	matf32_t* tmpmat = &m1;
	matf32_init(tmpmat, p_matarray[0]->num_rows, p_matarray[0]->num_cols, m1data);
	matf32_zeros(tmpmat);

#ifdef MATH_MATRIX_CHECK 
	if (matf32_sub(p_matarray[0], p_matarray[1], tmpmat) == MATH_SIZE_MISMATCH)
		return MATH_SIZE_MISMATCH;
#else
	matf32_sub(p_matarray[0], p_matarray[1], tmpmat)
#endif

	for (uint16_t i = 2; i < length; i++)
	{
#ifdef MATH_MATRIX_CHECK 
		if (matf32_sub(tmpmat, p_matarray[i], tmpmat) == MATH_SIZE_MISMATCH)
			return MATH_SIZE_MISMATCH;
#else
		matf32_sub(tmpmat, p_matarray[i], tmpmat);
#endif
	}
	matf32_copy(tmpmat, p_dst);
	return MATH_SUCCESS;
}


err_status_t
matf32_arr_mul(const matf32_t** p_matarray, const uint16_t length, matf32_t* p_dst)
{
	if (length < 3)
		return MATH_ARGUMENT_ERROR;

	matf32_t* tmpmat1 = &m1;
	matf32_t* tmpmat2 = &m2;

	matf32_init(tmpmat1, p_matarray[0]->num_rows, p_matarray[1]->num_cols, m1data);
	matf32_init(tmpmat2, p_matarray[0]->num_rows, p_matarray[1]->num_cols, m2data);

#ifdef MATH_MATRIX_CHECK
	if (matf32_mul(p_matarray[0], p_matarray[1], tmpmat1) == MATH_SIZE_MISMATCH)
		return MATH_SIZE_MISMATCH;
#else 
	matf32_mul(p_matarray[0], p_matarray[1], tmpmat1);
#endif

	for (uint16_t i = 2; i < length; i++)
	{
		matf32_reshape(tmpmat2, tmpmat1->num_rows, p_matarray[i]->num_cols);
#ifdef MATH_MATRIX_CHECK 
		if (matf32_mul(tmpmat1, p_matarray[i], tmpmat2) == MATH_SIZE_MISMATCH)
			return MATH_SIZE_MISMATCH;
#else
		matf32_mul(tmpmat1, p_matarray[i], tmpmat2);
#endif
		matf32_reshape(tmpmat1, tmpmat2->num_rows, tmpmat2->num_cols);
		matf32_copy(tmpmat2, tmpmat1);
	}

	matf32_copy(tmpmat2, p_dst);
	return MATH_SUCCESS;
}


// ====================================================================================================
// Linear algebra routines that do not depend on the matrix datatype
// ====================================================================================================
float
dot(float* p_srca, float* p_srcb, uint16_t length)
{
	float sum = 0;	// Reset;

	// Multiply each row
	for (int i = 0; i < length; ++i)
		sum += (*(p_srca++)) * (*(p_srcb++));
	return sum;
}


void
eye(float* p_dst, uint16_t row, uint16_t column)
{
	// Reset first
	memset(p_dst, 0, row * column * sizeof(float));

	for (int i = 0; i < row; i++)
	{
		*p_dst = 1.0;
		p_dst += row + 1;
	}
}


void
diag(float* p_src, float* p_dst, int row_d, int column_d)
{
	// Reset the matrix array
	memset(p_dst, 0, row_d * column_d * sizeof(float));

	for (int i = 0; i < row_d; i++) {
		for (int j = 0; j < column_d; j++) {
			if (j == i) {
				*p_dst = p_src[i];
				p_dst += column_d + 1;
			}
		}
	}
}


void
zeros(float* p_dst, int row, int column)
{
	memset(p_dst, 0, row * column * sizeof(float));
}


void
ones(float* p_dst, int row, int column)
{
	memset(p_dst, 1, row * column * sizeof(float));
}


void
randn(float* p_dst, uint16_t length, float mu, float sigma)
{
	srand(time(NULL));
	for (uint16_t i = 0; i < length; i++)
		p_dst[i] = generate_gauss(mu, sigma);
}


// ====================================================================================================
// Miscellaneous
// ====================================================================================================
void
copy(float* p_src, float* p_dst, int row, int column)
{
	memcpy(p_dst, p_src, column * row * sizeof(float));
}

void 
print(float* p_src, uint16_t row, uint16_t column)
{
	for (uint16_t i = 0; i < row; i++) 
	{
		for (uint16_t j = 0; j < column; j++)
			printf("%0.18f\t", *(p_src++));
		printf("\n");
	}
	printf("\n");
}


float 
saturation(float input, float lower_limit, float upper_limit) 
{
	if (input > upper_limit)
		return upper_limit;
	else if (input < lower_limit)
		return lower_limit;
	else 
		return input; // No action
}


float 
sign(float number) 
{
	if (number > 0) 
		return 1;
	else if (number < 0) 
		return -1;
	else 
		return 0;
}


float
mean(float* p_src, uint16_t length)
{
	float s = 0;

	for (uint16_t i = 0; i < length; i++)
		s += p_src[i];
	return s / ((float)length);
}


float
std(float* p_src, uint16_t length)
{
	float mu = mean(p_src, length);
	float sigma = 0;
	
	for (uint16_t i = 0; i < length; i++)
		sigma += (p_src[i] - mu) * (p_src[i] - mu);
	return sqrtf(sigma / ((float)length));
}