#ifndef TMATRIX_H
#define TMATRIX_H

/** 
 * @file tmatrix.h
 *
 * @brief A dimension-templatized class for matrices of values.
 */
#include <cmath>
#include <mbed.h>

// Structures for static assert.  http://www.boost.org
template <bool x> struct STATIC_ASSERTION_FAILURE;
template <> struct STATIC_ASSERTION_FAILURE<true> { enum { value = 1 }; };
template<int x> struct static_assert_test{};
#define STATIC_ASSERT( B ) \
  typedef __attribute__((unused)) static_assert_test<sizeof(STATIC_ASSERTION_FAILURE<(bool)(B)>) > \
  static_assert_typedef##__LINE__

#if DEBUG
#define ERROR_CHECK(X) (X) 
#else
#define ERROR_CHECK(X)
#endif

// Forward Decl.
template <uint16_t, uint16_t, typename> class BasicMatrix;
template <uint16_t, uint16_t, typename> class TMatrix;
class TMatrixDummy { };

/**
 * @brief Class that layers on operator[] functionality for
 * typical matrices.
 */
template <uint16_t Rows, uint16_t Cols, typename value_type>
class BasicIndexMatrix : public BasicMatrix<Rows,Cols,value_type> {
	typedef BasicMatrix<Rows,Cols,value_type> BaseType;
protected:
	BasicIndexMatrix(TMatrixDummy d) : BaseType(d) { }
public:
	BasicIndexMatrix() { }
	BasicIndexMatrix(const value_type* data) : BaseType(data) {}

	const value_type* operator[](uint16_t r) const {
		ERROR_CHECK(if (r >= Rows) {
			std::clog << "Invalid row index " << r << std::endl;
			return &BaseType::mData[0];
		}
		)
		return &BaseType::mData[r*Cols];
	}

	value_type* operator[](uint16_t r) {
		ERROR_CHECK(if (r >= Rows) {
			std::clog << "Invalid row index " << r << std::endl;
			return &BaseType::mData[0];
		}
		)
		return &BaseType::mData[r*Cols];
	}
};

/**
 * @brief Specialization of BasicIndexMatrix that provides
 * single-indexing operator for column vectors.
 */
template <uint16_t Rows, typename value_type>
class BasicIndexMatrix<Rows,1,value_type> :
		public BasicMatrix<Rows,1,value_type> {
	typedef BasicMatrix<Rows,1,value_type> BaseType;
protected:
	BasicIndexMatrix(TMatrixDummy dummy) : BaseType(dummy) {}
public:
	BasicIndexMatrix() { }
	BasicIndexMatrix(const value_type* data) : BaseType(data) {}

	value_type operator[](uint16_t r) const {
		ERROR_CHECK(if (r >= Rows) {
			std::clog << "Invalid vector index " << r << std::endl;
			return BaseType::mData[0];
		})
		return BaseType::mData[r];
	}
	value_type& operator[](uint16_t r) {
		ERROR_CHECK(if (r >= Rows) {
			std::clog << "Invalid vector index " << r << std::endl;
			return BaseType::mData[0];
		})
		return BaseType::mData[r];
	}

	value_type norm() const {
		return sqrt(norm2());
	}

	value_type norm2() const {
		double normSum = 0;
		for (uint32_t i = 0; i < Rows; i++) {
			normSum += BaseType::mData[i]*BaseType::mData[i];
		}
		return normSum;
	}

	/** @brief Returns matrix with vector elements on diagonal. */
	TMatrix<Rows, Rows, value_type> diag(void) const {
		TMatrix<Rows, Rows, value_type> d;
		for (uint32_t i = 0; i < Rows; i++) d.element(i,i, BaseType::mData[i]);
		return d;
	}

};

/**
 * @brief A dimension-templatized class for matrices of values.
 *
 * This template class generically supports any constant-sized
 * matrix of values.  The @p Rows and @p Cols template parameters
 * define the size of the matrix at @e compile-time.  Hence, the
 * size of the matrix cannot be chosen at runtime.  However, the
 * dimensions are appropriately type-checked at compile-time where
 * possible.
 *
 * By default, the matrix contains values of type @p double.  The @p
 * value_type template parameter may be selected to allow matrices
 * of integers or floats.
 *
 * @note At present, type cohersion between matrices with different
 * @p value_type parameters is not implemented.  It is recommended
 * that matrices with value type @p double be used for all numerical
 * computation.
 *
 * Note that the special cases of row and column vectors are
 * subsumed by this class.
 */
template <uint16_t Rows, uint16_t Cols, typename value_type = double>
class TMatrix : public BasicIndexMatrix<Rows, Cols, value_type> {
	typedef BasicIndexMatrix<Rows, Cols, value_type>  BaseType;
	template <uint16_t R, uint16_t C, typename vt> friend class BasicMatrix;
	TMatrix(TMatrixDummy d) : BaseType(d) {}

public:
	TMatrix() : BaseType() { }
	TMatrix(const value_type* data) : BaseType(data) {}
};

/**
 * @brief Template specialization of TMatrix for Vector4.
 */
template <typename value_type>
class TMatrix<4,1, value_type> : public BasicIndexMatrix<4,1, value_type> {
	typedef BasicIndexMatrix<4, 1, value_type>  BaseType;
	template <uint16_t R, uint16_t C, typename vt> friend class BasicMatrix;
	TMatrix(TMatrixDummy d) : BaseType(d) {}

public:
	TMatrix() { }
	TMatrix(const value_type* data) : BaseType(data) {}
	TMatrix(value_type a0, value_type a1, value_type a2, value_type a3) : BaseType(TMatrixDummy()) {
		BaseType::mData[0] = a0;
		BaseType::mData[1] = a1;
		BaseType::mData[2] = a2;
		BaseType::mData[3] = a3;
	}
};

/**
 * @brief Template specialization of TMatrix for Vector3.
 */
template <typename value_type>
class TMatrix<3,1, value_type> : public BasicIndexMatrix<3,1, value_type> {
	typedef BasicIndexMatrix<3, 1, value_type>  BaseType;
	template <uint16_t R, uint16_t C, typename vt> friend class BasicMatrix;
	TMatrix(TMatrixDummy d) : BaseType(d) {}

public:
	TMatrix() { }
	TMatrix(const value_type* data) : BaseType(data) {}
	TMatrix(value_type a0, value_type a1, value_type a2) : BaseType(TMatrixDummy()) {
		BaseType::mData[0] = a0;
		BaseType::mData[1] = a1;
		BaseType::mData[2] = a2;
	}

	TMatrix<3,1,value_type> cross(const TMatrix<3,1, value_type>& v) const {
		const TMatrix<3,1,value_type>& u = *this;
		return TMatrix<3,1,value_type>(u[1]*v[2]-u[2]*v[1],
									   u[2]*v[0]-u[0]*v[2],
									   u[0]*v[1]-u[1]*v[0]);
	}
};

/**
 * @brief Template specialization of TMatrix for Vector2.
 */
template <typename value_type>
class TMatrix<2,1, value_type> : public BasicIndexMatrix<2,1, value_type> {
	typedef BasicIndexMatrix<2, 1, value_type>  BaseType;
	template <uint16_t R, uint16_t C, typename vt> friend class BasicMatrix;
	TMatrix(TMatrixDummy d) : BaseType(d) {}

public:
	TMatrix() { }
	TMatrix(const value_type* data) : BaseType(data) {}
	TMatrix(value_type a0, value_type a1) : BaseType(TMatrixDummy()) {
		BaseType::mData[0] = a0;
		BaseType::mData[1] = a1;
	}
};

/**
 * @brief Template specialization of TMatrix for a 1x1 vector.
 */
template <typename value_type>
class TMatrix<1,1, value_type> : public BasicIndexMatrix<1,1, value_type> {
	typedef BasicIndexMatrix<1, 1, value_type>  BaseType;
	template <uint16_t R, uint16_t C, typename vt> friend class BasicMatrix;
	TMatrix(TMatrixDummy dummy) : BaseType(dummy) {}

public:
	TMatrix() { }
	TMatrix(const value_type* data) : BaseType(data) {}

	// explicit conversion from value_type
	explicit TMatrix(value_type a0) : BaseType(TMatrixDummy()) { // don't initialize
		BaseType::mData[0] = a0;
	}

	// implicit conversion to value_type
	operator value_type() const {
		return BaseType::mData[0];
	}

	const TMatrix<1,1, value_type>&
	operator=(const TMatrix<1,1, value_type>& m) {
		BaseType::operator=(m);
		return *this;
	}

	double operator=(double a0) {
		BaseType::mData[0] = a0;
		return BaseType::mData[0];
	}
};


/**
 * @brief Base class implementing standard matrix functionality.
 */
template <uint16_t Rows, uint16_t Cols, typename value_type = double>
class BasicMatrix {
protected:
	value_type mData[Rows*Cols];

	// Constructs uninitialized matrix.
	BasicMatrix(TMatrixDummy dummy) {}
public:
	BasicMatrix() { // constructs zero matrix
		for (uint16_t i = 0; i < Rows*Cols; ++i) {
			mData[i] = 0;
		}
	}
	BasicMatrix(const value_type* data) { // constructs from array
		MBED_ASSERT(data);

		for (uint16_t i = 0; i < Rows*Cols; ++i) {
			mData[i] = data[i];
		}
	}

	uint32_t rows() const { return Rows; }
	uint32_t columns() const { return Cols; }
	uint32_t elementCount() const { return Cols*Rows; }

	value_type element(uint16_t row, uint16_t col) const {
		ERROR_CHECK(if (row >= rows() || col >= columns()) {
			std::cerr << "Illegal read access: " << row << ", " << col
					  << " in " << Rows << "x" << Cols << " matrix." << std::endl;
			return mData[0];
		})
		return mData[row*Cols+col];
	}

	value_type& element(uint16_t row, uint16_t col) {
		ERROR_CHECK(if (row >= rows() || col >= columns()) {
			std::cerr << "Illegal read access: " << row << ", " << col
					  << " in " << Rows << "x" << Cols << " matrix." << std::endl;
			return mData[0];
		})
		return mData[row*Cols+col];
	}

	void element(uint16_t row, uint16_t col, value_type value) {
		ERROR_CHECK(if (row >= rows() || col >= columns()) {
			std::cerr << "Illegal write access: " << row << ", " << col
					  << " in " << Rows << "x" << Cols << " matrix." << std::endl;
			return ;
		})
		mData[row*Cols+col] = value;
	}

	TMatrix<Rows*Cols,1, value_type> vec() const {
		return TMatrix<Rows*Cols,1, value_type>(mData);
	}

	void vec(const TMatrix<Rows*Cols, 1, value_type>& vector) {
		for (uint32_t i = 0; i < Rows*Cols; i++) {
			mData[i] = vector.mData[i];
		}
	}

	template <uint16_t R, uint16_t C, uint16_t RowRangeSize, uint16_t ColRangeSize>
	TMatrix<RowRangeSize, ColRangeSize, value_type> subMatrix(void) const {
		STATIC_ASSERT((R+RowRangeSize <= Rows) &&
					  (C+ColRangeSize <= Cols));
		TMatrix<RowRangeSize, ColRangeSize, value_type> result;
		for (uint32_t i = 0; i < RowRangeSize; i++) {
			for (uint32_t j = 0; j < ColRangeSize; j++) {
				result.element(i,j, element(i+R, j+C));
			}
		}
		return result;
	}


	template <uint16_t R, uint16_t C, uint16_t RowRangeSize, uint16_t ColRangeSize>
	void subMatrix(const TMatrix<RowRangeSize, ColRangeSize, value_type>& m) {
		STATIC_ASSERT((R+RowRangeSize <= Rows) &&
					  (C+ColRangeSize <= Cols));
		for (uint32_t i = 0; i < RowRangeSize; i++) {
			for (uint32_t j = 0; j < ColRangeSize; j++) {
				element(i+R,j+C, m.element(i, j));
			}
		}
	}

	/**
	 * @brief Matrix multiplication operator.
	 * @return A matrix where result is the matrix product
	 * (*this) * rhs.
	 */
	template <uint16_t RhsCols>
	TMatrix<Rows, RhsCols, value_type> operator*(const TMatrix<Cols, RhsCols, value_type>& rhs) const {

		TMatrix<Rows, RhsCols, value_type> result;
		const value_type* rPtr = rhs.row(0);
		for (uint32_t i = 0; i < Rows; i++)
		{
			const value_type* rL = row(i);
			const value_type* cR = rPtr;
			value_type* resultRow = result.row(i);
			for (uint32_t j = 0; j < RhsCols; j++)
			{
				const value_type* rR = cR; // start at first element of right col
				const value_type* cL = rL; // start at first element of left row
				double r = 0;
				for (uint32_t k = 0; k < Cols; k++)
				{
					r += (*cL)*(*rR);
					cL++; // step to next col of left matrix
					rR += Cols; // step to next row of right matrix
				}
				resultRow[j] = r;
				cR++; // step to next column of right matrix
			}
		}
		return result;
	}

	/**
	 * @brief Element-wise addition operator.
	 * @return A matrix where result(i,j) = (*this)(i,j) + rhs(i,j).
	 */
	TMatrix<Rows, Cols, value_type> operator+(const TMatrix<Rows, Cols, value_type>& rhs) const {
		TMatrixDummy dummy;
		TMatrix<Rows, Cols, value_type> result(dummy);
		for (uint32_t i = 0;  i < Rows*Cols;  i++) {
			result.mData[i] = mData[i] + rhs.mData[i];
		}
		return result;
	}

	/**
	 * @brief Element-wise subtraction operator.
	 * @return A matrix where result(i,j) = (*this)(i,j) - rhs(i,j).
	 */
	TMatrix<Rows, Cols, value_type> operator-(const TMatrix<Rows, Cols, value_type>& rhs) const {
		TMatrixDummy dummy;
		TMatrix<Rows, Cols, value_type> result(dummy);
		for (uint32_t i = 0;  i < Rows*Cols;  i++) {
			result.mData[i] = mData[i] - rhs.mData[i];
		}
		return result;
	}

	/**
	 * @brief Scalar multiplication operator.
	 * @return A matrix where result(i,j) = (*this)(i,j) * s.
	 */
	TMatrix<Rows, Cols, value_type> operator*(value_type s) const {
		TMatrixDummy dummy;
		TMatrix<Rows, Cols, value_type> result(dummy);
		for (uint32_t i = 0;  i < Rows*Cols;  i++) {
			result.mData[i] = mData[i] * s;
		}
		return result;
	}

	/**
	 * @brief Scalar division operator.
	 * @return A matrix where result(i,j) = (*this)(i,j) / s.
	 */
	TMatrix<Rows, Cols, value_type> operator/(value_type s) const {
		TMatrixDummy dummy;
		TMatrix<Rows, Cols, value_type> result(dummy);
		for (uint32_t i = 0;  i < Rows*Cols;  i++) {
			result.mData[i] = mData[i] / s;
		}
		return result;
	}

	/**
	 * @brief Unary negation operator.
	 * @return A matrix where result(i,j) = -(*this)(i,j).
	 */
	TMatrix<Rows, Cols, value_type> operator-(void) const {
		TMatrixDummy dummy;
		TMatrix<Rows, Cols, value_type> result(dummy);
		for (uint32_t i = 0;  i < Rows*Cols;  i++) {
			result.mData[i] = -mData[i];
		}
		return result;
	}

	/**
	 * @brief Returns the matrix transpose of this matrix.
	 *
	 * @return A TMatrix of dimension @p Cols by @p Rows where
	 * result(i,j) = (*this)(j,i) for each element.
	 */
	TMatrix<Cols, Rows, value_type> transpose(void) const {
		TMatrixDummy dummy;
		TMatrix<Cols, Rows, value_type> result(dummy);
		for (uint16_t i = 0;  i < Rows;  i++) {
			for (uint16_t j = 0;  j < Cols;  j++) {
				result.element(j,i, element(i,j));
			}
		}
		return result;
	}

	/**
	 * @brief Returns the diagonal elements of the matrix.
	 *
	 * @return A column vector @p v with dimension MIN(Rows,Cols) where
	 * @p v[i] = (*this)[i][i].
	 */
	TMatrix<(Rows>Cols)?Cols:Rows, 1, value_type> diag() const {
		TMatrixDummy dummy;
		TMatrix<(Rows>Cols)?Cols:Rows, 1> d(dummy);
		for (uint32_t i = 0; i < d.rows(); i++) {
			d[i] = mData[i*(Cols + 1)];
		}
		return d;
	}

	/** @brief Returns the sum of the matrix entries. */
	value_type sum(void) const {
		value_type s = 0;
		for (uint32_t i = 0; i < Rows*Cols; i++) { s += mData[i]; }
		return s;
	}
	/** @brief Returns the sum of the log of the matrix entries.
	 */
	value_type sumLog(void) const {
		value_type s = 0;
		for (uint32_t i = 0; i < Rows*Cols; i++) { s += log(mData[i]); }
		return s;
	}

	/** @brief Returns this vector with its elements replaced by their reciprocals. */
	TMatrix<Rows,Cols, value_type> recip(void) const {
		TMatrixDummy dummy;
		TMatrix<Rows,Cols, value_type> result(dummy);
		for (uint32_t i = 0; i < Rows*Cols; i++) {
			result.mData[i] = 1.0/mData[i];
		}
		return result;
	}

	/** @brief Returns this vector with its elements replaced by their reciprocals,
	 * unless a value is less than epsilon, in which case it is left as zero.
	 *
	 * This is used mostly for pseudo-inverse computations.
	 */
	TMatrix<Rows,Cols, value_type> pseudoRecip(double epsilon = 1e-50) const {
		TMatrixDummy dummy;
		TMatrix<Rows,Cols, value_type> result(dummy);
		for (uint32_t i = 0; i < Rows*Cols; i++) {
			if (fabs(mData[i]) >= epsilon) {
				result.mData[i] = 1.0/mData[i];
			} else {
				result.mData[i] = 0;
			}
		}
		return result;
	}

	/**
	 * @brief Returns an "identity" matrix with dimensions given by the
	 * class's template parameters.
	 *
	 * In the case that @p Rows != @p Cols, this matrix is simply the
	 * one where the Aii elements for i < min(Rows, Cols) are 1, and all
	 * other elements are 0.
	 *
	 * @return A TMatrix<Rows, Cols, value_type> with off-diagonal
	 * elements set to 0, and diagonal elements set to 1.
	 */
	static TMatrix<Rows, Cols, value_type> identity() {
		TMatrix<Rows, Cols, value_type> id;
		for (uint16_t i = 0; i < Rows && i < Cols; i++) {
			id.element(i,i) = 1;
		}
		return id;
	}

	/**
	 * @brief Returns a ones matrix with dimensions given by the
	 * class's template parameters.
	 *
	 * @return A TMatrix<Rows, Cols, value_type> with all elements set
	 * to 1.
	 */
	static TMatrix<Rows, Cols, value_type> one() {
		TMatrix<Rows, Cols, value_type> ones;
		for (uint16_t i = 0; i < Rows; i++) {
			for (uint16_t j = 0; j < Cols; j++) {
				ones.element(i,j, 1);
			}
		}
		return ones;
	}

	/**
	 * @brief Returns a zero matrix with dimensions given by the
	 * class's template parameters.
	 *
	 * @return A TMatrix<Rows, Cols, value_type> containing all 0.
	 */
	static TMatrix<Rows, Cols, value_type> zero() {
		return TMatrix<Rows, Cols, value_type>();
	}

	value_type* row(uint32_t i) { return &mData[i*Cols]; }
	const value_type* row(uint32_t i) const { return &mData[i*Cols]; }

	/**
	 * @brief Checks to see if any of this matrix's elements are NaN.
	 */
	bool hasNaN(void) const {
		for (uint32_t i = 0; i < Rows*Cols; i++) {
			if (isnan(mData[i])) {
				return true;
			}
		}
		return false;
	}

	void print(Stream & os, bool oneLine = false) const {
		for (uint16_t i = 0; i < Rows; i++) {
			for (uint16_t j = 0; j < Cols; j++) {
				os.printf("%.06f ", element(i, j));
			}

			if(!oneLine)
			{
				os.printf("\n");
			}
		}
	}

private:

	template <uint16_t Rows2, uint16_t Cols2, typename value_type2>
	friend TMatrix<Rows2,Cols2,value_type2> operator*(double s, const TMatrix<Rows2, Cols2, value_type2>& m);
};

typedef TMatrix<2,2, float>  TMatrix2;
typedef TMatrix<3,3, float>  TMatrix3;
typedef TMatrix<4,4, float>  TMatrix4;
typedef TMatrix<2,1, float>  TVector2;
typedef TMatrix<3,1, float>  TVector3;
typedef TMatrix<4,1, float>  TVector4;

// left-side scalar multiply
template <uint16_t Rows, uint16_t Cols, typename value_type>
TMatrix<Rows,Cols,value_type> operator*(double s, const TMatrix<Rows, Cols, value_type>& m) {
	return m * s;
}


#endif /* TMATRIX_H */