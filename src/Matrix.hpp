#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <array>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>

/**
 * The Matrix class is an implementation of the mathematical concept of a matrix (not the movie).
 * @see https://en.wikipedia.org/wiki/Matrix_(mathematics) for more information.
 *
 * typename T: T must be an arithmetic type, i.e. an integral or floating point type
 * const std::size_t M: rows, number of rows of the matrix
 * const std::size_t N: columns, number of columns of the matrix
 */
template< typename T, const std::size_t M /* number of rows */, const std::size_t N /* number of columns */>
class Matrix
{
	public:
		/**
		 * @name Compile-time assertion checking: see http://en.cppreference.com/w/cpp/language/static_assert
		 */
		//@{
		/**
		 *
		 */
		static_assert( std::is_arithmetic<T>::value, "Value T must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");
		/**
		 *
		 */
		static_assert( M > 0 && N > 0, "M (rows) and N (columns) must both be greater than 0");
		//@}
		/**
		 * @name Constructors and destructor
		 */
		//@{
		/**
		 * Default ctor. Initialises all cells with 0 or the given value.
		 */
		explicit Matrix( T value = 0);
		/**
		 * Ctor with a linear list of values that must contain M*N elements
		 */
		explicit Matrix( const std::initializer_list< T >& aList);
		/**
		 * Ctor with a list of lists of values where aList must contain M elements and each list in aList must contain N elements
		 */
		explicit Matrix( const std::initializer_list< std::initializer_list< T > >& aList);
		/**
		 * Cpy ctor
		 */
		Matrix( const Matrix< T, M, N >& aMatrix);
		/**
		 * Dtor
		 */
		virtual ~Matrix() = default;
		//@}
		/**
		 * @name Dimension access
		 */
		//@{
		/**
		 *
		 */
		inline static std::size_t getRows()
		{
			return M;
		}
		/**
		 *
		 */
		inline static std::size_t getColumns()
		{
			return N;
		}
		//@}
		/**
		 * @name Element access
		 */
		//@{
		/**
		 * Returns the row at aRowIndex
		 * If aRowIndex > getRows() an exception of type std::out_of_range is thrown.
		 */
		std::array< T, N >& at( std::size_t aRowIndex);
		/**
		 * Returns the row at aRowIndex
		 * If aRowIndex > getRows() an exception of type std::out_of_range is thrown.
		 */
		const std::array< T, N >& at( std::size_t aRowIndex) const;
		/**
		 * Returns the cell at (aRowIndex,aColumnIndex)
		 * If aRowIndex > getRows() or aColumnIndex > getColumns an exception of type std::out_of_range is thrown.
		 */
		T& at( 	std::size_t aRowIndex,
				std::size_t aColumnIndex);
		/**
		 * Returns the element at (aRowIndex,aColumnIndex)
		 * If aRowIndex > getRows() or aColumnIndex > getColumns an exception of type std::out_of_range is thrown.
		 */
		const T& at( 	std::size_t aRowIndex,
						std::size_t aColumnIndex) const;
		/**
		 * Returns the row at aRowIndex. No range checking is done.
		 */
		std::array< T, N >& operator[]( std::size_t aRowIndex);
		/**
		 * Returns the row at aRowIndex. No range checking is done.
		 */
		const std::array< T, N >& operator[]( std::size_t aRowIndex) const;
		//@}
		/**
		 * @name Matrix operators
		 */
		//@{
		/**
		 * Assignment operator
		 */
		Matrix< T, M, N >& operator=( const Matrix< T, M, N >& rhs);
		/**
		 * Comparison operator
		 */
		bool operator==( const Matrix< T, M, N>& rhs) const;
		//@}
		/**
		 * @name Scalar arithmetic operations supporting only rhs-scalars
		 */
		//@{
		/**
		 *
		 */
		template< class T2 = T >
		Matrix< T, M, N >& operator*=( const T2& scalar);
		/**
		 *
		 */
		template< class T2 = T >
		Matrix< T, M, N > operator*( const T2& scalar) const;
		/**
		 *
		 */
		template< class T2 = T >
		Matrix< T, M, N >& operator/=( const T2& scalar);
		/**
		 *
		 */
		template< class T2 = T >
		Matrix< T, M, N > operator/( const T2& scalar) const;
		//@}
		/**
		 * @name Matrix arithmetic operations
		 */
		//@{
		/**
		 *
		 */
		Matrix< T, M, N >& operator+=( const Matrix< T, M, N >& rhs);
		/**
		 *
		 */
		Matrix< T, M, N > operator+( const Matrix< T, M, N >& rhs) const;
		/**
		 *
		 */
		Matrix< T, M, N >& operator-=( const Matrix< T, M, N >& rhs);
		/**
		 *
		 */
		Matrix< T, M, N > operator-( const Matrix< T, M, N >& rhs) const;
		/**
		 * (M, N) * (N, O) -> (M, O)
		 */
		template< std::size_t columns>
		Matrix< T, M, columns>  operator*( const Matrix< T, N, columns >& rhs) const;
		//@}
		/**
		 * @name Matrix functions
		 */
		//@{
		/**
		 * @see https://en.wikipedia.org/wiki/Transpose
		 */
		Matrix< T, N, M > transpose() const;
		/**
		 * @see https://en.wikipedia.org/wiki/Identity_matrix
		 */
		Matrix< T, M, N > identity() const;
		/**
		 * @see https://en.wikipedia.org/wiki/Gaussian_elimination
		 */
		Matrix< T, M, N > gauss() const;
		/**
		 * @see https://en.wikipedia.org/wiki/Invertible_matrix
		 */
		Matrix< T, M, N > gaussJordan() const;
		/**
		 *
		 */
		Matrix< T, M, 1 > solve() const;

		Matrix< T, 1, N > stochasticRight(const Matrix< T, 1, N >& inputMatrix, long iterations) const;

		Matrix< T, M, 1 > stochasticLeft(const Matrix< T, M, 1 >& inputMatrix, long iterations) const;
		/**
		 * @see https://en.wikipedia.org/wiki/Invertible_matrix
		 */
		Matrix< T, M, N > inverse() const;
		//@}
		/**
		 * @name Other methods
		 */
		//@{
		/**
		 * @return a string representation of the matrix for printing on the screen
		 */
		std::string to_string() const;
		//@}
	private:
		std::array< std::array< T, N >, M > matrix;
};

/**
 *
 */
template< typename T, std::size_t M, std::size_t N >
inline bool operator==( const Matrix< T, M, N >& lhs,
						const Matrix< T, M, N >& rhs)
{
	return lhs.matrix == rhs.matrix;
}
/**
 *
 */
template< typename T, std::size_t M, std::size_t N >
inline std::ostream& operator<<( std::ostream& stream,
								 const Matrix< T, M, N >& aMatrix)
{
	return stream << aMatrix.to_string();
}

/**
 * Compare two row vectors using a aPrecision and a factor. The actual used precision is aPrecission*aFactor.
 *
 */
template< typename T, const std::size_t N >
bool equals(const Matrix< T, 1, N>& lhs,
			const Matrix< T, 1, N>& rhs,
			const T aPrecision = std::numeric_limits<T>::epsilon(),
			const unsigned long aFactor = 1);
/**
 * Compare two column vectors using a aPrecision and a factor. The actual used precision is aPrecission*aFactor.
 */
template< typename T, const std::size_t M>
bool equals(const Matrix< T, M, 1>& lhs,
			const Matrix< T, M, 1>& rhs,
			const T aPrecision = std::numeric_limits<T>::epsilon(),
			const unsigned long aFactor = 1);
/**
 * Compare two matrices using a Precision and a factor. The actual used  precision is aPrecission*aFactor.
 * If all rows are equal according this aPrecission*aFactor true is returned, false otherwise
 */
template< typename T, const std::size_t M, const std::size_t N>
bool equals(const Matrix< T, M, N>& lhs,
			const Matrix< T, M, N>& rhs,
			const T aPrecision = std::numeric_limits<T>::epsilon(),
			const unsigned long aFactor = 1);

template< class T, std::size_t M, std::size_t N>
Matrix< T, M, N > determinePredictedStateVector(Matrix< T, M, M > A, Matrix< T, M, N > stateVector, Matrix< T, M, M > B, Matrix< T, M, N > u ){
	Matrix< T, M, N > result;
	result = (A * stateVector) + (B * u); //TODO:FIX
	return (result);
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > determinePredictedProcessCovarianceMatrix(Matrix< T, M, N > A, Matrix< T, M, N > processCovariance, Matrix< T, M, N > transposedA){
	Matrix< T, M, N > result;
	result = A * processCovariance * transposedA;
	result.at(0, 1) = 0;
	result.at(1, 0) = 0;
	return (result);
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > determineKalmanGain(Matrix< T, M, N > predictedCovarianceMatrix, Matrix< T, M, N > processError, Matrix< T, M, N > C, Matrix< T, M, N > transposedC){
	Matrix< T, M, N > result;
	Matrix<T, M, M> temp = ((C * predictedCovarianceMatrix * transposedC) + processError);
	result = (predictedCovarianceMatrix * transposedC) * temp.inverse();
	return (result);
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > determineAdjustedStateVector(Matrix< T, M, N > predictedStateVector, Matrix< T, M, M > kalman, Matrix< T, M, N > measurement, Matrix< T, M, M > C){
	Matrix< T, M, N > result;
	result = predictedStateVector + (kalman * (measurement - C * predictedStateVector));
	return (result);
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > determineAdjustedProcessCovarianceMatrix(Matrix< T, M, N > I, Matrix< T, M, N > kalman, Matrix< T, M, N > C, Matrix< T, M, N > predictedProcessCovarianceMatrix){
	Matrix< T, M, N > result;
	result = (I - kalman * C) * predictedProcessCovarianceMatrix;
	return (result);
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > determineMeasurementVector(Matrix< T, M, N > C, Matrix< T, M, N > X){
	Matrix< T, M, N > result;
	result = C * X;
	return (result);
}

template< class T, std::size_t M, std::size_t N>
std::pair<Matrix< T, M, N >, Matrix< T, M, M >> kalmanFilter(Matrix<T, M, N>stateVector, Matrix<T, M, M>processCovarianceMatrix, Matrix<T, M, M>A, Matrix<T, M, M>B, Matrix<T, M, M>I, Matrix<T, M , N> measurement, Matrix<T, M, N> u, Matrix<T, M, M> Q){
	Matrix< T, M, N > predictedStateVector = determinePredictedStateVector(A, stateVector, B, u);
	Matrix< T, M, M > predictedCovarianceMatrix = determinePredictedProcessCovarianceMatrix(A, processCovarianceMatrix, A.transpose());

    Matrix<T, M, M>C = predictedCovarianceMatrix.identity();
//    Matrix<T, M, M>Q{ {0.001218471, 0}, {0, 10} };

	Matrix< T, M, M > kalmanGain = determineKalmanGain(predictedCovarianceMatrix, Q, I, I.transpose());
	std::cout << kalmanGain << std::endl;

	Matrix< T, M, N > adjustedStateVector = determineAdjustedStateVector(predictedStateVector, kalmanGain, measurement, C);
	Matrix< T, M, M > adjustedProcessCovarianceMatrix = determineAdjustedProcessCovarianceMatrix(I, kalmanGain, C, predictedCovarianceMatrix);

	return (std::make_pair(adjustedStateVector, adjustedProcessCovarianceMatrix));
}

template<typename T, std::size_t M_state, std::size_t N_state, std::size_t M_tm, std::size_t N_tm>
Matrix<T, M_state, N_state> calc(const Matrix<T, M_state, N_state> &m0, Matrix<T, M_tm, N_tm> &m1, long power){
	Matrix<T, M_tm, N_tm> temp = m1.identity();
	for(int i = 0; i < power; ++i){
		temp = temp * m1;
	}
	Matrix<T, M_state,  N_state> endState = m0 * temp;
	std::cout  << endState << std::endl;
	return endState;
}

#include "Matrix.inc"

#endif /* MATRIX_HPP_ */
