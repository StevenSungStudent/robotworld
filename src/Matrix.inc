#include <cassert>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <utility>
#include <iomanip>

#include <iostream>
#include <algorithm>
#include <iomanip>

/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( T value)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) = value;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< T >& aList)
{
	// Check the arguments
	assert( aList.size() == M * N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column, ++row_iter)
		{
			matrix.at( row).at( column) = *row_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const std::initializer_list< std::initializer_list< T > >& aList)
{
	// Check the arguments, the static assert assures that there is at least 1 M and 1 N!
	assert( aList.size() == M && (*aList.begin()).size() == N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
	{
		auto column_iter = (*row_iter).begin();
		for (std::size_t column = 0; column < (*row_iter).size(); ++column, ++column_iter)
		{
			matrix.at( row).at( column) = *column_iter;
		}
	}
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >::Matrix( const Matrix< T, M, N >& aMatrix) :
				matrix( aMatrix.matrix)
{
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex)
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::at( std::size_t aRowIndex) const
{
	return matrix.at( aRowIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
T& Matrix< T, M, N >::at( 	std::size_t aRowIndex,
							std::size_t aColumnIndex)
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const T& Matrix< T, M, N >::at( std::size_t aRowIndex,
								std::size_t aColumnIndex) const
{
	return matrix.at( aRowIndex).at( aColumnIndex);
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex)
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
const std::array< T, N >& Matrix< T, M, N >::operator[]( std::size_t aRowIndex) const
{
	return matrix[aRowIndex];
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator=( const Matrix< T, M, N >& rhs)
{
	if (this != &rhs)
	{
		matrix = rhs.matrix;
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
bool Matrix< T, M, N >::operator==( const Matrix< T, M, N >& rhs) const
{
	return matrix == rhs.matrix;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator*=( const T2& scalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) *= scalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator*( const T2& scalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result *= scalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N >& Matrix< T, M, N >::operator/=( const T2& aScalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at( row).at( column) /= aScalar;
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
template< class T2 >
Matrix< T, M, N > Matrix< T, M, N >::operator/( const T2& aScalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix< T, M, N > result( *this);
	return result /= aScalar;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator+=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] += rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator+( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result += rhs;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N >& Matrix< T, M, N >::operator-=( const Matrix< T, M, N >& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] -= rhs.at( row, column);
		}
	}
	return *this;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::operator-( const Matrix< T, M, N >& rhs) const
{
	Matrix< T, M, N > result( *this);
	return result -= rhs;
}
/**
 * (M, N) * (N, P) -> (M, P)
 */
template< typename T, std::size_t M, std::size_t N >
template< std::size_t columns >
Matrix< T, M, columns > Matrix< T, M, N >::operator*( const Matrix< T, N, columns >& rhs) const
{
	Matrix< T, M, columns > result;

	for(std::size_t resultColumn = 0; resultColumn < columns; ++resultColumn){
		for(std::size_t row = 0; row < M; ++row){

			T temp = 0;
			for(std::size_t column = 0; column < N; ++column){
				temp += matrix[row][column] * rhs.at(column, resultColumn);
			}
			result.at(row, resultColumn) = temp;
		}
	}

	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, N, M > Matrix< T, M, N >::transpose() const
{
	Matrix< T, N, M > result;
	for(std::size_t column = 0; column < N; ++column){
		for(std::size_t row = 0; row < M; ++row){
			result.at(column,row) = matrix[row][column];
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::identity() const
{
	Matrix< T, M, N > result;

	for(std::size_t row = 0; row < M; ++row){
		for(std::size_t column = 0; column < N; ++column){
			if(row == column ){
				result.at(row, column) = 1;
			}else{
				result.at(row, column) = 0;
			}
		}
	}

	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::gauss() const
{
	Matrix< T, M, N > result = *this;

	std::size_t currentRow = 0;

	for(std::size_t column = 0; column < M; ++column){
		std::size_t pivotCellColumn = column;
		std::size_t pivotCellRow = column;

		//Determine pivot row
		for(std::size_t row = 0; row < M; ++row){
			if(abs(result.at(pivotCellRow, column)) < abs(result.at(row, column))){
				pivotCellRow = row;
				pivotCellColumn = column;
			}
		}

		//Divide and Swap pivot row
		T divider = result.at(pivotCellRow, pivotCellColumn);
		for(std::size_t sourceColumn = 0; sourceColumn < N; ++sourceColumn){

			T tempValue = result.at(pivotCellRow, sourceColumn) / divider;

			result.at(pivotCellRow, sourceColumn) = result.at(currentRow, sourceColumn);
			result.at(currentRow, sourceColumn) = tempValue;

		}
		pivotCellRow = currentRow;
		currentRow += 1;

		//Solve the other rows with the solved pivot row
		for(std::size_t solveRow = currentRow ; solveRow < M; ++solveRow){
			T multiplier = result.at(solveRow, column);
			for(std::size_t solveColumn = 0; solveColumn < N; ++solveColumn){
				result.at(solveRow, solveColumn) -= multiplier * result.at(pivotCellRow, solveColumn);

			}
		}
	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::gaussJordan() const
{
	Matrix< T, M, N > result = *this;

	for(std::size_t column = 0; column < M; ++column){
		std::size_t pivotCellColumn = column;
		std::size_t pivotCellRow = column;
		//Determine pivot row
		for(std::size_t row = 0; row < M; ++row){
			if(abs(result.at(pivotCellRow, column)) < abs(result.at(row, column))){
				pivotCellRow = row;
				pivotCellColumn = column;
			}
		}

		//Divide and Swap pivot row
		T divider = result.at(pivotCellRow, pivotCellColumn);
		for(std::size_t sourceColumn = 0; sourceColumn < N; ++sourceColumn){

			T tempValue = result.at(pivotCellRow, sourceColumn) / divider;

			result.at(pivotCellRow, sourceColumn) = result.at(column, sourceColumn);
			result.at(column, sourceColumn) = tempValue;

		}
		pivotCellRow = column;


		//Solve the other rows with the solved pivot row
		for(std::size_t solveRow = 0 ; solveRow < M; ++solveRow){
			T multiplier = result.at(solveRow, column);
			if(solveRow != column){
				for(std::size_t solveColumn = 0; solveColumn < N; ++solveColumn){
					result.at(solveRow, solveColumn) -= multiplier * result.at(pivotCellRow, solveColumn);
				}
			}
		}
	}
	return result;
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, 1 > Matrix< T, M, N >::solve() const
{
	Matrix < T, M, 1 > result;
	Matrix < T, M, N > gaussJordainMatrix = gaussJordan();
	for(std::size_t row = 0; row < M; ++row){
		result.at(row, 0) = gaussJordainMatrix[row][M];
	}

	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
Matrix< T, M, N > Matrix< T, M, N >::inverse() const
{
	Matrix< T, M, N > result;
	Matrix< T, M, N > identityMatrix = identity();
	Matrix< T, M, N + N > inverseMatrix;

	//Concatenate the regular matrix with the identity matrix
	for(std::size_t row = 0; row < M; ++row){
		for(std::size_t column = 0; column < N; ++column){
			inverseMatrix[row][column] = matrix[row][column];
			inverseMatrix[row][column + N] = identityMatrix[row][column];
		}
	}

	for(std::size_t column = 0; column < M; ++column){

		//Determine pivot row
		std::size_t pivotCellColumn = column;
		std::size_t pivotCellRow = column;
		for(std::size_t row = column; row < M; ++row){
			if(abs(inverseMatrix.at(pivotCellRow, column)) < abs(inverseMatrix.at(row, column))){
				pivotCellRow = row;
				pivotCellColumn = column;
			}
		}

		//Divide and Swap pivot row
		T divider = inverseMatrix.at(pivotCellRow, pivotCellColumn);
		for(std::size_t sourceColumn = 0; sourceColumn < N + N; ++sourceColumn){

			T tempValue = inverseMatrix.at(pivotCellRow, sourceColumn) / divider;
			inverseMatrix.at(pivotCellRow, sourceColumn) = inverseMatrix.at(column, sourceColumn);
			inverseMatrix.at(column, sourceColumn) = tempValue;

		}

	    pivotCellRow = column;

		//Solve the other rows with the solved pivot row
		for(std::size_t solveRow = 0 ; solveRow < M; ++solveRow){
			T multiplier = inverseMatrix.at(solveRow, column);
			if(solveRow != column){
				for(std::size_t solveColumn = 0; solveColumn < N + N; ++solveColumn){
					inverseMatrix.at(solveRow, solveColumn) -= multiplier * inverseMatrix.at(pivotCellRow, solveColumn);
				}
			}
		}
	}

	for(std::size_t row = 0; row < M; ++row){
		for(std::size_t column = 0; column < N; ++column){
			result[row][column] = inverseMatrix[row][column + N];
		}
	}

	return result;
}

template< class T, std::size_t M, std::size_t N >
Matrix< T, 1, N > Matrix< T, M, N >::stochasticRight(const Matrix< T, 1, N >& inputMatrix,long iterations) const{
	Matrix<T, 1, N> result = inputMatrix;

//	for(long i = 0; i < iterations; ++i){
//		result = result * matrix;
//	}

	return result;

}

template< class T, std::size_t M, std::size_t N >
Matrix< T, M, 1 > Matrix< T, M, N >::stochasticLeft(const Matrix< T, M, 1 >& inputMatrix, long iterations) const{
	Matrix<T, M, 1> result = inputMatrix;

//	for(long i = 0; i < iterations; ++i){
//		result = matrix.transpose() * result;
//	}
	return result;
}
/**
 *
 */
template< class T, std::size_t M, std::size_t N >
std::string Matrix< T, M, N >::to_string() const
{
	std::string result = "Matrix<" + std::to_string( N) + "," + std::to_string( M) + ">\n{\n";
	for (std::size_t i = 0; i < M; ++i)
	{
		for (std::size_t j = 0; j < N; ++j)
		{
			result += std::to_string( matrix[i][j]) + ",";
		}
		result += "\n";
	}
	result += "}";
	return result;
}
/**
 *
 */
template< typename T, const std::size_t N >
bool equals(	const Matrix< T, 1, N >& lhs,
				const Matrix< T, 1, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{
	//   Implement this function
	for(std::size_t column = 0; column < N; ++column){
		if(std::abs(lhs[0][column] - rhs[0][column]) > aPrecision * aFactor){
			return false;
		}
	}
	return true;
}
/**
 *
 */
template< typename T, const std::size_t M >
bool equals(	const Matrix< T, M, 1 >& lhs,
				const Matrix< T, M, 1 >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{

	for(std::size_t row = 0; row < M; ++row){
		if(std::abs(lhs[row][0] - rhs[row][0]) > aPrecision * aFactor){
			return false;
		}
	}
	return true;
}
/**
 *
 */
template< typename T, const std::size_t M, const std::size_t N >
bool equals(	const Matrix< T, M, N >& lhs,
				const Matrix< T, M, N >& rhs,
				const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
				const unsigned long aFactor /*= 1*/)
{


	for(std::size_t column = 0; column < N; ++column){
		for(std::size_t row = 0; row < M; ++row){
			if(std::abs(lhs[row][column] - rhs[row][column]) > aPrecision * aFactor){
				return false;
			}
		}
	}

	return true;
}
