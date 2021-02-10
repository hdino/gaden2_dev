INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

HEADERS += \
    $$PWD/include/Eigen/Cholesky \
    $$PWD/include/Eigen/CholmodSupport \
    $$PWD/include/Eigen/Core \
    $$PWD/include/Eigen/Dense \
    $$PWD/include/Eigen/Eigen \
    $$PWD/include/Eigen/Eigenvalues \
    $$PWD/include/Eigen/Geometry \
    $$PWD/include/Eigen/Householder \
    $$PWD/include/Eigen/IterativeLinearSolvers \
    $$PWD/include/Eigen/Jacobi \
    $$PWD/include/Eigen/KLUSupport \
    $$PWD/include/Eigen/LU \
    $$PWD/include/Eigen/MetisSupport \
    $$PWD/include/Eigen/OrderingMethods \
    $$PWD/include/Eigen/PaStiXSupport \
    $$PWD/include/Eigen/PardisoSupport \
    $$PWD/include/Eigen/QR \
    $$PWD/include/Eigen/QtAlignedMalloc \
    $$PWD/include/Eigen/SPQRSupport \
    $$PWD/include/Eigen/SVD \
    $$PWD/include/Eigen/Sparse \
    $$PWD/include/Eigen/SparseCholesky \
    $$PWD/include/Eigen/SparseCore \
    $$PWD/include/Eigen/SparseLU \
    $$PWD/include/Eigen/SparseQR \
    $$PWD/include/Eigen/StdDeque \
    $$PWD/include/Eigen/StdList \
    $$PWD/include/Eigen/StdVector \
    $$PWD/include/Eigen/SuperLUSupport \
    $$PWD/include/Eigen/UmfPackSupport \
    $$PWD/include/Eigen/src/Cholesky/LDLT.h \
    $$PWD/include/Eigen/src/Cholesky/LLT.h \
    $$PWD/include/Eigen/src/Cholesky/LLT_LAPACKE.h \
    $$PWD/include/Eigen/src/CholmodSupport/CholmodSupport.h \
    $$PWD/include/Eigen/src/Core/ArithmeticSequence.h \
    $$PWD/include/Eigen/src/Core/Array.h \
    $$PWD/include/Eigen/src/Core/ArrayBase.h \
    $$PWD/include/Eigen/src/Core/ArrayWrapper.h \
    $$PWD/include/Eigen/src/Core/Assign.h \
    $$PWD/include/Eigen/src/Core/AssignEvaluator.h \
    $$PWD/include/Eigen/src/Core/Assign_MKL.h \
    $$PWD/include/Eigen/src/Core/BandMatrix.h \
    $$PWD/include/Eigen/src/Core/Block.h \
    $$PWD/include/Eigen/src/Core/BooleanRedux.h \
    $$PWD/include/Eigen/src/Core/CommaInitializer.h \
    $$PWD/include/Eigen/src/Core/ConditionEstimator.h \
    $$PWD/include/Eigen/src/Core/CoreEvaluators.h \
    $$PWD/include/Eigen/src/Core/CoreIterators.h \
    $$PWD/include/Eigen/src/Core/CwiseBinaryOp.h \
    $$PWD/include/Eigen/src/Core/CwiseNullaryOp.h \
    $$PWD/include/Eigen/src/Core/CwiseTernaryOp.h \
    $$PWD/include/Eigen/src/Core/CwiseUnaryOp.h \
    $$PWD/include/Eigen/src/Core/CwiseUnaryView.h \
    $$PWD/include/Eigen/src/Core/DenseBase.h \
    $$PWD/include/Eigen/src/Core/DenseCoeffsBase.h \
    $$PWD/include/Eigen/src/Core/DenseStorage.h \
    $$PWD/include/Eigen/src/Core/Diagonal.h \
    $$PWD/include/Eigen/src/Core/DiagonalMatrix.h \
    $$PWD/include/Eigen/src/Core/DiagonalProduct.h \
    $$PWD/include/Eigen/src/Core/Dot.h \
    $$PWD/include/Eigen/src/Core/EigenBase.h \
    $$PWD/include/Eigen/src/Core/ForceAlignedAccess.h \
    $$PWD/include/Eigen/src/Core/Fuzzy.h \
    $$PWD/include/Eigen/src/Core/GeneralProduct.h \
    $$PWD/include/Eigen/src/Core/GenericPacketMath.h \
    $$PWD/include/Eigen/src/Core/GlobalFunctions.h \
    $$PWD/include/Eigen/src/Core/IO.h \
    $$PWD/include/Eigen/src/Core/IndexedView.h \
    $$PWD/include/Eigen/src/Core/Inverse.h \
    $$PWD/include/Eigen/src/Core/Map.h \
    $$PWD/include/Eigen/src/Core/MapBase.h \
    $$PWD/include/Eigen/src/Core/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/MathFunctionsImpl.h \
    $$PWD/include/Eigen/src/Core/Matrix.h \
    $$PWD/include/Eigen/src/Core/MatrixBase.h \
    $$PWD/include/Eigen/src/Core/NestByValue.h \
    $$PWD/include/Eigen/src/Core/NoAlias.h \
    $$PWD/include/Eigen/src/Core/NumTraits.h \
    $$PWD/include/Eigen/src/Core/PartialReduxEvaluator.h \
    $$PWD/include/Eigen/src/Core/PermutationMatrix.h \
    $$PWD/include/Eigen/src/Core/PlainObjectBase.h \
    $$PWD/include/Eigen/src/Core/Product.h \
    $$PWD/include/Eigen/src/Core/ProductEvaluators.h \
    $$PWD/include/Eigen/src/Core/Random.h \
    $$PWD/include/Eigen/src/Core/Redux.h \
    $$PWD/include/Eigen/src/Core/Ref.h \
    $$PWD/include/Eigen/src/Core/Replicate.h \
    $$PWD/include/Eigen/src/Core/Reshaped.h \
    $$PWD/include/Eigen/src/Core/ReturnByValue.h \
    $$PWD/include/Eigen/src/Core/Reverse.h \
    $$PWD/include/Eigen/src/Core/Select.h \
    $$PWD/include/Eigen/src/Core/SelfAdjointView.h \
    $$PWD/include/Eigen/src/Core/SelfCwiseBinaryOp.h \
    $$PWD/include/Eigen/src/Core/Solve.h \
    $$PWD/include/Eigen/src/Core/SolveTriangular.h \
    $$PWD/include/Eigen/src/Core/SolverBase.h \
    $$PWD/include/Eigen/src/Core/StableNorm.h \
    $$PWD/include/Eigen/src/Core/StlIterators.h \
    $$PWD/include/Eigen/src/Core/Stride.h \
    $$PWD/include/Eigen/src/Core/Swap.h \
    $$PWD/include/Eigen/src/Core/Transpose.h \
    $$PWD/include/Eigen/src/Core/Transpositions.h \
    $$PWD/include/Eigen/src/Core/TriangularMatrix.h \
    $$PWD/include/Eigen/src/Core/VectorBlock.h \
    $$PWD/include/Eigen/src/Core/VectorwiseOp.h \
    $$PWD/include/Eigen/src/Core/Visitor.h \
    $$PWD/include/Eigen/src/Core/arch/AVX/Complex.h \
    $$PWD/include/Eigen/src/Core/arch/AVX/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/AVX/PacketMath.h \
    $$PWD/include/Eigen/src/Core/arch/AVX/TypeCasting.h \
    $$PWD/include/Eigen/src/Core/arch/AVX512/Complex.h \
    $$PWD/include/Eigen/src/Core/arch/AVX512/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/AVX512/PacketMath.h \
    $$PWD/include/Eigen/src/Core/arch/AVX512/TypeCasting.h \
    $$PWD/include/Eigen/src/Core/arch/AltiVec/Complex.h \
    $$PWD/include/Eigen/src/Core/arch/AltiVec/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/AltiVec/MatrixProduct.h \
    $$PWD/include/Eigen/src/Core/arch/AltiVec/PacketMath.h \
    $$PWD/include/Eigen/src/Core/arch/CUDA/Complex.h \
    $$PWD/include/Eigen/src/Core/arch/Default/BFloat16.h \
    $$PWD/include/Eigen/src/Core/arch/Default/ConjHelper.h \
    $$PWD/include/Eigen/src/Core/arch/Default/GenericPacketMathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/Default/GenericPacketMathFunctionsFwd.h \
    $$PWD/include/Eigen/src/Core/arch/Default/Half.h \
    $$PWD/include/Eigen/src/Core/arch/Default/Settings.h \
    $$PWD/include/Eigen/src/Core/arch/Default/TypeCasting.h \
    $$PWD/include/Eigen/src/Core/arch/GPU/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/GPU/PacketMath.h \
    $$PWD/include/Eigen/src/Core/arch/GPU/TypeCasting.h \
    $$PWD/include/Eigen/src/Core/arch/HIP/hcc/math_constants.h \
    $$PWD/include/Eigen/src/Core/arch/MSA/Complex.h \
    $$PWD/include/Eigen/src/Core/arch/MSA/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/MSA/PacketMath.h \
    $$PWD/include/Eigen/src/Core/arch/NEON/Complex.h \
    $$PWD/include/Eigen/src/Core/arch/NEON/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/NEON/PacketMath.h \
    $$PWD/include/Eigen/src/Core/arch/NEON/TypeCasting.h \
    $$PWD/include/Eigen/src/Core/arch/SSE/Complex.h \
    $$PWD/include/Eigen/src/Core/arch/SSE/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/SSE/PacketMath.h \
    $$PWD/include/Eigen/src/Core/arch/SSE/TypeCasting.h \
    $$PWD/include/Eigen/src/Core/arch/SYCL/InteropHeaders.h \
    $$PWD/include/Eigen/src/Core/arch/SYCL/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/SYCL/PacketMath.h \
    $$PWD/include/Eigen/src/Core/arch/SYCL/SyclMemoryModel.h \
    $$PWD/include/Eigen/src/Core/arch/SYCL/TypeCasting.h \
    $$PWD/include/Eigen/src/Core/arch/ZVector/Complex.h \
    $$PWD/include/Eigen/src/Core/arch/ZVector/MathFunctions.h \
    $$PWD/include/Eigen/src/Core/arch/ZVector/PacketMath.h \
    $$PWD/include/Eigen/src/Core/functors/AssignmentFunctors.h \
    $$PWD/include/Eigen/src/Core/functors/BinaryFunctors.h \
    $$PWD/include/Eigen/src/Core/functors/NullaryFunctors.h \
    $$PWD/include/Eigen/src/Core/functors/StlFunctors.h \
    $$PWD/include/Eigen/src/Core/functors/TernaryFunctors.h \
    $$PWD/include/Eigen/src/Core/functors/UnaryFunctors.h \
    $$PWD/include/Eigen/src/Core/products/GeneralBlockPanelKernel.h \
    $$PWD/include/Eigen/src/Core/products/GeneralMatrixMatrix.h \
    $$PWD/include/Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h \
    $$PWD/include/Eigen/src/Core/products/GeneralMatrixMatrixTriangular_BLAS.h \
    $$PWD/include/Eigen/src/Core/products/GeneralMatrixMatrix_BLAS.h \
    $$PWD/include/Eigen/src/Core/products/GeneralMatrixVector.h \
    $$PWD/include/Eigen/src/Core/products/GeneralMatrixVector_BLAS.h \
    $$PWD/include/Eigen/src/Core/products/Parallelizer.h \
    $$PWD/include/Eigen/src/Core/products/SelfadjointMatrixMatrix.h \
    $$PWD/include/Eigen/src/Core/products/SelfadjointMatrixMatrix_BLAS.h \
    $$PWD/include/Eigen/src/Core/products/SelfadjointMatrixVector.h \
    $$PWD/include/Eigen/src/Core/products/SelfadjointMatrixVector_BLAS.h \
    $$PWD/include/Eigen/src/Core/products/SelfadjointProduct.h \
    $$PWD/include/Eigen/src/Core/products/SelfadjointRank2Update.h \
    $$PWD/include/Eigen/src/Core/products/TriangularMatrixMatrix.h \
    $$PWD/include/Eigen/src/Core/products/TriangularMatrixMatrix_BLAS.h \
    $$PWD/include/Eigen/src/Core/products/TriangularMatrixVector.h \
    $$PWD/include/Eigen/src/Core/products/TriangularMatrixVector_BLAS.h \
    $$PWD/include/Eigen/src/Core/products/TriangularSolverMatrix.h \
    $$PWD/include/Eigen/src/Core/products/TriangularSolverMatrix_BLAS.h \
    $$PWD/include/Eigen/src/Core/products/TriangularSolverVector.h \
    $$PWD/include/Eigen/src/Core/util/BlasUtil.h \
    $$PWD/include/Eigen/src/Core/util/ConfigureVectorization.h \
    $$PWD/include/Eigen/src/Core/util/Constants.h \
    $$PWD/include/Eigen/src/Core/util/DisableStupidWarnings.h \
    $$PWD/include/Eigen/src/Core/util/ForwardDeclarations.h \
    $$PWD/include/Eigen/src/Core/util/IndexedViewHelper.h \
    $$PWD/include/Eigen/src/Core/util/IntegralConstant.h \
    $$PWD/include/Eigen/src/Core/util/MKL_support.h \
    $$PWD/include/Eigen/src/Core/util/Macros.h \
    $$PWD/include/Eigen/src/Core/util/Memory.h \
    $$PWD/include/Eigen/src/Core/util/Meta.h \
    $$PWD/include/Eigen/src/Core/util/NonMPL2.h \
    $$PWD/include/Eigen/src/Core/util/ReenableStupidWarnings.h \
    $$PWD/include/Eigen/src/Core/util/ReshapedHelper.h \
    $$PWD/include/Eigen/src/Core/util/StaticAssert.h \
    $$PWD/include/Eigen/src/Core/util/SymbolicIndex.h \
    $$PWD/include/Eigen/src/Core/util/XprHelper.h \
    $$PWD/include/Eigen/src/Eigenvalues/ComplexEigenSolver.h \
    $$PWD/include/Eigen/src/Eigenvalues/ComplexSchur.h \
    $$PWD/include/Eigen/src/Eigenvalues/ComplexSchur_LAPACKE.h \
    $$PWD/include/Eigen/src/Eigenvalues/EigenSolver.h \
    $$PWD/include/Eigen/src/Eigenvalues/GeneralizedEigenSolver.h \
    $$PWD/include/Eigen/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h \
    $$PWD/include/Eigen/src/Eigenvalues/HessenbergDecomposition.h \
    $$PWD/include/Eigen/src/Eigenvalues/MatrixBaseEigenvalues.h \
    $$PWD/include/Eigen/src/Eigenvalues/RealQZ.h \
    $$PWD/include/Eigen/src/Eigenvalues/RealSchur.h \
    $$PWD/include/Eigen/src/Eigenvalues/RealSchur_LAPACKE.h \
    $$PWD/include/Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h \
    $$PWD/include/Eigen/src/Eigenvalues/SelfAdjointEigenSolver_LAPACKE.h \
    $$PWD/include/Eigen/src/Eigenvalues/Tridiagonalization.h \
    $$PWD/include/Eigen/src/Geometry/AlignedBox.h \
    $$PWD/include/Eigen/src/Geometry/AngleAxis.h \
    $$PWD/include/Eigen/src/Geometry/EulerAngles.h \
    $$PWD/include/Eigen/src/Geometry/Homogeneous.h \
    $$PWD/include/Eigen/src/Geometry/Hyperplane.h \
    $$PWD/include/Eigen/src/Geometry/OrthoMethods.h \
    $$PWD/include/Eigen/src/Geometry/ParametrizedLine.h \
    $$PWD/include/Eigen/src/Geometry/Quaternion.h \
    $$PWD/include/Eigen/src/Geometry/Rotation2D.h \
    $$PWD/include/Eigen/src/Geometry/RotationBase.h \
    $$PWD/include/Eigen/src/Geometry/Scaling.h \
    $$PWD/include/Eigen/src/Geometry/Transform.h \
    $$PWD/include/Eigen/src/Geometry/Translation.h \
    $$PWD/include/Eigen/src/Geometry/Umeyama.h \
    $$PWD/include/Eigen/src/Geometry/arch/Geometry_SSE.h \
    $$PWD/include/Eigen/src/Householder/BlockHouseholder.h \
    $$PWD/include/Eigen/src/Householder/Householder.h \
    $$PWD/include/Eigen/src/Householder/HouseholderSequence.h \
    $$PWD/include/Eigen/src/IterativeLinearSolvers/BasicPreconditioners.h \
    $$PWD/include/Eigen/src/IterativeLinearSolvers/BiCGSTAB.h \
    $$PWD/include/Eigen/src/IterativeLinearSolvers/ConjugateGradient.h \
    $$PWD/include/Eigen/src/IterativeLinearSolvers/IncompleteCholesky.h \
    $$PWD/include/Eigen/src/IterativeLinearSolvers/IncompleteLUT.h \
    $$PWD/include/Eigen/src/IterativeLinearSolvers/IterativeSolverBase.h \
    $$PWD/include/Eigen/src/IterativeLinearSolvers/LeastSquareConjugateGradient.h \
    $$PWD/include/Eigen/src/IterativeLinearSolvers/SolveWithGuess.h \
    $$PWD/include/Eigen/src/Jacobi/Jacobi.h \
    $$PWD/include/Eigen/src/KLUSupport/KLUSupport.h \
    $$PWD/include/Eigen/src/LU/Determinant.h \
    $$PWD/include/Eigen/src/LU/FullPivLU.h \
    $$PWD/include/Eigen/src/LU/InverseImpl.h \
    $$PWD/include/Eigen/src/LU/PartialPivLU.h \
    $$PWD/include/Eigen/src/LU/PartialPivLU_LAPACKE.h \
    $$PWD/include/Eigen/src/LU/arch/Inverse_NEON.h \
    $$PWD/include/Eigen/src/LU/arch/Inverse_SSE.h \
    $$PWD/include/Eigen/src/MetisSupport/MetisSupport.h \
    $$PWD/include/Eigen/src/OrderingMethods/Amd.h \
    $$PWD/include/Eigen/src/OrderingMethods/Eigen_Colamd.h \
    $$PWD/include/Eigen/src/OrderingMethods/Ordering.h \
    $$PWD/include/Eigen/src/PaStiXSupport/PaStiXSupport.h \
    $$PWD/include/Eigen/src/PardisoSupport/PardisoSupport.h \
    $$PWD/include/Eigen/src/QR/ColPivHouseholderQR.h \
    $$PWD/include/Eigen/src/QR/ColPivHouseholderQR_LAPACKE.h \
    $$PWD/include/Eigen/src/QR/CompleteOrthogonalDecomposition.h \
    $$PWD/include/Eigen/src/QR/FullPivHouseholderQR.h \
    $$PWD/include/Eigen/src/QR/HouseholderQR.h \
    $$PWD/include/Eigen/src/QR/HouseholderQR_LAPACKE.h \
    $$PWD/include/Eigen/src/SPQRSupport/SuiteSparseQRSupport.h \
    $$PWD/include/Eigen/src/SVD/BDCSVD.h \
    $$PWD/include/Eigen/src/SVD/JacobiSVD.h \
    $$PWD/include/Eigen/src/SVD/JacobiSVD_LAPACKE.h \
    $$PWD/include/Eigen/src/SVD/SVDBase.h \
    $$PWD/include/Eigen/src/SVD/UpperBidiagonalization.h \
    $$PWD/include/Eigen/src/SparseCholesky/SimplicialCholesky.h \
    $$PWD/include/Eigen/src/SparseCholesky/SimplicialCholesky_impl.h \
    $$PWD/include/Eigen/src/SparseCore/AmbiVector.h \
    $$PWD/include/Eigen/src/SparseCore/CompressedStorage.h \
    $$PWD/include/Eigen/src/SparseCore/ConservativeSparseSparseProduct.h \
    $$PWD/include/Eigen/src/SparseCore/MappedSparseMatrix.h \
    $$PWD/include/Eigen/src/SparseCore/SparseAssign.h \
    $$PWD/include/Eigen/src/SparseCore/SparseBlock.h \
    $$PWD/include/Eigen/src/SparseCore/SparseColEtree.h \
    $$PWD/include/Eigen/src/SparseCore/SparseCompressedBase.h \
    $$PWD/include/Eigen/src/SparseCore/SparseCwiseBinaryOp.h \
    $$PWD/include/Eigen/src/SparseCore/SparseCwiseUnaryOp.h \
    $$PWD/include/Eigen/src/SparseCore/SparseDenseProduct.h \
    $$PWD/include/Eigen/src/SparseCore/SparseDiagonalProduct.h \
    $$PWD/include/Eigen/src/SparseCore/SparseDot.h \
    $$PWD/include/Eigen/src/SparseCore/SparseFuzzy.h \
    $$PWD/include/Eigen/src/SparseCore/SparseMap.h \
    $$PWD/include/Eigen/src/SparseCore/SparseMatrix.h \
    $$PWD/include/Eigen/src/SparseCore/SparseMatrixBase.h \
    $$PWD/include/Eigen/src/SparseCore/SparsePermutation.h \
    $$PWD/include/Eigen/src/SparseCore/SparseProduct.h \
    $$PWD/include/Eigen/src/SparseCore/SparseRedux.h \
    $$PWD/include/Eigen/src/SparseCore/SparseRef.h \
    $$PWD/include/Eigen/src/SparseCore/SparseSelfAdjointView.h \
    $$PWD/include/Eigen/src/SparseCore/SparseSolverBase.h \
    $$PWD/include/Eigen/src/SparseCore/SparseSparseProductWithPruning.h \
    $$PWD/include/Eigen/src/SparseCore/SparseTranspose.h \
    $$PWD/include/Eigen/src/SparseCore/SparseTriangularView.h \
    $$PWD/include/Eigen/src/SparseCore/SparseUtil.h \
    $$PWD/include/Eigen/src/SparseCore/SparseVector.h \
    $$PWD/include/Eigen/src/SparseCore/SparseView.h \
    $$PWD/include/Eigen/src/SparseCore/TriangularSolver.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLUImpl.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_Memory.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_Structs.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_SupernodalMatrix.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_Utils.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_column_bmod.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_column_dfs.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_copy_to_ucol.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_gemm_kernel.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_heap_relax_snode.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_kernel_bmod.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_panel_bmod.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_panel_dfs.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_pivotL.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_pruneL.h \
    $$PWD/include/Eigen/src/SparseLU/SparseLU_relax_snode.h \
    $$PWD/include/Eigen/src/SparseQR/SparseQR.h \
    $$PWD/include/Eigen/src/StlSupport/StdDeque.h \
    $$PWD/include/Eigen/src/StlSupport/StdList.h \
    $$PWD/include/Eigen/src/StlSupport/StdVector.h \
    $$PWD/include/Eigen/src/StlSupport/details.h \
    $$PWD/include/Eigen/src/SuperLUSupport/SuperLUSupport.h \
    $$PWD/include/Eigen/src/UmfPackSupport/UmfPackSupport.h \
    $$PWD/include/Eigen/src/misc/Image.h \
    $$PWD/include/Eigen/src/misc/Kernel.h \
    $$PWD/include/Eigen/src/misc/RealSvd2x2.h \
    $$PWD/include/Eigen/src/misc/blas.h \
    $$PWD/include/Eigen/src/misc/lapack.h \
    $$PWD/include/Eigen/src/misc/lapacke.h \
    $$PWD/include/Eigen/src/misc/lapacke_mangling.h \
    $$PWD/include/Eigen/src/plugins/ArrayCwiseBinaryOps.h \
    $$PWD/include/Eigen/src/plugins/ArrayCwiseUnaryOps.h \
    $$PWD/include/Eigen/src/plugins/BlockMethods.h \
    $$PWD/include/Eigen/src/plugins/CommonCwiseBinaryOps.h \
    $$PWD/include/Eigen/src/plugins/CommonCwiseUnaryOps.h \
    $$PWD/include/Eigen/src/plugins/IndexedViewMethods.h \
    $$PWD/include/Eigen/src/plugins/MatrixCwiseBinaryOps.h \
    $$PWD/include/Eigen/src/plugins/MatrixCwiseUnaryOps.h \
    $$PWD/include/Eigen/src/plugins/ReshapedMethods.h
