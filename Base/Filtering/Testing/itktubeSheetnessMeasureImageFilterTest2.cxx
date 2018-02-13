/*=========================================================================

Library:   TubeTK

Copyright 2010 Kitware Inc. 28 Corporate Drive,
Clifton Park, NY, 12065, USA.

All rights reserved.

Licensed under the Apache License, Version 2.0 ( the "License" );
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

=========================================================================*/

#include "itktubeSheetnessMeasureImageFilter.h"
#include "itktubeSymmetricEigenVectorAnalysisImageFilter.h"

#include <itkHessianRecursiveGaussianImageFilter.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>

int itktubeSheetnessMeasureImageFilterTest2( int argc, char * argv[] )
{
  if( argc < 4 )
    {
    std::cerr << "Missing arguments." << std::endl;
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << "  inputImage sheetnessImage";
    std::cerr << " primaryEigenVectorOutputImage [sigma]";
    std::cerr << " [sheetnessThresholdValue]"<< std::endl;
    return EXIT_FAILURE;
    }

  // Define the dimension of the images
  enum { Dimension = 3 };

  // Define the pixel type
  using PixelType = short;

  // Declare the types of the images
  using InputImageType = itk::Image<PixelType, Dimension>;

  // Declare the reader
  using ReaderType = itk::ImageFileReader< InputImageType >;

  // Create the reader and writer
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( argv[1] );
  reader->Update();

  // Declare the type for the Hessian filter
  using HessianFilterType = itk::HessianRecursiveGaussianImageFilter<
                                            InputImageType >;

  // Declare the type for the sheetness measure filter
  using SheetnessFilterType = itk::tube::SheetnessMeasureImageFilter< float >;

  // Create a Hessian Filter
  HessianFilterType::Pointer filterHessian = HessianFilterType::New();

  // Create a sheetness Filter
  SheetnessFilterType::Pointer filterSheetness = SheetnessFilterType::New();

  // Connect the input images
  filterHessian->SetInput( reader->GetOutput() );
  filterSheetness->SetInput( filterHessian->GetOutput() );

  //Select the value of Sigma for the Hessian computation
  double sigma = 0.5;
  if( argc > 4 )
    {
    sigma = atof ( argv[4] );
    std::cout << "Setting the sigma value....:\t" << sigma << std::endl;
    }
  filterHessian->SetSigma( sigma );

  // Execute the filter
  std::cout << "Generate sheetness measure" << std::endl;
  filterSheetness->Update();

  //Write out the sheetness image
  //Define output type
  using SheetnessImageType = SheetnessFilterType::OutputImageType;

  std::cout << "Write out the sheetness image" << std::endl;
  using SheetnessImageWriterType = itk::ImageFileWriter<SheetnessImageType>;
  SheetnessImageWriterType::Pointer writer= SheetnessImageWriterType::New();
  writer->SetFileName( argv[2] );
  writer->SetInput( filterSheetness->GetOutput() );
  writer->Update();

  //Set sheetness threshold
  double sheetnessThresholdValue = 0.0;
  if( argc > 5 )
    {
    sheetnessThresholdValue = std::atof( argv[5] );
    std::cout << "Setting sheetness threshold value...:\t"
      << sheetnessThresholdValue << std::endl;
    }

  //Compute the eigenvalues
  typedef  SheetnessFilterType::InputImageType
    SymmetricSecondRankTensorImageType;
  using EigenValueArrayType = itk::FixedArray< double, Dimension>;
  using EigenValueImageType = itk::Image< EigenValueArrayType, Dimension>;

  using EigenAnalysisFilterType = itk::SymmetricEigenAnalysisImageFilter<
    SymmetricSecondRankTensorImageType, EigenValueImageType>;

  EigenAnalysisFilterType::Pointer eigenAnalysisFilter =
    EigenAnalysisFilterType::New();
  eigenAnalysisFilter->SetDimension( Dimension );
  eigenAnalysisFilter->OrderEigenValuesBy(
      EigenAnalysisFilterType::FunctorType::OrderByValue );

  eigenAnalysisFilter->SetInput( filterHessian->GetOutput() );
  eigenAnalysisFilter->Update();


  //Generate and write out the primary eigenvector image
  using EigenVectorMatrixType = itk::Matrix< double, 3, 3>;
  using EigenVectorImageType = itk::Image< EigenVectorMatrixType, Dimension>;

  using EigenVectorAnalysisFilterType = itk::tube::SymmetricEigenVectorAnalysisImageFilter<
    SymmetricSecondRankTensorImageType, EigenValueImageType,
    EigenVectorImageType>;

  EigenVectorAnalysisFilterType::Pointer eigenVectorAnalysisFilter =
    EigenVectorAnalysisFilterType::New();
  eigenVectorAnalysisFilter->SetDimension( Dimension );
  eigenVectorAnalysisFilter->OrderEigenValuesBy(
    EigenVectorAnalysisFilterType::FunctorType::OrderByValue );

  eigenVectorAnalysisFilter->SetInput( filterHessian->GetOutput() );
  eigenVectorAnalysisFilter->Update();

  //Generate an image with eigenvector pixel that correspond to the
  //largest eigenvalue
  EigenVectorImageType::ConstPointer eigenVectorImage =
    eigenVectorAnalysisFilter->GetOutput();

  using VectorImageType = itk::VectorImage< double, 3 >;
  VectorImageType::Pointer primaryEigenVectorImage = VectorImageType::New();

  unsigned int vectorLength = 3; // Eigenvector length
  primaryEigenVectorImage->SetVectorLength ( vectorLength );

  VectorImageType::RegionType region;
  region.SetSize( eigenVectorImage->GetLargestPossibleRegion().GetSize() );
  region.SetIndex( eigenVectorImage->GetLargestPossibleRegion().GetIndex() );
  primaryEigenVectorImage->SetRegions( region );
  primaryEigenVectorImage->SetOrigin( eigenVectorImage->GetOrigin() );
  primaryEigenVectorImage->SetSpacing( eigenVectorImage->GetSpacing() );
  primaryEigenVectorImage->Allocate();

  //Fill up the buffer with null vector
  itk::VariableLengthVector< double > nullVector( vectorLength );
  for( unsigned int i=0; i < vectorLength; i++ )
    {
    nullVector[i] = 0.0;
    }
  primaryEigenVectorImage->FillBuffer( nullVector );

  //Setup the iterators
  //
  //Iterator for the eigenvector matrix image
  itk::ImageRegionConstIterator<EigenVectorImageType>
    eigenVectorImageIterator;
  eigenVectorImageIterator =
    itk::ImageRegionConstIterator<EigenVectorImageType>( eigenVectorImage,
    eigenVectorImage->GetRequestedRegion() );
  eigenVectorImageIterator.GoToBegin();

  //Iterator for the input eigenvalue image
  EigenValueImageType::ConstPointer eigenImage =
    eigenAnalysisFilter->GetOutput();
  itk::ImageRegionConstIterator<EigenValueImageType>
    eigenValueImageIterator;
  eigenValueImageIterator = itk::ImageRegionConstIterator<
    EigenValueImageType>( eigenImage, eigenImage->GetRequestedRegion() );
  eigenValueImageIterator.GoToBegin();

  //Iterator for the Sheetness input image
  SheetnessImageType::ConstPointer sheetnessImage =
    filterSheetness->GetOutput();
  itk::ImageRegionConstIterator<SheetnessImageType>
    sheetnessValueImageIterator;
  sheetnessValueImageIterator = itk::ImageRegionConstIterator<
    SheetnessImageType>( sheetnessImage,
    sheetnessImage->GetRequestedRegion() );
  sheetnessValueImageIterator.GoToBegin();

  //Iterator for the output image with the largest eigenvector
  itk::ImageRegionIterator<VectorImageType> primaryEigenVectorImageIterator;
  primaryEigenVectorImageIterator = itk::ImageRegionIterator<
    VectorImageType>( primaryEigenVectorImage, primaryEigenVectorImage->
    GetRequestedRegion() );
  primaryEigenVectorImageIterator.GoToBegin();


  double toleranceEigenValues = 1e-4;

  while( !eigenValueImageIterator.IsAtEnd() )
    {
    // Get the eigenvalue
    EigenValueArrayType eigenValue;
    eigenValue = eigenValueImageIterator.Get();

    // Find the largest eigenvalue
    double largest = vnl_math::abs( eigenValue[0] );
    unsigned int largestEigenValueIndex=0;

    for( unsigned int i=1; i <=2; i++ )
      {
      if(  vnl_math::abs( eigenValue[i] > largest ) )
        {
        largest = vnl_math::abs( eigenValue[i] );
        largestEigenValueIndex = i;
        }
      }

    EigenValueImageType::IndexType pixelIndex;
    pixelIndex = eigenValueImageIterator.GetIndex();

    EigenVectorMatrixType   matrixPixel;
    matrixPixel = eigenVectorImageIterator.Get();

    //If the eigenvalue is above the tolerance eigenvalue and the
    //sheetness is above
    //a threshold, then write out the eigenvector

    SheetnessImageType::PixelType sheetnessValue;
    sheetnessValue = sheetnessValueImageIterator.Get();
    if( ( vnl_math::abs( largest ) >  toleranceEigenValues )  &&
        ( sheetnessValue >  sheetnessThresholdValue ) )
      {
      //Assuming eigenvectors are rows
      itk::VariableLengthVector<double> primaryEigenVector( vectorLength );
      for( unsigned int i=0; i < vectorLength; i++ )
        {
        primaryEigenVector[i] = matrixPixel[largestEigenValueIndex][i];
        }

      primaryEigenVectorImageIterator.Set( primaryEigenVector );
      }

    ++eigenValueImageIterator;
    ++eigenVectorImageIterator;
    ++sheetnessValueImageIterator;
    ++primaryEigenVectorImageIterator;
    }

  using EigenVectorWriterType = itk::ImageFileWriter< VectorImageType >;
  EigenVectorWriterType::Pointer eigenVectorWriter =
    EigenVectorWriterType::New();
  eigenVectorWriter->SetFileName( argv[3] );
  eigenVectorWriter->SetInput( primaryEigenVectorImage );

  std::cout << "Write out the primary eigenvector image" << std::endl;
  eigenVectorWriter->Update();

  return EXIT_SUCCESS;
}
