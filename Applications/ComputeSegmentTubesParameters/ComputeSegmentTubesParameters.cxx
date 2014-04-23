/*=========================================================================

Library:   TubeTK

Copyright 2010 Kitware Inc. 28 Corporate Drive,
Clifton Park, NY, 12065, USA.

All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

=========================================================================*/

#include "tubeCLIFilterWatcher.h"
#include "tubeCLIProgressReporter.h"
#include "tubeMessage.h"

#include <itkTimeProbesCollectorBase.h>
#include <itkImageFileWriter.h>
#include <itkImageFileReader.h>
#include <itkImageRegionIteratorWithIndex.h>

#include <itktubeRidgeExtractor.h>

#include <tubeBrentOptimizer1D.h>
#include <tubeOptimizerND.h>

#include <itktubeMetaTubeParams.h>

// Must include CLP before including tubeCLIHelperFunctions
#include "ComputeSegmentTubesParametersCLP.h"

// Must do a forward declaration of DoIt before including
// tubeCLIHelperFunctions
template< class TPixel, unsigned int VDimension >
int DoIt( int argc, char * argv[] );

// Must follow include of "...CLP.h" and forward declaration of int DoIt( ... ).
#include "tubeCLIHelperFunctions.h"

template< class ImageT >
int WriteOutputImage( std::string & fileName, typename ImageT::Pointer
  & image )
{
  typedef itk::ImageFileWriter< ImageT  >  WriterType;

  typename WriterType::Pointer writer = WriterType::New();

  writer->SetInput( image );
  writer->SetFileName( fileName.c_str() );
  writer->SetUseCompression( true );
  try
    {
    writer->Update();
    }
  catch( itk::ExceptionObject & err )
    {
    tube::ErrorMessage( "Writing " + fileName + " : Exception caught: "
      + std::string(err.GetDescription()) );
    return EXIT_FAILURE;
    }
  return EXIT_SUCCESS;
}

template< int VDimension >
int WriteOutputData( std::ofstream & fileStream, itk::ContinuousIndex<
  double, VDimension > & cIndx, double intensity, double ridgeness,
  double roundness, double curvature, double linearity )
{
  for( unsigned int i = 0; i < VDimension; ++i )
    {
    fileStream << cIndx[i] << " ";
    }
  fileStream << intensity << " ";
  fileStream << ridgeness << " ";
  fileStream << roundness << " ";
  fileStream << curvature << " ";
  fileStream << linearity << std::endl;

  return EXIT_SUCCESS;
}

typedef vnl_vector< double >  MetricVectorType;
typedef std::list< MetricVectorType > SampleListType;

class MyOptFunc :
  public tube::UserFunction< vnl_vector< double >, double >
{
private:
  SampleListType * m_Tube;
  SampleListType * m_Bkg;
  double m_Error;

public:
  MyOptFunc( void )
    {
    m_Tube = NULL;
    m_Bkg = NULL;
    m_Error = 0;
    }

  void SetTubeSampleList( SampleListType * tube )
    {
    m_Tube = tube;
    }

  void SetBkgSampleList( SampleListType * bkg )
    {
    m_Bkg = bkg;
    }

  const double & Value( const vnl_vector< double > & x )
    {
    double maxTubeCount = 0.01 * m_Tube->size();
    int tubeCount = 0;
    SampleListType::iterator itr = m_Tube->begin();
    while( itr != m_Tube->end() )
      {
      bool fail = false;
      for( unsigned int f=0; f<5; ++f )
        {
        if( (*itr)[f] < x[f] )
          {
          fail = true;
          break;
          }
        }
      if( fail )
        {
        ++tubeCount;
        }
      ++itr;
      }

    double maxBkgCount = 0.01 * m_Bkg->size();
    int bkgCount = 0;
    SampleListType::iterator itrBkg = m_Bkg->begin();
    while( itrBkg != m_Bkg->end() )
      {
      bool fail = false;
      for( unsigned int f=0; f<5; ++f )
        {
        if( (*itrBkg)[f] > x[f] )
          {
          fail = true;
          break;
          }
        }
      if( fail )
        {
        ++bkgCount;
        }
      ++itrBkg;
      }

    m_Error = 0;
    if( bkgCount > maxBkgCount )
      {
      m_Error += ( bkgCount - maxBkgCount ) / maxBkgCount;
      }
    if( tubeCount > maxTubeCount )
      {
      m_Error += 4 * ( tubeCount - maxTubeCount ) / maxTubeCount;
      }

    std::cout << x << " = " << m_Error << std::endl;

    return m_Error;
    }
};

class MyOptFuncDeriv:
  public tube::UserFunction< vnl_vector< double >, vnl_vector< double > >
{
private:
  vnl_vector< double > m_XStep;
  vnl_vector< double > m_Error;
  MyOptFunc m_Func;

public:
  MyOptFuncDeriv( void )
    {
    m_XStep.set_size( 5 );
    m_XStep = 0.01;
    m_Error.set_size( 5 );
    }

  void SetXStep( const vnl_vector< double > & xStep )
    {
    m_XStep = xStep;
    }

  void SetTubeSampleList( SampleListType * tube )
    {
    m_Func.SetTubeSampleList( tube );
    }

  void SetBkgSampleList( SampleListType * bkg )
    {
    m_Func.SetBkgSampleList( bkg );
    }

  const vnl_vector< double > & Value( const vnl_vector< double > & x )
    {
    vnl_vector< double > xx = x;
    double e0 = m_Func.Value( xx );
    for( unsigned int f=0; f<5; ++f )
      {
      xx[f] += m_XStep[f];
      m_Error[f] = ( m_Func.Value( xx ) - e0 );
      xx[f] = x[f];
      }
    return m_Error;
    }

};

// Your code should be within the DoIt function...
template< class TPixel, unsigned int VDimension >
int DoIt( int argc, char * argv[] )
{
  PARSE_ARGS;

  // The timeCollector is used to perform basic profiling of the components
  //   of your algorithm.
  itk::TimeProbesCollectorBase timeCollector;

  // CLIProgressReporter is used to communicate progress with the Slicer GUI
  tube::CLIProgressReporter    progressReporter(
    "ComputeSegmentTubesParameters", CLPProcessInformation );
  progressReporter.Start();

  typedef TPixel                                    InputPixelType;
  typedef itk::Image< InputPixelType, VDimension >  InputImageType;
  typedef itk::ImageFileReader< InputImageType >    ReaderType;

  typedef int                                       MaskPixelType;
  typedef itk::Image< MaskPixelType, VDimension >   MaskImageType;
  typedef itk::ImageFileReader< MaskImageType >     MaskReaderType;

  typedef float                                     OutputPixelType;
  typedef itk::Image< OutputPixelType, VDimension > OutputImageType;

  typedef itk::tube::RidgeExtractor< InputImageType > RidgeFuncType;

  timeCollector.Start("Load data");

  typename ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( inputImageFileName.c_str() );
  try
    {
    reader->Update();
    }
  catch( itk::ExceptionObject & err )
    {
    tube::ErrorMessage( "Reading volume: Exception caught: "
                        + std::string(err.GetDescription()) );
    timeCollector.Report();
    return EXIT_FAILURE;
    }
  typename InputImageType::Pointer inImage = reader->GetOutput();

  bool useMask = false;
  typename MaskImageType::Pointer maskImage = MaskImageType::New();
  if( !maskImageFileName.empty() )
    {
    useMask = true;

    typename MaskReaderType::Pointer maskReader = MaskReaderType::New();
    maskReader->SetFileName( maskImageFileName.c_str() );
    try
      {
      maskReader->Update();
      }
    catch( itk::ExceptionObject & err )
      {
      tube::ErrorMessage( "Reading mask: Exception caught: "
                          + std::string(err.GetDescription()) );
      timeCollector.Report();
      return EXIT_FAILURE;
      }
    maskImage = maskReader->GetOutput();
    }

  timeCollector.Stop("Load data");
  double progress = 0.1;
  progressReporter.Report( progress );

  if( scale < inImage->GetSpacing()[0] * 0.3 )
    {
    scale = inImage->GetSpacing()[0] * 0.3;
    tubeWarningMacro( << "Reseting scale to " << scale );
    }

  timeCollector.Start("Compute ridgeness images");

  typename OutputImageType::RegionType region =
    inImage->GetLargestPossibleRegion();
  typename OutputImageType::SpacingType spacing =
    inImage->GetSpacing();
  if( supersample != 1 )
    {
    typename OutputImageType::RegionType::SizeType size = region.GetSize();
    for( unsigned int i = 0; i < VDimension; ++i )
      {
      size[i] *= supersample;
      spacing[i] /= supersample;
      }
    region.SetSize( size );
    }

  bool useOutputImages = false;
  typename OutputImageType::Pointer outImageRidgeness =
    OutputImageType::New();
  typename OutputImageType::Pointer outImageRoundness =
    OutputImageType::New();
  typename OutputImageType::Pointer outImageCurvature =
    OutputImageType::New();
  typename OutputImageType::Pointer outImageLinearity =
    OutputImageType::New();
  if( !outputImagesBaseFileName.empty() )
    {
    useOutputImages = true;

    outImageRidgeness->CopyInformation( inImage );
    outImageRidgeness->SetSpacing( spacing );
    outImageRidgeness->SetRegions( region );
    outImageRidgeness->Allocate();

    outImageRoundness->CopyInformation( inImage );
    outImageRoundness->SetSpacing( spacing );
    outImageRoundness->SetRegions( region );
    outImageRoundness->Allocate();

    outImageCurvature->CopyInformation( inImage );
    outImageCurvature->SetSpacing( spacing );
    outImageCurvature->SetRegions( region );
    outImageCurvature->Allocate();

    outImageLinearity->CopyInformation( inImage );
    outImageLinearity->SetSpacing( spacing );
    outImageLinearity->SetRegions( region );
    outImageLinearity->Allocate();
    }

  bool useOutputDataStream = false;
  std::ofstream outputDataStreamInit;
  std::ofstream outputDataStreamTube;
  std::ofstream outputDataStreamBkg;
  if( !outputDataFileBaseName.empty() )
    {
    useOutputDataStream = true;
    std::string fileName = outputDataFileBaseName + ".init.txt";
    outputDataStreamInit.open( fileName.c_str(), std::ios::binary |
      std::ios::out );
    outputDataStreamInit.precision( 6 );
    fileName = outputDataFileBaseName + ".tube.txt";
    outputDataStreamTube.open( fileName.c_str(), std::ios::binary |
      std::ios::out );
    outputDataStreamTube.precision( 6 );
    fileName = outputDataFileBaseName + ".bkg.txt";
    outputDataStreamBkg.open( fileName.c_str(), std::ios::binary |
      std::ios::out );
    outputDataStreamBkg.precision( 6 );
    }

  SampleListType seed;
  SampleListType tube;
  MetricVectorType tubeMean( 5, 0 );
  SampleListType bkg;

  itk::ImageRegionIteratorWithIndex< MaskImageType > itM(
    maskImage, region );
  itk::ImageRegionIteratorWithIndex< OutputImageType > itR(
    outImageRidgeness, region );
  itk::ImageRegionIteratorWithIndex< OutputImageType > itO(
    outImageRoundness, region );
  itk::ImageRegionIteratorWithIndex< OutputImageType > itC(
    outImageCurvature, region );
  itk::ImageRegionIteratorWithIndex< OutputImageType > itL(
    outImageLinearity, region );

  typename RidgeFuncType::Pointer ridgeFunc = RidgeFuncType::New();
  ridgeFunc->SetInputImage( inImage );
  ridgeFunc->SetScale( scale );

  std::vector< int > failedCount( 5, 0 );
  while( !itR.IsAtEnd() )
    {
    if( !useMask || itM.Get() == maskBackgroundId
      || itM.Get() == maskTubeId )
      {
      double supersampleFactor = 1.0 / supersample;
      double intensity = 0;
      double ridgeness = 0;
      double roundness = 0;
      double curvature = 0;
      double linearity = 0;
      typename RidgeFuncType::ContinuousIndexType cIndx;
      if( supersample != 1 )
        {
        for( unsigned int i = 0; i < VDimension; ++i )
          {
          cIndx[i] = itR.GetIndex()[i] * supersampleFactor;
          }
        }
      else
        {
        for( unsigned int i = 0; i < VDimension; ++i )
          {
          cIndx[i] = itR.GetIndex()[i];
          }
        }
      if( itM.Get() == maskTubeId )
        {
        ridgeness = ridgeFunc->Ridgeness( cIndx, intensity, roundness,
          curvature, linearity );

        MetricVectorType instance(5, 0);
        instance[0] = intensity;
        instance[1] = ridgeness;
        instance[2] = roundness;
        instance[3] = curvature;
        instance[4] = linearity;
        seed.push_back( instance );

        if( useOutputDataStream )
          {
          WriteOutputData< VDimension >( outputDataStreamInit, cIndx,
            intensity, ridgeness, roundness, curvature, linearity );
          }

        ridgeFunc->LocalRidge( cIndx );
        }

      ridgeness = ridgeFunc->Ridgeness( cIndx, intensity, roundness,
        curvature, linearity );
      if( useOutputImages )
        {
        itR.Set( (OutputPixelType) ridgeness );
        itO.Set( (OutputPixelType) roundness );
        itC.Set( (OutputPixelType) curvature );
        itL.Set( (OutputPixelType) linearity );
        }

      if( itM.Get() == maskTubeId )
        {
        MetricVectorType instance(5, 0);
        instance[0] = intensity;
        instance[1] = ridgeness;
        instance[2] = roundness;
        instance[3] = curvature;
        instance[4] = linearity;
        tube.push_back( instance );

        int count = tube.size();
        for( unsigned int f=0; f<5; ++f )
          {
          tubeMean[f] += ( instance[f] - tubeMean[f] ) / ( count + 1 );
          }

        if( useOutputDataStream )
          {
          WriteOutputData< VDimension >( outputDataStreamTube, cIndx,
            intensity, ridgeness, roundness, curvature, linearity );
          }
        }
      else if( itM.Get() == maskBackgroundId )
        {
        MetricVectorType instance(5, 0);
        instance[0] = intensity;
        instance[1] = ridgeness;
        instance[2] = roundness;
        instance[3] = curvature;
        instance[4] = linearity;
        bkg.push_back( instance );

        if( useOutputDataStream )
          {
          WriteOutputData< VDimension >( outputDataStreamBkg, cIndx,
            intensity, ridgeness, roundness, curvature, linearity );
          }
        }
      }
    if( useMask )
      {
      ++itM;
      }
    if( useOutputImages )
      {
      ++itR;
      ++itO;
      ++itC;
      ++itL;
      }
    }

  MyOptFunc myFunc;
  myFunc.SetTubeSampleList( & tube );
  myFunc.SetBkgSampleList( & bkg );
  MyOptFuncDeriv myFuncD;
  myFuncD.SetTubeSampleList( & tube );
  myFuncD.SetBkgSampleList( & bkg );

  tube::BrentOptimizer1D opt1D;
  tube::OptimizerND opt( 2, &myFunc, &myFuncD, &opt1D );

  MetricVectorType xMin( 5, 0 );
  opt.SetXMin( xMin );

  MetricVectorType xMax( 5, 1 );
  opt.SetXMax( xMax );

  opt.SetTolerance( 0.0001 );
  opt.SetMaxIterations( 30000 );
  opt.SetSearchForMin( true );

  double xVal = 500;
  MetricVectorType x( 5 );
  for( unsigned int f=0; f<5; ++f )
    {
    x[f] = tubeMean[f];
    }

  MetricVectorType xStep( 5, 0.1 );
  opt.SetXStep( xStep );
  myFuncD.SetXStep( xStep );
  if( !opt.Extreme( x, &xVal ) )
    {
    std::cout << "Optimization failed!" << std::endl;
    }
  std::cout << "x = " << x << std::endl;
  std::cout << "   xVal = " << xVal << std::endl;

  MetricVectorType xStep2( 5, 0.01 );
  opt.SetXStep( xStep2 );
  myFuncD.SetXStep( xStep2 );
  if( !opt.Extreme( x, &xVal ) )
    {
    std::cout << "Optimization failed!" << std::endl;
    }
  std::cout << "x = " << x << std::endl;
  std::cout << "   xVal = " << xVal << std::endl;

  MetricVectorType xStep3( 5, 0.001 );
  opt.SetXStep( xStep3 );
  myFuncD.SetXStep( xStep3 );
  if( !opt.Extreme( x, &xVal ) )
    {
    std::cout << "Optimization failed!" << std::endl;
    }
  std::cout << "x = " << x << std::endl;
  std::cout << "   xVal = " << xVal << std::endl;

  if( useOutputDataStream )
    {
    outputDataStreamBkg.close();
    outputDataStreamTube.close();
    }
  timeCollector.Stop("Compute ridgeness images");

  int result = EXIT_SUCCESS;
  if( useOutputImages )
    {
    timeCollector.Start("Save data");

    std::string outName = outputImagesBaseFileName + ".ridge.mha";
    result = WriteOutputImage< OutputImageType >( outName,
      outImageRidgeness );

    outName = outputImagesBaseFileName + ".round.mha";
    result += WriteOutputImage< OutputImageType >( outName,
      outImageRoundness );

    outName = outputImagesBaseFileName + ".curve.mha";
    result += WriteOutputImage< OutputImageType >( outName,
      outImageCurvature );

    outName = outputImagesBaseFileName + ".line.mha";
    result += WriteOutputImage< OutputImageType >( outName,
      outImageLinearity );

    timeCollector.Stop("Save data");
    }

  //
  //
  itk::tube::MetaTubeParams params;

  itk::tube::MetaTubeParams::VectorType seedScales( 1, 1.0 );
  double seedIntensityMin = 0;
  double seedIntensityMax = 0;
  double seedIntensityPercentile = 0;
  params.SetSeedParams( seedScales, seedIntensityMin, seedIntensityMax,
    seedIntensityPercentile );

  double tubeIntensityMin = 0;
  double tubeIntensityMax = 0;
  bool   tubeBright = true;
  itk::tube::MetaTubeParams::VectorType tubeColor(4, 0.0);
  tubeColor[0] = 1.0;
  tubeColor[3] = 1.0;

  params.SetTubeParams( tubeIntensityMin, tubeIntensityMax,
    tubeBright, tubeColor );

  double ridgeScale = scale;
  double ridgeScaleExtent = 1.5;
  bool   ridgeDynamicScale = true;
  double ridgeStepX = 0.2;
  double ridgeThreshTangentChange = 0.8;
  double ridgeThreshXChange = 0.5;
  double ridgeThreshRidgeness = x[1];
  double ridgeThreshRidgenessStart = x[1];
  double ridgeThreshRoundness = x[2];
  double ridgeThreshRoundnessStart = x[2];
  double ridgeCurvatureMax = x[3];
  double ridgeThreshCurvature = x[3];
  double ridgeThreshCurvatureStart = x[3];
  double ridgeThreshLinearity = x[4];
  double ridgeThreshLinearityStart = x[4];
  int    ridgeRecoveryMax = 3;
  params.SetTubeRidgeParams( ridgeScale, ridgeScaleExtent,
    ridgeDynamicScale, ridgeStepX,
    ridgeThreshTangentChange,
    ridgeThreshXChange,
    ridgeThreshRidgeness, ridgeThreshRidgenessStart,
    ridgeThreshRoundness, ridgeThreshRoundnessStart,
    ridgeCurvatureMax,
    ridgeThreshCurvature, ridgeThreshCurvatureStart,
    ridgeThreshLinearity, ridgeThreshLinearityStart,
    ridgeRecoveryMax );

  double radiusScale = scale;
  double radiusMin = 0.3;
  double radiusMax = 10;
  double radiusThreshMedialness = 0.04;
  double radiusThreshMedialnessStart = 0.01;
  params.SetTubeRadiusParams( radiusScale,
    radiusMin, radiusMax,
    radiusThreshMedialness, radiusThreshMedialnessStart );

  params.Write( outputParametersFile.c_str() );

  progress = 1.0;
  progressReporter.Report( progress );
  progressReporter.End();

  timeCollector.Report();
  return result;
}

// Main
int main( int argc, char * argv[] )
{
  PARSE_ARGS;

  // You may need to update this line if, in the project's .xml CLI file,
  //   you change the variable name for the inputImageFileName.
  return tube::ParseArgsAndCallDoIt( inputImageFileName, argc, argv );
}