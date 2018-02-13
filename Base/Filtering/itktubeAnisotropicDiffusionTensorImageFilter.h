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

#ifndef __itktubeAnisotropicDiffusionTensorImageFilter_h
#define __itktubeAnisotropicDiffusionTensorImageFilter_h

#include "itktubeAnisotropicDiffusionTensorFunction.h"

#include <itkFiniteDifferenceImageFilter.h>
#include <itkMultiThreader.h>

namespace itk
{

namespace tube
{

/** \class AnisotropicDiffusionTensorImageFilter
 * \brief This is a superclass for filters that iteratively enhance edges in
 *        an image by solving a non-linear diffusion equation.
 *
 * \warning Does not handle image directions.  Re-orient images to axial
 * ( direction cosines = identity matrix ) before using this function.
 *
 * \sa AnisotropicEdgeEnhancementDiffusionImageFilter
 * \sa AnisotropicCoherenceEnhancingDiffusionImageFilter
 * \sa AnisotropicHybridDiffusionImageFilter
 *
 * \ingroup FiniteDifferenceFunctions
 * \ingroup Functions
 */
template< class TInputImage, class TOutputImage >
class AnisotropicDiffusionTensorImageFilter
  : public FiniteDifferenceImageFilter< TInputImage, TOutputImage >
{
public:
  /** Standard class type alias */
  using Self = AnisotropicDiffusionTensorImageFilter;
  using Superclass = FiniteDifferenceImageFilter<TInputImage, TOutputImage>;
  using Pointer = SmartPointer< Self >;
  using ConstPointer = SmartPointer< const Self >;

  // itkNewMacro( Self );  // Not included since pure virtual

  /** Run-time type information ( and related methods ) */
  itkTypeMacro( AnisotropicDiffusionTensorImageFilter,
    FiniteDifferenceImageFiler );

  /** Convenient type alias */
  using InputImageType = typename Superclass::InputImageType;
  using OutputImageType = typename Superclass::OutputImageType;
  using PixelType = typename Superclass::PixelType;

  /** Dimensionality of input and output data is assumed to be the same.
   * It is inherited from the superclass. */
  static constexpr unsigned int ImageDimension = Superclass::ImageDimension;

  /** Type of associated function, with associated type alias */
  using FiniteDifferenceFunctionType = AnisotropicDiffusionTensorFunction< InputImageType >;
  typedef typename FiniteDifferenceFunctionType::DiffusionTensorType
      TensorPixelType;
  typedef typename FiniteDifferenceFunctionType::DiffusionTensorImageType
      DiffusionTensorImageType;

  // Define the type for storing the eigenvalues
  using EigenValueArrayType = FixedArray< double, ImageDimension >;

  // Declare the types of the output images
  using EigenAnalysisOutputImageType = Image< EigenValueArrayType, ImageDimension >;

  /** The value type of a time step.  Inherited from the superclass. */
  using TimeStepType = typename Superclass::TimeStepType;

  /** The container type for the update buffer. */
  using UpdateBufferType = OutputImageType;

  /** Define diffusion image neighborhood type */
  typedef typename
    FiniteDifferenceFunctionType::DiffusionTensorNeighborhoodType
      DiffusionTensorNeighborhoodType;

  /** Set/Get Macro for diffusion tensor image filter parameters */
  itkSetMacro( TimeStep, double );
  itkGetMacro( TimeStep, double );

#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro( OutputTimesDoubleCheck,
    ( Concept::MultiplyOperator< PixelType, double > ) );
  itkConceptMacro( OutputAdditiveOperatorsCheck,
    ( Concept::AdditiveOperators< PixelType > ) );
  itkConceptMacro( InputConvertibleToOutputCheck,
    ( Concept::Convertible< typename TInputImage::PixelType, PixelType > ) );
  /** End concept checking */
#endif

protected:
  AnisotropicDiffusionTensorImageFilter( void );
 ~AnisotropicDiffusionTensorImageFilter( void ) {}
  void PrintSelf( std::ostream& os, Indent indent ) const;

  /* overloaded GenerateData method */
  virtual void GenerateData( void );

  /** A simple method to copy the data from the input to the output. ( Supports
   * "read-only" image adaptors in the case where the input image type converts
   * to a different output image type. )  */
  virtual void CopyInputToOutput( void );

  /** This method applies changes from the m_UpdateBuffer to the output using
   * the ThreadedApplyUpdate() method and a multithreading mechanism.  "dt" is
   * the time step to use for the update of each pixel. */
  virtual void ApplyUpdate( const TimeStepType& dt );

  /** Method to allow subclasses to get direct access to the update
   * buffer */
  virtual UpdateBufferType* GetUpdateBuffer( void )
    { return m_UpdateBuffer; }

  /** This method populates an update buffer with changes for each pixel in the
   * output using the ThreadedCalculateChange() method and a multithreading
   * mechanism. Returns value is a time step to be used for the update. */
  virtual TimeStepType CalculateChange( void );

  /** This method allocates storage in m_UpdateBuffer.  It is called from
   * Superclass::GenerateData(). */
  virtual void AllocateUpdateBuffer( void );

  /** This method allocates storage for the diffusion tensor image */
  void AllocateDiffusionTensorImage( void );

  /** Update diffusion tensor image */
  void virtual UpdateDiffusionTensorImage( void ) = 0;

  /** The type of region used for multithreading */
  using ThreadRegionType = typename UpdateBufferType::RegionType;

  /** The type of region used for multithreading */
  typedef typename DiffusionTensorImageType::RegionType
      ThreadDiffusionTensorImageRegionType;

  typedef typename DiffusionTensorImageType::Pointer
      DiffusionTensorImagePointerType;

  /**  Does the actual work of updating the output from the UpdateContainer
   *   over an output region supplied by the multithreading mechanism.
   *  \sa ApplyUpdate
   *  \sa ApplyUpdateThreaderCallback */
  virtual void ThreadedApplyUpdate( TimeStepType dt,
    const ThreadRegionType &regionToProcess,
    const ThreadDiffusionTensorImageRegionType &diffusionRegionToProcess,
    ThreadIdType threadId );

  /** Does the actual work of calculating change over a region supplied by
   * the multithreading mechanism.
   * \sa CalculateChange
   * \sa CalculateChangeThreaderCallback */
  virtual TimeStepType ThreadedCalculateChange(
    const ThreadRegionType &regionToProcess,
    const ThreadDiffusionTensorImageRegionType &diffusionRegionToProcess,
    ThreadIdType threadId );

  /** Prepare for the iteration process. */
  virtual void InitializeIteration( void );

  DiffusionTensorImagePointerType GetDiffusionTensorImage( void );

private:
  //purposely not implemented
  AnisotropicDiffusionTensorImageFilter( const Self& );
  void operator=( const Self& ); //purposely not implemented

  /** Structure for passing information into static callback methods.  Used in
   * the subclasses' threading mechanisms. */
  struct DenseFDThreadStruct
    {
    AnisotropicDiffusionTensorImageFilter *Filter;
    TimeStepType TimeStep;
    std::vector< TimeStepType > TimeStepList;
    std::vector< bool > ValidTimeStepList;

    }; // End struct DenseFDThreadStruct

  /** This callback method uses ImageSource::SplitRequestedRegion to acquire an
   * output region that it passes to ThreadedApplyUpdate for processing. */
  static ITK_THREAD_RETURN_TYPE ApplyUpdateThreaderCallback( void *arg );

  /** This callback method uses SplitUpdateContainer to acquire a region
   * which it then passes to ThreadedCalculateChange for processing. */
  static ITK_THREAD_RETURN_TYPE CalculateChangeThreaderCallback( void *arg );

  typename DiffusionTensorImageType::Pointer            m_DiffusionTensorImage;

  /** The buffer that holds the updates for an iteration of the algorithm. */
  typename UpdateBufferType::Pointer                    m_UpdateBuffer;

  TimeStepType                                          m_TimeStep;

}; // End class AnisotropicDiffusionTensorImageFilter

} // End namespace tube

} // End namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itktubeAnisotropicDiffusionTensorImageFilter.hxx"
#endif

#endif // End !defined( __itktubeAnisotropicDiffusionTensorImageFilter_h )
