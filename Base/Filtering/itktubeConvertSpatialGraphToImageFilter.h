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

#ifndef __itktubeConvertSpatialGraphToImageFilter_h
#define __itktubeConvertSpatialGraphToImageFilter_h

#include <itkImage.h>
#include <itkImageRegionIterator.h>
#include <itkImageToImageFilter.h>
#include <itkProcessObject.h>
#include <vector>

namespace itk
{

namespace tube
{

template< class TInputImage, class TOutputImage >
class ConvertSpatialGraphToImageFilter
  : public ImageToImageFilter< TInputImage, TOutputImage >
{
public:

  /** Standard class type alias. */
  using Self = ConvertSpatialGraphToImageFilter;
  using Superclass = ImageToImageFilter< TInputImage, TOutputImage>;
  using Pointer = SmartPointer< Self >;
  using ConstPointer = SmartPointer< const Self >;

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  itkTypeMacro( ConvertSpatialGraphToImageFilter, ImageToImageFilter );

  static constexpr unsigned int ImageDimension = TInputImage::ImageDimension;

  using InputImageType = TInputImage;
  using InputPixelType = typename InputImageType::PixelType;
  using OutputImageType = TOutputImage;
  using InputImagePointer = typename InputImageType::Pointer;
  using OutputImagePointer = typename OutputImageType::Pointer;

  itkGetMacro( AdjacencyMatrixImage, OutputImagePointer );
  itkGetMacro( BranchnessImage, OutputImagePointer );
  itkGetMacro( RadiusImage, OutputImagePointer );
  itkGetMacro( CentralityImage, OutputImagePointer );

  void SetAdjacencyMatrix( vnl_matrix< double > );
  void SetBranchnessVector( vnl_vector< double > );
  void SetRadiusVector( vnl_vector< double > );
  void SetCentralityVector( vnl_vector< double > );

protected:
  ConvertSpatialGraphToImageFilter( void );
  ~ConvertSpatialGraphToImageFilter( void ) {}

  void PrintSelf( std::ostream& os, Indent indent ) const;
  virtual void GenerateData( void );

private:
  ConvertSpatialGraphToImageFilter( const Self& );
  void operator=( const Self& );

  typename OutputImageType::Pointer            m_AdjacencyMatrixImage;
  typename OutputImageType::Pointer            m_BranchnessImage;
  typename OutputImageType::Pointer            m_RadiusImage;
  typename OutputImageType::Pointer            m_CentralityImage;
  typename InputImageType::ConstPointer        m_InputImage;

  vnl_matrix< double > m_AdjacencyMatrix;
  vnl_vector< double > m_BranchnessVector;
  vnl_vector< double > m_RadiusVector;
  vnl_vector< double > m_CentralityVector;

}; // End class CVTImageFilter

} // End namespace tube

} // End namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itktubeConvertSpatialGraphToImageFilter.hxx"
#endif

#endif // End !defined( _itktubeConvertSpatialGraphToImageFilter_h )
