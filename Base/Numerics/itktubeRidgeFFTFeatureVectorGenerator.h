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

#ifndef __itktubeRidgeFFTFeatureVectorGenerator_h
#define __itktubeRidgeFFTFeatureVectorGenerator_h

#include "itktubeFeatureVectorGenerator.h"

#include <itkImage.h>

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include <vector>

namespace itk
{

namespace tube
{

template< class TImage >
class RidgeFFTFeatureVectorGenerator
  : public FeatureVectorGenerator< Image< typename TImage::PixelType,
                                          TImage::ImageDimension > >
{
public:

  using Self = RidgeFFTFeatureVectorGenerator;
  using Superclass = FeatureVectorGenerator< TImage >;
  using Pointer = SmartPointer< Self >;
  using ConstPointer = SmartPointer< const Self >;

  itkTypeMacro( RidgeFFTFeatureVectorGenerator, FeatureVectorGenerator );

  itkNewMacro( Self );

  //
  static constexpr unsigned int ImageDimension = TImage::ImageDimension;

  using FeatureValueType = typename Superclass::FeatureValueType;

  using FeatureVectorType = typename Superclass::FeatureVectorType;

  using FeatureImageType = typename Superclass::FeatureImageType;

  using ValueListType = typename Superclass::ValueListType;

  using ImageType = typename Superclass::ImageType;

  using IndexType = typename Superclass::IndexType;

  using RidgeScalesType = std::vector< double >;

  using FeatureImageListType = std::vector< typename FeatureImageType::Pointer >;

  //
  virtual unsigned int GetNumberOfFeatures( void ) const;

  void SetScales( const RidgeScalesType & scales );
  const RidgeScalesType & GetScales( void ) const;

  virtual FeatureVectorType GetFeatureVector(
    const IndexType & indx ) const;

  virtual FeatureValueType GetFeatureVectorValue( const IndexType & indx,
    unsigned int fNum ) const;

  virtual typename FeatureImageType::Pointer GetFeatureImage(
    unsigned int fNum ) const;

  virtual void Update( void );

  itkSetMacro( UseIntensityOnly, bool );
  itkGetMacro( UseIntensityOnly, bool );

protected:

  RidgeFFTFeatureVectorGenerator( void );
  virtual ~RidgeFFTFeatureVectorGenerator( void );

  virtual void UpdateWhitenStatistics( void );

  void PrintSelf( std::ostream & os, Indent indent ) const;

private:

  // Purposely not implemented
  RidgeFFTFeatureVectorGenerator( const Self & );
  void operator = ( const Self & );      // Purposely not implemented

  RidgeScalesType                    m_Scales;

  FeatureImageListType               m_FeatureImageList;

  bool                               m_UseIntensityOnly;

}; // End class RidgeFFTFeatureVectorGenerator

} // End namespace tube

} // End namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itktubeRidgeFFTFeatureVectorGenerator.hxx"
#endif

#endif // End !defined( __itktubeRidgeFFTFeatureVectorGenerator_h )
