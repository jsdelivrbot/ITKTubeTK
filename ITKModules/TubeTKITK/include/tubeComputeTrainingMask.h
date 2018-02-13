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
#ifndef __tubeComputeTrainingMask_h
#define __tubeComputeTrainingMask_h

// ITK Includes
#include "itkProcessObject.h"

// TubeTK Includes
#include "tubeWrappingMacros.h"

#include "itktubeComputeTrainingMaskFilter.h"

namespace tube
{
/** \class ComputeTrainingMask
 *
 *  \ingroup TubeTKITK
 */

template< typename TImage >
class ComputeTrainingMask : public itk::ProcessObject
{
public:
  /** Standard class type alias. */
  using Self = ComputeTrainingMask;
  using Superclass = itk::ProcessObject;
  using Pointer = itk::SmartPointer< Self >;
  using ConstPointer = itk::SmartPointer< const Self >;

  /** Method for creation through the object factory. */
  itkNewMacro( Self );

  /** Run-time type information ( and related methods ). */
  itkTypeMacro( ComputeTrainingMask, ProcessObject );


  /** Typedef to images */
  using ImageType = TImage;

  static constexpr unsigned int ImageDimension = ImageType::ImageDimension;

  using FilterType = typename itk::tube::ComputeTrainingMaskFilter< ImageType >;
  using ImageTypeShort = typename FilterType::ImageTypeShort;

  tubeWrapSetMacro( Gap, double, ComputeTrainingMaskFilter );
  tubeWrapGetMacro( Gap, double, ComputeTrainingMaskFilter );
  tubeWrapSetMacro( NotVesselWidth, double, ComputeTrainingMaskFilter );
  tubeWrapGetMacro( NotVesselWidth, double, ComputeTrainingMaskFilter );
  tubeWrapGetConstObjectMacro( NotVesselMask, ImageTypeShort,
    ComputeTrainingMaskFilter );

  tubeWrapSetObjectMacro( Input, ImageType,
    ComputeTrainingMaskFilter );

  tubeWrapCallMacro( Update, ComputeTrainingMaskFilter );

  tubeWrapGetObjectMacro( Output, ImageTypeShort,
    ComputeTrainingMaskFilter );

protected:
  ComputeTrainingMask( void );
  ~ComputeTrainingMask() {}

  void PrintSelf( std::ostream & os, itk::Indent indent ) const;

private:
  /** itkComputeTrainingMask parameters **/
  ComputeTrainingMask( const Self & );

  void operator=( const Self & );

  // To remove warning "was hidden [-Woverloaded-virtual]"
  void SetInput( const DataObjectIdentifierType &, itk::DataObject * ) {};

  typename FilterType::Pointer m_ComputeTrainingMaskFilter;

};

} // End namespace tube

#ifndef ITK_MANUAL_INSTANTIATION
#include "tubeComputeTrainingMask.hxx"
#endif

#endif // End !defined( __tubeComputeTrainingMask_h )
