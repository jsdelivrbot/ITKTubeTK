/*=========================================================================
 *
 *  Copyright Insight Software Consortium
 *
 *  Licensed under the Apache License, Version 2.0 ( the "License" );
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
*=========================================================================*/
#ifndef __itktubeGaussianDerivativeFilter_h
#define __itktubeGaussianDerivativeFilter_h

#include "itkImage.h"
#include "itkImageToImageFilter.h"
#include "itktubeGaussianDerivativeImageSource.h"

namespace itk
{
namespace tube
{

template< typename TInputImage, typename TOutputImage =
Image< float, TInputImage::ImageDimension >  >
class GaussianDerivativeFilter :
  public ImageToImageFilter< TInputImage, TOutputImage >
{
public:

  /** Standard class type alias. */
  using Self = GaussianDerivativeFilter;
  using Superclass = ImageToImageFilter< TInputImage, TOutputImage >;
  using Pointer = SmartPointer< Self >;
  using ConstPointer = SmartPointer< const Self >;

  /** Run-time type information ( and related methods ). */
  itkTypeMacro( GaussianDerivativeFilter, ImageToImageFilter );

  static constexpr unsigned int ImageDimension = TInputImage::ImageDimension;

  using InputImageType = TInputImage;
  using OutputImageType = TOutputImage;
  using RealImageType = Image< double, ImageDimension >;
  using RealImagePointerType = typename RealImageType::Pointer;
  using GaussianDerivativeImageSourceType = GaussianDerivativeImageSource< RealImageType >;
  typedef typename GaussianDerivativeImageSourceType::OrdersType
                                              OrdersType;
  typedef typename GaussianDerivativeImageSourceType::SigmasType
                                              SigmasType;

  itkSetMacro( Orders, OrdersType& );
  itkGetConstReferenceMacro( Orders, OrdersType );

  itkSetMacro( Sigmas, SigmasType& );
  itkGetConstReferenceMacro( Sigmas, SigmasType );

  virtual void GenerateNJet( typename OutputImageType::Pointer & D,
    std::vector< typename TOutputImage::Pointer > & Dx,
    std::vector< typename TOutputImage::Pointer > & Dxx ) = 0;

protected:

  GaussianDerivativeFilter ( void );
  virtual ~GaussianDerivativeFilter ( void ) {}

  void PrintSelf ( std::ostream & os, Indent indent ) const;

  OrdersType   m_Orders;
  SigmasType   m_Sigmas;

private:
  // Purposely not implemented
  GaussianDerivativeFilter ( const Self & );
  void operator = ( const Self & );

}; // End class GaussianDerivativeFilter

} // End namespace tube
} // End namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itktubeGaussianDerivativeFilter.hxx"
#endif

#endif
