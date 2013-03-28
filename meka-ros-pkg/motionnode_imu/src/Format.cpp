/**
  Implementation of the Format class. See the header file for
  more details.

  @file    tools/sdk/cpp/src/Format.cpp
  @author  Luke Tokheim, luke@motionnode.com
  @version 1.2

  (C) Copyright GLI Interactive LLC 2009. All rights reserved.

  The coded instructions, statements, computer programs, and/or related
  material (collectively the "Data") in these files contain unpublished
  information proprietary to GLI Interactive LLC, which is protected by
  US federal copyright law and by international treaties.

  The Data may not be disclosed or distributed to third parties, in whole
  or in part, without the prior written consent of GLI Interactive LLC.

  The Data is provided "as is" without express or implied warranty, and
  with no claim as to its suitability for any purpose.
*/

#include <Format.hpp>

#include <limits>
#include <stdexcept>


namespace MotionNode { namespace SDK {

template<typename Quaternion>
Format::data_type quaternion_to_R3_rotation(const Quaternion & q);

#if !defined(_WIN32)
const std::size_t Format::PreviewElement::Length;
const std::size_t Format::SensorElement::Length;
const std::size_t Format::RawElement::Length;
const std::size_t Format::ConfigurableElement::Length;
#endif // _WIN32

std::string Format::ConfigurableElement::Name("Configurable");
std::string Format::PreviewElement::Name("Preview");
std::string Format::SensorElement::Name("Sensor");
std::string Format::RawElement::Name("Raw");


Format::ConfigurableElement::ConfigurableElement(const data_type & data)
  : Element<data_type::value_type>(data, Length)
{
}
const Format::ConfigurableElement::value_type & Format::ConfigurableElement::operator[](const std::size_t & index) const
{
  return access()[index];
}
std::size_t Format::ConfigurableElement::size() const
{
  return access().size();
}
Format::ConfigurableElement::data_type Format::ConfigurableElement::getRange(
  const std::size_t & base,
  const std::size_t & length) const
{
  return getData(base, length);
}


Format::PreviewElement::PreviewElement(const data_type & data)
  : Element<data_type::value_type>(data, Length)
{
}
Format::PreviewElement::data_type Format::PreviewElement::getEuler() const
{
  return getData(8, 3);
}
Format::PreviewElement::data_type Format::PreviewElement::getMatrix(bool local) const
{
  return quaternion_to_R3_rotation(getQuaternion(local));
}
Format::PreviewElement::data_type Format::PreviewElement::getQuaternion(bool local) const
{
  if (local) {
    return getData(4, 4);
  } else {
    return getData(0, 4);
  }
}
Format::PreviewElement::data_type Format::PreviewElement::getAccelerate() const
{
  return getData(11, 3);
}


Format::SensorElement::SensorElement(const data_type & data)
  : Element<data_type::value_type>(data, Length)
{
}
Format::SensorElement::data_type Format::SensorElement::getAccelerometer() const
{
  return getData(0, 3);
}
Format::SensorElement::data_type Format::SensorElement::getGyroscope() const
{
  return getData(6, 3);
}
Format::SensorElement::data_type Format::SensorElement::getMagnetometer() const
{
  return getData(3, 3);
}


Format::RawElement::RawElement(const data_type & data)
  : Element<data_type::value_type>(data, Length)
{
}
Format::RawElement::data_type Format::RawElement::getAccelerometer() const
{
  return getData(0, 3);
}
Format::RawElement::data_type Format::RawElement::getGyroscope() const
{
  return getData(6, 3);
}
Format::RawElement::data_type Format::RawElement::getMagnetometer() const
{
  return getData(3, 3);
}


/**
  Ported from the Boost.Quaternion library at:
  http://www.boost.org/libs/math/quaternion/HSO3.hpp

  @param q defines a quaternion in the format [w x y z] where
  <tt>q = w + x*i + y*j + z*k = (w, x, y, z)</tt>
  @return an array of 16 elements that defines a 4-by-4 rotation
  matrix computed from the input quaternion or identity matrix if
  the input quaternion has zero length
*/
template<typename Quaternion>
Format::PreviewElement::data_type quaternion_to_R3_rotation(const Quaternion & q)
{
  typedef Format::data_type::value_type real_type;

  // Initialize the result matrix to the identity.
  Format::PreviewElement::data_type result(16);
  {
    result[0] = result[5] = result[10] = result[15] = 1;
  }

  if (4 != q.size()) {
    return result;
  }


	const real_type a = q[0];
	const real_type b = q[1];
	const real_type c = q[2];
	const real_type d = q[3];

	const real_type aa = a*a;
	const real_type ab = a*b;
	const real_type ac = a*c;
	const real_type ad = a*d;
	const real_type bb = b*b;
	const real_type bc = b*c;
	const real_type bd = b*d;
	const real_type cc = c*c;
	const real_type cd = c*d;
	const real_type dd = d*d;

	const real_type norme_carre = aa+bb+cc+dd;

  if (norme_carre > 1e-6) {
    result[0] = (aa+bb-cc-dd)/norme_carre;
    result[1] = 2*(-ad+bc)/norme_carre;
    result[2] = 2*(ac+bd)/norme_carre;
    result[4] = 2*(ad+bc)/norme_carre;
    result[5] = (aa-bb+cc-dd)/norme_carre;
    result[6] = 2*(-ab+cd)/norme_carre;
    result[7] = 0;
    result[8] = 2*(-ac+bd)/norme_carre;
    result[9] = 2*(ab+cd)/norme_carre;
    result[10] = (aa-bb-cc+dd)/norme_carre;
  }

	return result;
}

}} // namespace MotionNode::SDK
