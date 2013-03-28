/*
  @file    tools/sdk/cpp/Format.hpp
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
#ifndef __MOTION_NODE_SDK_FORMAT_HPP_
#define __MOTION_NODE_SDK_FORMAT_HPP_
#include <cstring>
#include <detail/endian_to_native.hpp>

#include <algorithm>
#include <iterator>
#include <map>
#include <stdexcept>
#include <vector>


namespace MotionNode { namespace SDK {

/**
   The Format class methods read a single binary message from a MotionNode
   service and returns an object representation of that message. This is
   layered (by default) on top of the @ref Client class which handles the
   socket level binary message protocol.

   Example usage, extends the @ref Client class example:
   @code
   try {
     using MotionNode::SDK::Client;
     using MotionNode::SDK::Format;

     // Connect to the Preview data service.
     Client client("", 32079);

     Client::data_type data;
     while (client.readData(data)) {
       // Create an object representation of the current binary message.
       Format::preview_service_type preview =
         Format::Preview(data.begin(), data.end());
       
       // Iterate through the list of [id] => PreviewElement objects.
       for (Format::preview_service_type::iterator itr=preview.begin(); itr!=preview.end(); ++itr) {
         // Use the PreviewElement interface to access format specific data.
         const Format::PreviewElement & element = itr->second;
       }
     }

   } catch (std::runtime_error & e) {
     // The Client and Format class with throw std::runtime_error for
     // any unrecoverable conditions.
   }
   @endcode
*/
class Format {
public:
  /** Defines the type of the integer map key for all Format types. */
  typedef std::size_t id_type;
  /** Defines the type of the data array for all Format types. */
  typedef std::vector<float> data_type;


  /**
    MotionNode services send a list of data elements. The @ref Format
    functions create a map from integer id to array packed data for each
    service specific format.

    This is an abstract base class to implement a single format specific
    data element. The idea is that a child class implements a format specific
    interface (API) to access individual components of an array of packed
    data. The template parameter defines the type of the packed data
    elements.

    For example, the @ref PreviewElement class extends this class
    and provides a @ref PreviewElement#getEuler method to access
    an array of <tt>{x, y, z}</tt> Euler angles.
  */
  template <typename T>
  class Element {
  public:
    typedef T value_type;
    typedef std::vector<T> data_type;
  protected:

    /**
      Constructor is protected. Only allow child classes to call this.

      @param data array of packed data values for this Format::Element
      @param length valid length of the <tt>data</tt> array
      @pre <tt>data.size() == length</tt>
      @throws std::runtime_error if the <tt>data</tt> array does not
      have <tt>element_length</tt> values
    */
    Element(const data_type & data, const typename data_type::size_type & length)
      : m_data(data)
    {
      if ((0 != length) && (length != m_data.size())) {
        throw std::runtime_error("invalid input data for format element");
      }
    }

    /**
      Utility function to copy portions of the packed data array into its
      component elements.

      @param base starting index to copy data from the internal data array
      @param length number of data values in this component element
      @pre <tt>base < m_data.size()</tt>, <tt>base + element_length < m_data.size()</tt>
      @return an array of <tt>element_length</tt> values, assigned to
      <tt>{m_data[i] ... m_data[i+element_length]}</tt> if there are valid
      values available or zeros otherwise
    */
    data_type getData(
      const typename data_type::size_type & base,
      const typename data_type::size_type & length) const
    {
      data_type result(length);
      if (base + length <= m_data.size()) {
        std::copy(
          m_data.begin() + base, m_data.begin() + base + length,
          result.begin());
      }
      return result;
    }
  private:
    /**
      Array of packed binary data for this element. If <tt>data.empty() ==
      false</tt> then it contains a sample for each of the <tt>N</tt> channels.
    
      Define this as a private member. Only allow access through the
      getData method, and only allow it to child classes.
    */
    data_type m_data;

    /**
      Provide direct access to the internal data buffer from client programs.

      @code
      class ElementAccess {
      public:
        template <typename T>
        static const typename Format::Element<T>::data_type &
        get(const Format::Element<T> & element)
        {
          return element.m_data;
        }
      };
      @endcode
    */
    friend class ElementAccess;
  public:
    const data_type & access() const
    {
      return m_data;
    }
  }; // class Element

  /**
    The Configurable data services provides access to all data streams in
    a single message. The client selects channels and ordering at the
    start of the connection. The Configurable service sends a map of
    <tt>N</tt> data elements. Each data element is an array of <tt>M</tt>
    single precision floating point numbers.
  */
  class ConfigurableElement : public Element<float> {
  public:
    typedef Format::Element<float>::value_type value_type;
    typedef Format::Element<float>::data_type data_type;

    /** Variable length channels. */
    const static std::size_t Length = 0;
    static std::string Name;

    ConfigurableElement(const data_type & data);

    /**
      Get a single channel entry at specified index.
    */
    const value_type & operator[](const std::size_t & index) const;

    /**
      Convenience method. Size accessor.
    */
    std::size_t size() const;

    /**
      
    */
    data_type getRange(
      const std::size_t & base,
      const std::size_t & length) const;
  }; // class ConfigurableElement

  /**
    The Preview service provides access to the current orientation output as
    a quaternion, a set of Euler angles, or a 4-by-4 rotation matrix. The Preview
    service sends a map of <tt>N</tt> Preview data elements. Use this class to wrap
    a single Preview data element such that we can access individual components
    through a simple API.

    Preview element format:
    id => [global quaternion, local quaternion, local euler, global acceleration]
    id => {Gqw, Gqx, Gqy, Gqz, Lqw, Lqx, Lqy, Lqz, rx, ry, rz, ax, ay, az}
  */
  class PreviewElement : public Element<float> {
  public:
    typedef Format::Element<float>::data_type data_type;

    /** Two quaternion channels, two 3-axis channels. */
    const static std::size_t Length = 2*4 + 2*3;
    static std::string Name;

    /**
      Initialize this container identifier with a packed data
      array in the Preview format.

      @param data is a packed array of global quaternion, local
      quaternion, local Euler angle, and local translation channel
      data
      @pre <tt>data.size() == Length</tt>
    */
    PreviewElement(const data_type & data);

    /**
      Get a set of x, y, and z Euler angles that define the
      current orientation. Specified in radians assuming <tt>x-y-z</tt>
      rotation order. Not necessarily continuous over time, each
      angle lies on the domain <tt>[-pi, pi]</tt>.

      Euler angles are computed on the server side based on the
      current local quaternion orientation.

      @return a three element array <tt>{x, y, z}</tt> of Euler angles
      in radians or zeros if there is no available data
    */
    data_type getEuler() const;
	  
    /**
      Get a 4-by-4 rotation matrix from the current global or local quaternion
      orientation. Specified as a 16 element array in row-major order.

      @param local set local to true get the local orientation, set local
      to false to get the global orientation
    */
    data_type getMatrix(bool local) const;
	  
    /**
      Get the global or local unit quaternion that defines the current
      orientation.

      @param local set local to true get the local orientation, set local
      to false to get the global orientation
      @return a four element array <tt>{w, x, y, z}</tt> that defines a
      unit length quaternion <tt>q = w + x*i + y*j + z*k</tt> or zeros
      if there is no available data
    */
    data_type getQuaternion(bool local) const;
	  
    /**
      Get x, y, and z of the current estimate of linear acceleration.
      Specified in g.

      @return a three element array <tt>{x, y, z}</tt> of linear acceleration
      channels specified in g or zeros if there is no available data
    */
    data_type getAccelerate() const;
  }; // class PreviewElement


  /**
    The Sensor service provides access to the current un-filtered sensor signals
    in real units. The Sensor service sends a map of <tt>N</tt> data elements.
    Use this class to wrap a single Sensor data element such that we can access
    individual components through a simple API.

    Sensor element format:
    id => [accelerometer, magnetometer, gyroscope]
    id => {ax, ay, az, mx, my, mz, gx, gy, gz}
  */
  class SensorElement : public Element<float> {
  public:
    typedef Format::Element<float>::data_type data_type;

    /** Three 3-axis channels. */
    const static std::size_t Length = 3*3;
    static std::string Name;

    /**
      Initialize this container identifier with a packed data
      array in the Sensor format.

      @param data is a packed array of accelerometer, magnetometer,
      and gyroscope un-filtered signal data.
      @pre <tt>data.size() == Length</tt>
    */
    SensorElement(const data_type & data);

    /**
      Get a set of x, y, and z values of the current un-filtered
      accelerometer signal. Specified in <em>g</em> where 1 <em>g</em> 
      = <tt>9.80665 meter/second^2</tt>.

      Domain varies with configuration. Maximum is <tt>[-6, 6]</tt>
      <em>g</em>.

      @return a three element array <tt>{x, y, z}</tt> of acceleration
      in <em>g</em>s or zeros if there is no available data
    */
    data_type getAccelerometer() const;

    /**
      Get a set of x, y, and z values of the current un-filtered
      gyroscope signal. Specified in <tt>degree/second</tt>.

      Valid domain of the sensor is <tt>[-500, 500] degree/second</tt>.
      Expect values outside of this domain as the system does not crop
      the sensor outputs.

      @return a three element array <tt>{x, y, z}</tt> of angular velocity
      in <tt>degree/second</tt> or zeros if there is no available data
    */
    data_type getGyroscope() const;
	  
    /**
      Get a set of x, y, and z values of the current un-filtered
      magnetometer signal. Specified in <tt>µT</tt> (microtesla).

      Domain varies with local magnetic field strength. Expect values
      on domain <tt>[-60, 60]</tt> <tt>µT</tt> (microtesla).

      @return a three element array <tt>{x, y, z}</tt> of magnetic field
      strength in <tt>µT</tt> (microtesla) or zeros if there is no
      available data
    */
    data_type getMagnetometer() const;
  }; // class SensorElement


  /**
    The Raw service provides access to the current uncalibrated, unprocessed
    sensor signals in signed integer format. The Raw service sends a map of
    <tt>N</tt> data elements. Use this class to wrap a single Raw data element
    such that we can access individual components through a simple API.

    Raw element format:
    id => [accelerometer, magnetometer, gyroscope]
    id => {ax, ay, az, mx, my, mz, gx, gy, gz}

    All sensors output 12-bit integers. Process as 16-bit short integers on
    the server side.
  */
  class RawElement : public Element<short> {
  public:
    typedef Format::Element<short>::data_type data_type;

    /** Three 3-axis channels. */
    const static std::size_t Length = 3*3;
    static std::string Name;

    /**
      Initialize this container identifier with a packed data
      array in the Raw format.

      @param data is a packed array of accelerometer, magnetometer,
      and gyroscope unprocessed signal data
      @pre <tt>data.size() == Length</tt>
    */
    RawElement(const data_type & data);

    /**
      Get a set of x, y, and z values of the current unprocessed
      accelerometer signal.

      Valid domain is <tt>[0, 4095]</tt>.

      @return a three element array <tt>{x, y, z}</tt> of raw
      accelerometer output or zeros if there is no available data
    */
    data_type getAccelerometer() const;

    /**
      Get a set of x, y, and z values of the current unprocessed
      gyroscope signal.

      Valid domain is <tt>[0, 4095]</tt>.

      @return a three element array <tt>{x, y, z}</tt> of raw
      gyroscope output or zeros if there is no available data
    */
    data_type getGyroscope() const;
	  
    /**
      Get a set of x, y, and z values of the current unprocessed
      magnetometer signal.

      Valid domain is <tt>[0, 4095]</tt>.

      @return a three element array <tt>{x, y, z}</tt> of raw
      magnetometer output or zeros if there is no available data
    */
    data_type getMagnetometer() const;
  }; // class RawElement


  /**
    Define the associative container type for PreviewElement
    entries.
  */
  typedef std::map<
    id_type,
    ConfigurableElement
  > configurable_service_type;

  /**
    Convert a range of binary data into an associative
    container (std::map) of ConfigurableElement entries.

    @pre     <tt>[first, last)</tt> is a valid range
    @return  an associative container ConfigurableElement entries
  */
  template <typename InputIterator>
  static inline configurable_service_type Configurable(InputIterator first,
                                                       InputIterator last)
  {
    return Apply<ConfigurableElement>(first, last);
  }

  /**
    Define the associative container type for PreviewElement
    entries.
  */
  typedef std::map<
    id_type,
    PreviewElement
  > preview_service_type;

  /**
    Convert a range of binary data into an associative
    container (std::map) of PreviewElement entries.

    @pre     <tt>[first, last)</tt> is a valid range
    @return  an associative container PreviewElement entries
  */
  template <typename InputIterator>
  static inline preview_service_type Preview(InputIterator first,
                                             InputIterator last)
  {
    return Apply<PreviewElement>(first, last);
  }

  /**
    Define the associative container type for SensorElement
    entries.
  */
  typedef std::map<
    id_type,
    SensorElement
  > sensor_service_type;

  /**
    Convert a range of binary data into an associative
    container (std::map) of SensorElement entries.

    @pre     <tt>[first, last)</tt> is a valid range
    @return  an associative container SensorElement entries
  */
  template <typename InputIterator>
  static inline sensor_service_type Sensor(InputIterator first,
                                           InputIterator last)
  {
    return Apply<SensorElement>(first, last);
  }

  /**
    Define the associative container type for RawElement
    entries.
  */
  typedef std::map<
    id_type,
    RawElement
  > raw_service_type;

  /**
    Convert a range of binary data into an associative
    container (std::map) of RawElement entries.

    @pre     <tt>[first, last)</tt> is a valid range
    @return  an associative container RawElement entries
  */
  template <typename InputIterator>
  static inline raw_service_type Raw(InputIterator first,
                                     InputIterator last)
  {
    return Apply<RawElement>(first, last);
  }

private:
  /**
    Convert a binary packed data representation from a MotionNode
    service into a std::map<Format::id_type, Format::*Element>. Use
    the IdToValueArray method to handle the low level message
    parsing.
  */
  template <typename T, typename InputIterator>
  static std::map<id_type,T> Apply(InputIterator first,
                                   InputIterator last)
  {
    std::map<id_type,T> result;

    {
      // Use this to do most of the dirty work.
      std::map<id_type,typename T::data_type> map =
        IdToValueArray<id_type,typename T::data_type>(first, last, T::Length);
      if (!map.empty()) {
        // Copy each [id] => vector<float> entry into the [id] => PreviewMap result map.
        // Use the range insert operator since we can statically cast a vector<float>
        // object directly into a PreviewElement. 
        result.insert(map.begin(), map.end());

        // Make sure that we were able to insert all of the elements.
        if (map.size() != result.size()) {
          result.clear();
        }
      }
    }

    return result;
  }
  
  /**
    Convert a binary packed data representation from a MotionNode
    service into a std::map<integral type, container type>.

    @pre <tt>[first, last)</tt> is a valid range
    @pre type <tt>Key</tt> is an integral type
    @pre type <tt>Value</tt> is a model of a Sequence (STL) 
  */
  template <typename Key, typename Value, typename InputIterator>
  static std::map<Key,Value> IdToValueArray(InputIterator first,
                                            InputIterator last,
                                            const std::size_t & length)
  {
    typedef typename InputIterator::difference_type difference_type;
    typedef unsigned packed_key_type;

    std::map<Key,Value> result;

    if (std::distance(first, last) > 0) {
      // Compute the size in bytes of a single element. Conside the input
      // buffer to be a packed set of element entries.
      //std::size_t element_size = 0;
      //if (length > 0) {
      //  element_size = sizeof(packed_key_type) + sizeof(typename Value::value_type) * length;
      //}

      // while we have enough bytes to create a complete element.
      InputIterator itr = first;
      while (itr != last) {
        std::pair<Key,Value> value;

        // Read the integer id for this element. Unpack the unsigned 32-bit integer
        // into our host system key type.
        if (static_cast<difference_type>(sizeof(packed_key_type)) <= std::distance(itr, last)) {
          packed_key_type key_value = *reinterpret_cast<const packed_key_type *>(&(*itr));
          std::advance(itr, sizeof(packed_key_type));

          value.first = static_cast<Key>(detail::little_endian_to_native(key_value));
        } else {
          // Not enough bytes remaining. Invalid message.
          break;
        }

        std::size_t element_length = length;
        if (0 == element_length) {
          if (static_cast<difference_type>(sizeof(packed_key_type)) <= std::distance(itr, last)) {
            packed_key_type key_value = *reinterpret_cast<const packed_key_type *>(&(*itr));
            std::advance(itr, sizeof(packed_key_type));

            element_length = static_cast<Key>(detail::little_endian_to_native(key_value));
          } else {
            // Not enough bytes remaining. Invalid message.
            break;
          }
        }

        if ((element_length > 0) && (static_cast<difference_type>(sizeof(typename Value::value_type) * element_length) <= std::distance(itr, last))) {

          // Read the array of values for this element.
          value.second.resize(element_length);
          for (typename Value::iterator value_itr=value.second.begin(); value_itr!=value.second.end(); ++value_itr) {
            *value_itr = *reinterpret_cast<const typename Value::value_type *>(&(*itr));
            std::advance(itr, sizeof(typename Value::value_type));
          }

          // Big-endian systems need to implement byte swapping here. All
          // service data is little-endian.
          std::transform(
            value.second.begin(), value.second.end(),
            value.second.begin(), &detail::little_endian_to_native<typename Value::value_type>);

          result.insert(value);
        }
      }

      // If we did not consume all of the input bytes this is an
      // invalid message.
      if (itr != last) {
        result.clear();
      }
    }

    return result;
  }

  /**
    Hide the constructor. There is no need to instantiate
    the Format object.
  */
  Format();
}; // class Format

}} // namespace MotionNode::SDK

#endif // __MOTION_NODE_SDK_FORMAT_HPP_
