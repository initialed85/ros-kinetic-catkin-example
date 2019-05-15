// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file Example.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _EXAMPLE_H_
#define _EXAMPLE_H_

// TODO Poner en el contexto.

#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(Example_SOURCE)
#define Example_DllAPI __declspec( dllexport )
#else
#define Example_DllAPI __declspec( dllimport )
#endif // Example_SOURCE
#else
#define Example_DllAPI
#endif
#else
#define Example_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}

/*!
 * @brief This class represents the structure ExampleMessage defined by the user in the IDL file.
 * @ingroup EXAMPLE
 */
class ExampleMessage
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport ExampleMessage();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~ExampleMessage();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object ExampleMessage that will be copied.
     */
    eProsima_user_DllExport ExampleMessage(const ExampleMessage &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object ExampleMessage that will be copied.
     */
    eProsima_user_DllExport ExampleMessage(ExampleMessage &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object ExampleMessage that will be copied.
     */
    eProsima_user_DllExport ExampleMessage& operator=(const ExampleMessage &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object ExampleMessage that will be copied.
     */
    eProsima_user_DllExport ExampleMessage& operator=(ExampleMessage &&x);

    /*!
     * @brief This function copies the value in member first_name
     * @param _first_name New value to be copied in member first_name
     */
    inline eProsima_user_DllExport void first_name(const std::string &_first_name)
    {
        m_first_name = _first_name;
    }

    /*!
     * @brief This function moves the value in member first_name
     * @param _first_name New value to be moved in member first_name
     */
    inline eProsima_user_DllExport void first_name(std::string &&_first_name)
    {
        m_first_name = std::move(_first_name);
    }

    /*!
     * @brief This function returns a constant reference to member first_name
     * @return Constant reference to member first_name
     */
    inline eProsima_user_DllExport const std::string& first_name() const
    {
        return m_first_name;
    }

    /*!
     * @brief This function returns a reference to member first_name
     * @return Reference to member first_name
     */
    inline eProsima_user_DllExport std::string& first_name()
    {
        return m_first_name;
    }
    /*!
     * @brief This function copies the value in member last_name
     * @param _last_name New value to be copied in member last_name
     */
    inline eProsima_user_DllExport void last_name(const std::string &_last_name)
    {
        m_last_name = _last_name;
    }

    /*!
     * @brief This function moves the value in member last_name
     * @param _last_name New value to be moved in member last_name
     */
    inline eProsima_user_DllExport void last_name(std::string &&_last_name)
    {
        m_last_name = std::move(_last_name);
    }

    /*!
     * @brief This function returns a constant reference to member last_name
     * @return Constant reference to member last_name
     */
    inline eProsima_user_DllExport const std::string& last_name() const
    {
        return m_last_name;
    }

    /*!
     * @brief This function returns a reference to member last_name
     * @return Reference to member last_name
     */
    inline eProsima_user_DllExport std::string& last_name()
    {
        return m_last_name;
    }
    /*!
     * @brief This function sets a value in member age
     * @param _age New value for member age
     */
    inline eProsima_user_DllExport void age(uint8_t _age)
    {
        m_age = _age;
    }

    /*!
     * @brief This function returns the value of member age
     * @return Value of member age
     */
    inline eProsima_user_DllExport uint8_t age() const
    {
        return m_age;
    }

    /*!
     * @brief This function returns a reference to member age
     * @return Reference to member age
     */
    inline eProsima_user_DllExport uint8_t& age()
    {
        return m_age;
    }
    /*!
     * @brief This function sets a value in member score
     * @param _score New value for member score
     */
    inline eProsima_user_DllExport void score(uint32_t _score)
    {
        m_score = _score;
    }

    /*!
     * @brief This function returns the value of member score
     * @return Value of member score
     */
    inline eProsima_user_DllExport uint32_t score() const
    {
        return m_score;
    }

    /*!
     * @brief This function returns a reference to member score
     * @return Reference to member score
     */
    inline eProsima_user_DllExport uint32_t& score()
    {
        return m_score;
    }

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(const ExampleMessage& data, size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;

private:
    std::string m_first_name;
    std::string m_last_name;
    uint8_t m_age;
    uint32_t m_score;
};

#endif // _EXAMPLE_H_