/*
 * @brief sim_loc_random implements random number generators.
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SIM_LOC_RANDOM_H_INCLUDED
#define __SIM_LOC_RANDOM_H_INCLUDED

#include <boost/random.hpp>

namespace sick_scan
{
  /*!
   * class UniformRandomInteger generates uniform distributed integer random numbers.
   */
  class UniformRandomInteger
  {
  public:
    
    /*!
     * UniformRandomInteger constructor
     * @param[in] lower_bound min. value of random distribution, random numbers will be generated within the range lower_bound up to upper_bound,(lower and upper bound included)
     * @param[in] upper_bound max. value of random distribution, random numbers will be generated within the range lower_bound up to upper_bound (lower and upper bound included)
     */
    UniformRandomInteger(int lower_bound = 0, int upper_bound = 255);
    
    /*!
     * Returns a uniform distributed integer random number within the range lower_bound up to upper_bound
     * @return uniform distributed random number
     */
    int generate(void);
  
    /*!
     * Creates and returns uniform distributed binary random data of a given size
     * @param[in] data_size number of random bytes created, size of output data
     * @return binary random data of length data_size
     */
    std::vector<uint8_t> generate(int data_size);

  protected:
    
    /*
     * member data
     */

    boost::mt19937 m_random_engine; ///< mersenne twister engine
    boost::uniform_int<> m_uniform_distribution; ///< uniform integer distribution
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > m_random_generator; ///< random number generator (glues mersenne engine and distribution)
  
  }; // class UniformIntegerRandom
  
  /*!
   * class UniformRandomAsciiString generates uniform distributed ascii strings.
   */
  class UniformRandomAsciiString
  {
  public:
  
    /*!
     * UniformRandomAsciiString constructor
     */
    UniformRandomAsciiString();
  
    /*!
     * Creates and returns a random ascii string
     * @param[in] length length of string
     * @return random ascii string
     */
    std::string generate(int length);


  protected:
  
    /*
     * member data
     */
    UniformRandomInteger m_random_generator; ///< random number generator
    static const std::string s_ascii_chars;  ///< list of ascii chars: " !\"#$%&'()*+,-./0123456789:;=?@ ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_abcdefghijklmnopqrstuvwxyz{|}~"
  
  }; // class UniformRandomAsciiString
  
} // namespace sick_scan
#endif // __SIM_LOC_RANDOM_H_INCLUDED
