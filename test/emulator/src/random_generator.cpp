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
#include <time.h>

#include "sick_scan/random_generator.h"

const std::string sick_scan::UniformRandomAsciiString::s_ascii_chars = " !\"#$%&'()*+,-./0123456789:;=?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_abcdefghijklmnopqrstuvwxyz{|}~";  ///< static list of ascii chars


/*!
 * UniformRandomInteger constructor
 * @param[in] lower_bound min. value of random distribution, random numbers will be generated within the range lower_bound up to upper_bound
 * @param[in] upper_bound max. value of random distribution, random numbers will be generated within the range lower_bound up to upper_bound
 */
sick_scan::UniformRandomInteger::UniformRandomInteger(int lower_bound, int upper_bound)
: m_random_engine(time(0)), m_uniform_distribution(lower_bound, upper_bound), m_random_generator(m_random_engine, m_uniform_distribution)
{
}

/*!
 * Returns a uniform distributed integer random number within the range lower_bound up to upper_bound
 */
int sick_scan::UniformRandomInteger::generate(void)
{
  return m_random_generator();
}

/*!
 * Creates and returns uniform distributed binary random data of a given size
 * @param[in] data_size number of random bytes created, size of output data
 * @return binary random data of length data_size
 */
std::vector<uint8_t> sick_scan::UniformRandomInteger::generate(int data_size)
{
  std::vector<uint8_t> data(data_size, 0);
  for(int n = 0; n < data_size; n++)
  {
    data[n] = (uint8_t)(m_random_generator() & 0xFF);
  }
  return data;
}

/*!
 * UniformRandomAsciiString constructor
 */
sick_scan::UniformRandomAsciiString::UniformRandomAsciiString(): m_random_generator(0, (int)(s_ascii_chars.length()) - 1)
{
}

/*!
 * Creates and returns a random ascii string
 * @param[in] length length of string
 * @return random ascii string
 */
std::string sick_scan::UniformRandomAsciiString::generate(int length)
{
  std::string random_string;
  random_string.reserve(length);
  for(size_t n = 0; n < length; n++)
    random_string.push_back(s_ascii_chars[m_random_generator.generate()]);
  return random_string;
}